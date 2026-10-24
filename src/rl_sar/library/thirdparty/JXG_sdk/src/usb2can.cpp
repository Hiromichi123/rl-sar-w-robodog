#include "usb2can.h"
#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <thread>
#include <iomanip>
#include <algorithm>
#include <cmath>

#ifdef __linux__
#include <pthread.h>
#include <sched.h>
#include <iostream>
#endif

/**
 * CAN 收发线程实时性设置:SCHED_FIFO 85(低于 1kHz IMU=90,高于 200Hz 控制环=80)。
 * 4 路 CAN(每路 tx+rx 共 8 线程)原先全绑 CPU5,8 个忙轮询线程合计约 96% CPU,
 * 踩中内核 RT 带宽 95% 红线会触发 sched_rt throttling(冻结全部 RT 线程最长 50ms)。
 * 现按设备号均分到两个大核: 设备 0/1 -> CPU5, 设备 2/3 -> CPU7,单核约 50%。
 * 无 CAP_SYS_NICE 时 EPERM 静默降级为普通调度(仅告警一次)。
 */
static void apply_can_thread_rt(int cpu)
{
#ifdef __linux__
    static bool warned = false;
    struct sched_param sp{};
    sp.sched_priority = 85;
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0 && !warned) {
        warned = true;
        std::cerr << "[usb2can] WARN: 无法设置 CAN 线程 SCHED_FIFO(EPERM),"
                     "以普通调度运行(sudo setcap cap_sys_nice+ep <bin> 可启用)" << std::endl;
    }
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(cpu, &cpuset);
    pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);
#endif
}

// 设备号 -> 绑核: 0/1 号 CAN 适配器在 CPU5, 2/3 号在 CPU7
static inline int can_device_cpu(int device_idx)
{
    return (device_idx < 2) ? 5 : 7;
}

// L1/L2 统一单调时钟:steady_clock 自 epoch 的毫秒数(与 USB2CANDevice 内时间戳同基准)
static inline int64_t can_now_ms()
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}


static int float_to_uint(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    if (x > x_max) x = x_max;
    else if (x < x_min) x = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}
static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

// ================= USB2CANDevice 实现 =================

USB2CANDevice::USB2CANDevice(const std::string& dev_node,
                             std::function<int(uint32_t, const uint8_t*)> send_can_fn,
                             std::function<int(uint32_t&, uint8_t*)> recv_can_fn,
                             int device_idx)
    : dev_node_(dev_node),
      device_idx_(device_idx),
      send_can_fn_(std::move(send_can_fn)),
      recv_can_fn_(std::move(recv_can_fn)),
      running_(false),
      tx_counter_(dev_node + "_TX"),
      rx_counter_(dev_node + "_RX")
{
}

USB2CANDevice::~USB2CANDevice()
{
    stop();
}

void USB2CANDevice::start()
{
    if (running_.load()) return;
    running_.store(true);
    // 无反馈停发防护基准:启动后 kNoFeedbackStopMs 内无条件允许发送。
    // 此时电机可能尚未上电/使能、反馈帧还没开始回流,避免开机即误判停发。
    last_feedback_ms_.store(can_now_ms());
    tx_thread_ = std::thread(&USB2CANDevice::tx_thread_func, this);
    rx_thread_ = std::thread(&USB2CANDevice::rx_thread_func, this);
    //std::cout << "[start] 发送/接收线程已启动" << std::endl; // 新增
}

void USB2CANDevice::stop()
{
    if (!running_.load()) return;
    running_.store(false);

    // notify tx thread in case it's waiting
    tx_cv_.notify_all();

    // 等待发送线程退出
    if (tx_thread_.joinable()) {
        tx_thread_.join();
    }

    // 等待接收线程退出
    if (rx_thread_.joinable()) {
        rx_thread_.join();
    }
}

void USB2CANDevice::enqueue_command(const MotorCommand& cmd)
{
    // 埋点:入队前打时间戳(覆盖入队时也刷新为最新时刻)
    MotorCommand c = cmd;
    c.enqueue_time = std::chrono::steady_clock::now();
    {
        std::lock_guard<std::mutex> lk(tx_mtx_);
        // L4: 电机指令是"状态"而非"事件"。若队列中已有同 can_id 待发帧,原地覆盖为
        // 最新值(保留排队位置),保证每个 can_id 至多一份、队列长度有界(≤该路电机数)。
        bool replaced = false;
        for (auto& q : tx_queue_) {
            if (q.can_id == c.can_id) { q = c; replaced = true; break; }
        }
        if (!replaced) tx_queue_.push_back(c);
    }
    tx_cv_.notify_one();
}

bool USB2CANDevice::get_feedback(uint32_t can_id, MotorFeedback& out)
{
    std::lock_guard<std::mutex> lk(fb_mtx_);
    auto it = feedback_map_.find(can_id);
    if (it == feedback_map_.end()) return false;
    out = it->second;
    return true;
}

// L1: 返回某电机距上次合法有效反馈帧的毫秒数;从未收到返回 -1
int64_t USB2CANDevice::feedback_age_ms(uint32_t can_id)
{
    std::lock_guard<std::mutex> lk(fb_mtx_);
    auto it = feedback_map_.find(can_id);
    if (it == feedback_map_.end()) return -1;
    const auto& t = it->second.last_rx_time;
    if (t.time_since_epoch().count() == 0) return -1;  // 从未收到
    auto age = std::chrono::steady_clock::now() - t;
    return std::chrono::duration_cast<std::chrono::milliseconds>(age).count();
}

void USB2CANDevice::set_managed_ids(const std::vector<uint32_t>& ids)
{
    std::lock_guard<std::mutex> lk(fb_mtx_);
    managed_ids_ = ids;
    for (auto id : ids) {
        if (feedback_map_.find(id) == feedback_map_.end()) {
            feedback_map_[id] = MotorFeedback();
            feedback_map_[id].can_id = id;
        }
        // 如果没有设置电机型号，默认使用LINGZU_03
        if (motor_types_.find(id) == motor_types_.end()) {
            motor_types_[id] = MotorType::LINGZU_03;
        }
        feedback_map_[id].motor_type = motor_types_[id];
    }
}

void USB2CANDevice::set_motor_type(uint32_t can_id, MotorType type) {
    std::lock_guard<std::mutex> lk(fb_mtx_);
    motor_types_[can_id] = type;
    // 如果该电机的feedback已经存在，更新其motor_type
    if (feedback_map_.find(can_id) != feedback_map_.end()) {
        feedback_map_[can_id].motor_type = type;
    }
}

// -------- TX 线程函数：从队列弹出并发送 ----------
// 传输封装已下移到 motor_manager 注入的回调(usb2can=HDSC 串口帧 / socketcan=裸 CAN 帧),
// 此处仅组 MIT 8 字节载荷并调 send_can_fn_。
void USB2CANDevice::tx_thread_func()
{
    apply_can_thread_rt(can_device_cpu(device_idx_));
    // L4: 连续发送失败达到该次数即进入熔断(约几十~上百 ms,取决于帧率)
    constexpr int kTxFailThreshold = 100;
    // 熔断态探测/退避节拍:不按 60us 空转,降到 10ms,降低无效 write 负载
    constexpr auto kBrokenBackoff = std::chrono::milliseconds(10);

    // ── 埋点:TX 队列驻留时间统计(单线程写,无需加锁) ──
    // 评估"丢弃超过 100ms 旧命令"是否可行:观察正常/断线场景下队列驻留分布。
    uint64_t stat_total = 0;       // 本窗口弹出帧数
    uint64_t stat_stale100 = 0;    // 驻留 >100ms 的帧数(若丢弃方案启用,这些会被丢)
    uint64_t stat_broken = 0;      // 熔断态弹出的探测帧数
    uint64_t stat_max_us = 0;      // 本窗口最大驻留时间(us)
    uint64_t b_lt1ms = 0, b_1_10ms = 0, b_10_50ms = 0, b_50_100ms = 0, b_gt100ms = 0;
    auto stat_last_print = std::chrono::steady_clock::now();

    while (running_.load()) {
        MotorCommand cmd;
        {
            std::unique_lock<std::mutex> lk(tx_mtx_);
            if (tx_queue_.empty()) {
                tx_cv_.wait_for(lk, std::chrono::milliseconds(50));
                if (!running_.load()) break;
                continue;
            }
            cmd = tx_queue_.front();
            tx_queue_.pop_front();
        }

        // 埋点:计算队列驻留时间并更新统计
        auto now = std::chrono::steady_clock::now();
        uint64_t age_us = 0;
        if (cmd.enqueue_time.time_since_epoch().count() != 0) {
            age_us = (uint64_t)std::chrono::duration_cast<std::chrono::microseconds>(now - cmd.enqueue_time).count();
        }
        stat_total++;
        if (tx_broken_.load()) stat_broken++;
        if (age_us > stat_max_us) stat_max_us = age_us;
        if (age_us < 1000) b_lt1ms++;
        else if (age_us < 10000) b_1_10ms++;
        else if (age_us < 50000) b_10_50ms++;
        else if (age_us < 100000) b_50_100ms++;
        else { b_gt100ms++; stat_stale100++; }

        bool ok = send_motor_command_frame(cmd, tx_broken_.load() /*熔断态强制上线探测*/);

        if (tx_broken_.load()) {
            // ── 熔断态:本帧作为链路探测(真实上线,见 force_probe) ──
            // 恢复判据必须是"电机真的回了 0xFD",不能只看 write 成功:
            // socketcan 死总线上帧先进 qdisc,write 恒成功,随后才异步 ENOBUFS,
            // 只看 write 会假恢复→普通帧洪流→2s 后闸门再关→熔断,周而复始。
            // 探测 10ms 一拍,电机上电/总线恢复后通常 1~2 拍内即回帧,恢复延迟 ~10ms。
            const bool feedback_live =
                can_now_ms() - last_feedback_ms_.load() <= 300;
            if (ok && feedback_live) {
                // 链路恢复:清空全部待发帧(可能是拔线期间堆积的旧周期指令),
                // 从下一控制周期的最新帧重新开始,杜绝过期指令补发
                {
                    std::lock_guard<std::mutex> lk(tx_mtx_);
                    tx_queue_.clear();
                }
                tx_fail_streak_.store(0);
                tx_broken_.store(false);
                std::cerr << "[" << dev_node_ << "] CAN TX 链路恢复(已收到反馈帧),"
                             "已清空待发队列" << std::endl;
            } else {
                // 仍不通:探测帧放回队首(若期间已有同 id 更新帧入队,则丢弃本旧帧)
                {
                    std::lock_guard<std::mutex> lk(tx_mtx_);
                    bool newer_exists = false;
                    for (auto& q : tx_queue_)
                        if (q.can_id == cmd.can_id) { newer_exists = true; break; }
                    if (!newer_exists) tx_queue_.push_front(cmd);
                }
                std::this_thread::sleep_for(kBrokenBackoff);
            }
            // 埋点:每秒统计队列驻留。正常链路(stale=0/broken=0)无信息量,静默;
            // 仅在熔断态等出现异常时才打印,避免每秒刷屏。计数每秒照常重置。
            if (now - stat_last_print >= std::chrono::seconds(1)) {
                if (stat_stale100 > 0 || stat_broken > 0) {
                    fprintf(stderr,
                        "[TX_STAT %s] total=%llu stale>100ms=%llu(%.1f%%) broken=%llu max=%.1fms | "
                        "<1ms:%llu 1-10ms:%llu 10-50ms:%llu 50-100ms:%llu >100ms:%llu\n",
                        dev_node_.c_str(),
                        (unsigned long long)stat_total,
                        (unsigned long long)stat_stale100,
                        stat_total ? 100.0 * stat_stale100 / stat_total : 0.0,
                        (unsigned long long)stat_broken,
                        stat_max_us / 1000.0,
                        (unsigned long long)b_lt1ms, (unsigned long long)b_1_10ms,
                        (unsigned long long)b_10_50ms, (unsigned long long)b_50_100ms,
                        (unsigned long long)b_gt100ms);
                }
                stat_total = stat_stale100 = stat_broken = stat_max_us = 0;
                b_lt1ms = b_1_10ms = b_10_50ms = b_50_100ms = b_gt100ms = 0;
                stat_last_print = now;
            }
            continue;
        }

        if (ok) {
            // 正常发送:清零连续失败计数
            if (tx_fail_streak_.load(std::memory_order_relaxed) != 0)
                tx_fail_streak_.store(0);
        } else {
            // 连续失败累计;达到阈值熔断。熔断后停止从队列取帧正常发送,
            // 入队仍按 can_id 覆盖刷新(队列有界不膨胀),等待上面的探测恢复。
            int n = tx_fail_streak_.fetch_add(1) + 1;
            if (n >= kTxFailThreshold) {
                tx_broken_.store(true);
                std::cerr << "[" << dev_node_ << "] CAN TX 连续 " << n
                          << " 帧发送失败,进入熔断(停止发送,等待恢复)" << std::endl;
            }
        }

        // 埋点:每秒统计队列驻留。正常链路(stale=0/broken=0)无信息量,静默;
        // 仅在出现 >100ms 延迟帧或熔断时才打印,避免每秒刷屏。计数每秒照常重置。
        if (now - stat_last_print >= std::chrono::seconds(1)) {
            if (stat_stale100 > 0 || stat_broken > 0) {
                fprintf(stderr,
                    "[TX_STAT %s] total=%llu stale>100ms=%llu(%.1f%%) broken=%llu max=%.1fms | "
                    "<1ms:%llu 1-10ms:%llu 10-50ms:%llu 50-100ms:%llu >100ms:%llu\n",
                    dev_node_.c_str(),
                    (unsigned long long)stat_total,
                    (unsigned long long)stat_stale100,
                    stat_total ? 100.0 * stat_stale100 / stat_total : 0.0,
                    (unsigned long long)stat_broken,
                    stat_max_us / 1000.0,
                    (unsigned long long)b_lt1ms, (unsigned long long)b_1_10ms,
                    (unsigned long long)b_10_50ms, (unsigned long long)b_50_100ms,
                    (unsigned long long)b_gt100ms);
            }
            stat_total = stat_stale100 = stat_broken = stat_max_us = 0;
            b_lt1ms = b_1_10ms = b_10_50ms = b_50_100ms = b_gt100ms = 0;
            stat_last_print = now;
        }

        std::this_thread::sleep_for(std::chrono::microseconds(tx_spacing_us_));
    }
}

bool USB2CANDevice::is_motor_healthy(uint32_t can_id) {
    std::lock_guard<std::mutex> lk(status_mtx_);
    auto it = motor_status_.find(can_id);
    if (it == motor_status_.end()) {
        return false; // 未记录的电机视为不健康
    }
    return it->second.is_healthy;
}


bool USB2CANDevice::send_motor_command_frame(const MotorCommand& cmd, bool force_probe)
{
    // ── 无反馈自动停发闸门(usb2can/socketcan 两种模式通用) ──
    // 电机断电/总线断线时无节点 ACK:
    //   socketcan: write 先进内核队列后异步 ENOBUFS,持续发送撑爆队列并楔死
    //              candleLight 通道固件(曾只能整盒断电 10s 复位);
    //   usb2can : 串口 write 虽仍成功,但向死链路灌帧同样无意义且掩盖故障。
    // 超过 kNoFeedbackStopMs 未收到任何 0xFD 反馈帧即停发(返回 false 计入 TX
    // 熔断,发送节拍自动降到 10ms 探测),rx 线程一旦重新收到反馈帧立即自动恢复。
    //
    // force_probe(熔断态探测帧)必须绕过本闸门:闸门关闭后普通帧不上线,电机就
    // 收不到任何控制帧,自然不会回 0xFD——若探测帧也被拦,链路即使恢复也永远
    // 等不到反馈(0xFC 等配置帧不保证回标准 0xFD 状态帧),形成永久死锁。
    if (!force_probe && can_now_ms() - last_feedback_ms_.load() > kNoFeedbackStopMs) {
        static int64_t last_warn_ms = 0;  // 仅限频日志用,多线程竞争无害
        const int64_t now = can_now_ms();
        // 弹跳接口:down/up 清除 CAN 控制器卡死的重传帧与错误计数。30s 限频;
        // 弹跳后不假设成功,仍保持停发,必须看到真实反馈帧才恢复。
        // usb2can 串口模式 bounce_interface() 白名单校验不通过,静默 no-op。
        if (now - last_bounce_ms_.load() >= kBounceIntervalMs) {
            last_bounce_ms_.store(now);
            bounce_interface();
        }
        if (now - last_warn_ms >= 1000) {
            last_warn_ms = now;
            std::cerr << "[" << dev_node_ << "] 超过 " << kNoFeedbackStopMs
                      << "ms 无反馈，暂停发送（收到反馈帧后自动恢复）" << std::endl;
        }
        return false;
    }

    // 根据电机型号获取参数范围
    MotorLimits limits = MotorLimits::getLimits(cmd.motor_type);

    // 位置 16bit，速度 12bit，kp 12bit，kd 12bit，torque 12bit
    int p = float_to_uint(cmd.position, limits.p_min, limits.p_max, 16);
    int v = float_to_uint(cmd.speed, limits.v_min, limits.v_max, 12);
    int kp = float_to_uint(cmd.kp, limits.kp_min, limits.kp_max, 12);
    int kd = float_to_uint(cmd.kd, limits.kd_min, limits.kd_max, 12);
    int tq = float_to_uint(cmd.torque, limits.t_min, limits.t_max, 12);

    uint8_t data[8] = {0};
    data[0] = (p >> 8) & 0xFF;
    data[1] = p & 0xFF;
    data[2] = (v >> 4) & 0xFF;
    data[3] = ((v & 0xF) << 4) | ((kp >> 8) & 0xF);
    data[4] = kp & 0xFF;
    data[5] = (kd >> 4) & 0xFF;
    data[6] = ((kd & 0xF) << 4) | ((tq >> 8) & 0xF);
    data[7] = tq & 0xFF;

    return send_can_fn_(static_cast<uint32_t>(cmd.can_id), data) > 0;
}

// -------- 无反馈保护:弹跳接口(down → bitrate/restart-ms → up) ----------
// 命令序列与 scripts/can_setup.sh 单接口配置一致。system() 在 tx 线程阻塞数百
// ms 可接受(该路已 2s 无反馈,且熔断态本就 10ms 一拍探测)。弹跳只复位宿主侧
// 控制器状态、清 qdisc/重传队列,不影响适配器供电。
// 仅 socketcan 模式实际执行:socketcan 分支 dev_node_ 即内核接口名("can0"~"can3");
// usb2can 模式 dev_node_="/dev/ttycanX",白名单校验不过直接 no-op。
bool USB2CANDevice::bounce_interface()
{
    // 严格白名单:必须是 "can" + 纯数字(防 dev_node_ 异常时命令注入/误操作其他接口)
    if (dev_node_.size() < 4 || dev_node_.compare(0, 3, "can") != 0) return false;
    for (size_t i = 3; i < dev_node_.size(); ++i) {
        if (dev_node_[i] < '0' || dev_node_[i] > '9') return false;
    }

    const std::string cmd =
        "ip link set " + dev_node_ + " down 2>/dev/null; "
        "ip link set " + dev_node_ + " type can bitrate 1000000 restart-ms 100; "
        "ip link set " + dev_node_ + " txqueuelen 100; "
        "ip link set " + dev_node_ + " up";
    const int rc = std::system(cmd.c_str());
    if (rc != 0) {
        std::cerr << "[" << dev_node_ << "] 弹跳接口失败 rc=" << rc
                  << "(接口可能已僵尸,需重新枚举后重跑 can_setup.sh)" << std::endl;
        return false;
    }
    std::cerr << "[" << dev_node_ << "] 接口弹跳完成,等待反馈恢复" << std::endl;
    return true;
}

// -------- RX 线程函数：阻塞接收并解析更新回馈 ----------
void USB2CANDevice::rx_thread_func()
{
    apply_can_thread_rt(can_device_cpu(device_idx_));
    while (running_.load()) {
        uint32_t can_id = 0;
        uint8_t data[8] = {0};
        int r = recv_can_fn_(can_id, data);
        if (r <= 0) {
            continue;
        }

        // 灵足主从模式:反馈帧固定 can_id=0xFD,电机 ID 在 data[0];
        // 其他 can_id 的帧(使能应答/保留帧等)仅限频提示,不刷新停发基准——
        // 基准只认物理反馈流,与上层单电机量程校验完全解耦。
        if (can_id != CAN_FEEDBACK_ID) {
            static int64_t last_unknown_ms = 0;  // 仅限频日志,多线程竞争无害
            const int64_t now = can_now_ms();
            if (now - last_unknown_ms >= 1000) {
                last_unknown_ms = now;
                std::cerr << "[" << dev_node_ << "] 收到非反馈帧 can_id=0x"
                          << std::hex << can_id << std::dec << "(忽略)" << std::endl;
            }
            continue;
        }

        // 先刷新"链路活着"基准,再解析:即使后续某台电机量程校验不过,
        // 也不影响本路 TX 闸门的恢复判定(数据不可用 ≠ 链路断)。
        last_feedback_ms_.store(can_now_ms());
        process_recv_frame(can_id, data);
    }

    std::cout << "[" << dev_node_ << "] rx_thread exit\n";
}

void USB2CANDevice::process_recv_frame(uint32_t frame_can_id, const uint8_t* frame_data)
{
    (void)frame_can_id;  // 灵足反馈帧:电机 ID 在载荷 data[0],帧 can_id 固定 0xFD,不使用
    MotorFeedback fb;
    fb.can_id = frame_data[0];
    uint32_t motor_id = fb.can_id;

    {
        std::lock_guard<std::mutex> lk(fb_mtx_);
        if (std::find(managed_ids_.begin(), managed_ids_.end(), motor_id) == managed_ids_.end()) {
            return; // 忽略未管理的ID
        }
        fb.motor_type = motor_types_[motor_id];
    }

    // L2: 收到一帧"帧头/帧尾合法且属于受管电机"的反馈,即说明本路 RX 链路活着。
    // 与数据量程校验结果无关(校验不过只表示该帧值不可用,不代表链路断)。
    last_rx_ms_.store(can_now_ms(), std::memory_order_relaxed);

    // 根据电机型号获取参数范围
    MotorLimits limits = MotorLimits::getLimits(fb.motor_type);

    const uint8_t* data = frame_data;  // 裸帧回调:frame_data 即 8 字节 MIT 反馈载荷

    uint16_t q_uint = (uint16_t(data[1]) << 8) | data[2];  // 位置：data[1]高8位 + data[2]低8位
    uint16_t dq_uint = (uint16_t(data[3]) << 4) | (data[4] >> 4);  // 速度：data[3]高4位 + data[4]高4位
    uint16_t tau_uint = (uint16_t(data[4] & 0x0F) << 8) | data[5];  // 扭矩：data[4]低4位 + data[5]低8位

    // data[6:7] = 电机温度 ×0.1℃(大端,低 11bit 无符号)。两轮实测证据:
    //   正常时: 大电机 data[6]=0x81、髋/足轮=0x01/0x00,换算温度均 25~36℃;
    //   断电恢复后: 大电机 data[6] 翻转为 0x21(bit7 清、bit5 置),同期 data[7]
    //   自然 +0~40 raw(温升 0~4℃),用 &0x07 换算仍得 26~38℃ 合理值。
    // 结论: data[6] 的 bit3~bit7 是固件运行状态标志(使能/故障等,断电会翻转),
    //   温度高字节仅占 bit2:bit0(11bit 量程 0~204.7℃,覆盖电机过温阈值)。
    //   mask 必须用 0x07:用 0x7F 会在 bit5 置位时读成 851℃ 触发假过温报警。
    // 灵足 MIT 反馈帧无独立 mode/error 字段(历史教训:曾把 data[7] 温度低字节
    // 如 25.0℃ 的 0xFA 误报为错误码),mode/error_code 恒置 0。
    const uint16_t temp_raw =
        static_cast<uint16_t>((static_cast<uint16_t>(data[6]) & 0x07) << 8 | data[7]);
    fb.temperature = static_cast<float>(temp_raw) * 0.1f;
    fb.mode = 0;
    fb.error_code = 0;

    // 转换为浮点值
    fb.pos_f = uint_to_float(q_uint, limits.p_min, limits.p_max, 16);
    fb.speed_f = uint_to_float(dq_uint, limits.v_min, limits.v_max, 12);
    fb.torque_f = uint_to_float(tau_uint, limits.t_min, limits.t_max, 12);

    // 数据校验参数
    static constexpr float LIMIT_TOLERANCE = 1.05f; // 量程 tolerance
    auto is_finite = [](float v)->bool { return std::isfinite(v); };
    bool data_ok = true;

    // 有限值检查
    if (!is_finite(fb.pos_f) || !is_finite(fb.speed_f) || !is_finite(fb.torque_f)) {
        data_ok = false;
    }

    // 量程超限检查
    if (data_ok) {
        if (std::abs(fb.pos_f) > limits.p_max * LIMIT_TOLERANCE ||
            std::abs(fb.speed_f) > limits.v_max * LIMIT_TOLERANCE ||
            std::abs(fb.torque_f) > limits.t_max * LIMIT_TOLERANCE) {
            data_ok = false;
        }
    }

    // 更新电机状态和反馈数据
    {
        std::lock_guard<std::mutex> lk(status_mtx_);
        // 初始化未记录的电机状态
        if (motor_status_.find(motor_id) == motor_status_.end()) {
            motor_status_[motor_id] = MotorStatus();
        }
        MotorStatus& status = motor_status_[motor_id];

        if (data_ok) {
            // 数据正常，更新反馈映射表
            fb.last_rx_time = std::chrono::steady_clock::now();  // L1: 有效帧打戳
            std::lock_guard<std::mutex> fb_lk(fb_mtx_);
            feedback_map_[motor_id] = fb;
            status.bad_count = 0;
            status.is_healthy = true;
        } else {
            status.bad_count++;
            if (status.bad_count >= 5) {
                status.is_healthy = false;
            }
        }

        // 无论数据是否通过校验，都更新历史值，避免死锁
        status.last_pos = fb.pos_f;
        status.last_speed = fb.speed_f;
        status.last_torque = fb.torque_f;
    }

    if (data_ok) {
        // std::cout << "[接收解析] ID=" << motor_id
        //           << ", 位置=" << fb.pos_f
        //           << ", 速度=" << fb.speed_f
        //           << ", 扭矩=" << fb.torque_f << std::endl;
    }
}

// ================= USB2CANManager 实现 =================

int USB2CANManager::add_device(const std::string& dev_node,
                   std::function<int(uint32_t, const uint8_t*)> send_can_fn,
                   std::function<int(uint32_t&, uint8_t*)> recv_can_fn)
{
    int idx = (int)devices_.size();
    devices_.emplace_back(new USB2CANDevice(dev_node, std::move(send_can_fn), std::move(recv_can_fn), idx));
    return idx;
}

void USB2CANManager::start_all()
{
    for (auto& d : devices_) {
        d->start();
    }
}

void USB2CANManager::stop_all()
{
    for (auto& d : devices_) {
        d->stop();
    }
}

bool USB2CANManager::is_motor_healthy(int device_idx, uint32_t can_id) {
    if (device_idx < 0 || device_idx >= (int)devices_.size()) return false;
    return devices_[device_idx]->is_motor_healthy(can_id);
}

void USB2CANManager::set_device_motor_ids(int device_idx, const std::vector<uint32_t>& ids)
{
    if (device_idx < 0 || device_idx >= (int)devices_.size()) return;
    devices_[device_idx]->set_managed_ids(ids);
}

void USB2CANManager::set_motor_type(int device_idx, uint32_t can_id, MotorType type) {
    if (device_idx < 0 || device_idx >= (int)devices_.size()) return;
    devices_[device_idx]->set_motor_type(can_id, type);
}

void USB2CANManager::send_command_to_device(int device_idx, const MotorCommand& cmd)
{
    if (device_idx < 0 || device_idx >= (int)devices_.size()) return;
    devices_[device_idx]->enqueue_command(cmd);
}

bool USB2CANManager::get_feedback(int device_idx, uint32_t can_id, MotorFeedback& out)
{
    if (device_idx < 0 || device_idx >= (int)devices_.size()) return false;
    return devices_[device_idx]->get_feedback(can_id, out);
}

int64_t USB2CANManager::device_last_rx_ms(int device_idx)
{
    if (device_idx < 0 || device_idx >= (int)devices_.size()) return -1;
    return devices_[device_idx]->device_last_rx_ms();
}

int64_t USB2CANManager::feedback_age_ms(int device_idx, uint32_t can_id)
{
    if (device_idx < 0 || device_idx >= (int)devices_.size()) return -1;
    return devices_[device_idx]->feedback_age_ms(can_id);
}
