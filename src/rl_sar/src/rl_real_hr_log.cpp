// ============================================================================
// C3-fix: HR 日志模块(从 rl_real_JXG.cpp 拆分)
// 职责: 高分辨率 CSV 日志(200Hz RL/20Hz 其余), 按模型名自动重命名
// 依赖: rl_real_JXG.hpp 中声明的成员变量(hr_file, hr_num_dofs, robot_name 等)
//
// v8 性能重构(方案B): RT 线程(loop_control 200Hz)内零文件 I/O、零堆分配。
// 原 HrLogPush 直接操作 ofstream: 每秒 opendir/ifstream 扫盘(27ms 尖峰根因,
// 已移至看护线程)、每 10s 强制 flush 1MB(周期性卡顿)。现拆为三线程:
//   RT 线程   : 采数 + snprintf 进栈缓冲 + 写入无锁 SPSC 环(满则丢行)
//   写盘线程  : 独占 hr_file, 出环写盘/flush/模型切换时重建文件
//   看护线程  : 每秒扫磁盘 config.yaml, 模型变化只置位原子标志
// ============================================================================
#include "rl_real_JXG.hpp"
#include <dirent.h>
#include <sys/stat.h>
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <ctime>
#include <thread>

// 模型文件名安全化:去掉 .onnx 扩展名(否则每秒循环重建文件清空数据),
// 非字母数字转下划线,截断 48 字符。
static std::string HrModelFileTag(std::string model_name) {
    size_t dot = model_name.rfind('.');
    if (dot != std::string::npos) model_name = model_name.substr(0, dot);
    for (char &ch : model_name) {
        if (!std::isalnum((unsigned char)ch)) ch = '_';
    }
    if (model_name.size() > 48) model_name = model_name.substr(0, 48);
    return model_name;
}

// 从磁盘读取当前选中的模型名:扫 policy/<robot>/<子目录>/config.yaml
std::string RL_Real::HrLogReadModelNameFromDisk()
{
    std::string model_name;
    std::string policy_dir = std::string("./policy/") + this->robot_name;
    if (DIR *dp = opendir(policy_dir.c_str()))
    {
        std::vector<std::string> cfgs;
        struct dirent *ent;
        while ((ent = readdir(dp)) != nullptr)
        {
            std::string sub = ent->d_name;
            if (sub == "." || sub == "..") continue;
            std::string cfg = policy_dir + "/" + sub + "/config.yaml";
            struct stat st;
            if (stat(cfg.c_str(), &st) == 0 && S_ISREG(st.st_mode))
                cfgs.push_back(cfg);
        }
        closedir(dp);
        std::sort(cfgs.begin(), cfgs.end());
        for (const auto &cfg : cfgs)
        {
            std::ifstream f(cfg);
            std::string line;
            while (std::getline(f, line))
            {
                size_t b = line.find_first_not_of(" \t\r\n");
                if (b == std::string::npos || line.compare(b, 11, "model_name:") != 0) continue;
                std::string v = line.substr(b + 11);
                size_t v0 = v.find_first_not_of(" \t\r\n");
                if (v0 != std::string::npos) v = v.substr(v0);
                size_t v1 = v.find_last_not_of(" \t\r\n");
                if (v1 != std::string::npos) v = v.substr(0, v1 + 1);
                if (v.size() >= 2 && ((v.front() == '"' && v.back() == '"') ||
                                      (v.front() == '\'' && v.back() == '\'')))
                    v = v.substr(1, v.size() - 2);
                model_name = v;
                break;
            }
            if (!model_name.empty()) break;
        }
    }
    return model_name;
}

// 按磁盘当前模型名重开 HR 文件并写表头/重置计时。
// 只允许独占 hr_file 的线程调用: 启动阶段=主线程, 运行期重建=写盘线程。
// 不重读 num_dofs/joint_mapping/offset(仅 HrLogInit 启动时读一次),
// 因此与 RT 线程的行格式化无数据竞争。
void RL_Real::HrLogReopenFile()
{
    auto now = std::chrono::system_clock::now();
    std::time_t tt = std::chrono::system_clock::to_time_t(now);
    std::tm tmv = *std::localtime(&tt);
    char ts[32];
    std::strftime(ts, sizeof(ts), "%Y%m%d_%H%M", &tmv);

    std::string model_name = this->HrLogReadModelNameFromDisk();
    if (model_name.empty())
        model_name = this->params.Get<std::string>("model_name");
    std::string model_tag = HrModelFileTag(model_name);

    char path[256];
    if (!model_tag.empty())
        std::snprintf(path, sizeof(path), "logs/%s_hr_%s_%s.csv",
                      this->robot_name.c_str(), model_tag.c_str(), ts);
    else
        std::snprintf(path, sizeof(path), "logs/%s_hr_%s.csv",
                      this->robot_name.c_str(), ts);

    static std::vector<char> hr_buf(1 << 18);  // 256KB 用户缓冲(仅写盘线程触碰)
    this->hr_file.rdbuf()->pubsetbuf(hr_buf.data(), static_cast<std::streamsize>(hr_buf.size()));
    this->hr_file.open(path, std::ios::out | std::ios::trunc);
    this->hr_cur_path = path;
    this->hr_named_by_model = !model_name.empty();
    if (!this->hr_file.is_open())
    {
        std::cerr << LOGGER::WARNING << "HR logger: 无法打开 " << path << std::endl;
        return;
    }

    if (this->hr_num_dofs <= 0)
        this->hr_num_dofs = this->params.Get<int>("num_of_dofs");

    this->hr_file << "t,gyro_x,gyro_y,gyro_z,acc_x,acc_y,acc_z,"
                     "quat_w,quat_x,quat_y,quat_z,cmd_x,cmd_y,cmd_z,state";
    for (int i = 0; i < this->hr_num_dofs; ++i)
        this->hr_file << ",m" << i << "_tgt_pos,m" << i << "_real_pos,m" << i
                      << "_tgt_vel,m" << i << "_real_vel,m" << i << "_real_tau,m" << i
                      << "_err,m" << i << "_resp_ms,m" << i << "_temp";
    this->hr_file << "\n";
    this->hr_file.flush();

    this->hr_t0 = std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    this->hr_rows = 0;
    std::cout << LOGGER::INFO << "HR logger 已启动(RL 状态 200Hz/其余 20Hz): " << path << std::endl;
}

void RL_Real::HrLogInit()
{
    // 配置快照只在启动时读一次;运行期模型切换重建(HrLogReopenFile)不重读,
    // 避免 RT 线程格式化行时与重建线程并发读写这些 vector
    this->hr_num_dofs = this->params.Get<int>("num_of_dofs");
    this->hr_jm = this->params.Get<std::vector<int>>("joint_mapping");
    this->hr_offset = this->params.Get<std::vector<float>>("lying_down_offset");

    this->HrLogReopenFile();
    if (!this->hr_file.is_open()) return;

    // 启动写盘线程(仅一次,重建/重复 Init 时因 joinable 守卫跳过)
    if (!this->hr_writer_thread_.joinable())
    {
        this->hr_writer_stop_.store(false);
        this->hr_writer_running_.store(true);
        this->hr_writer_thread_ = std::thread(&RL_Real::HrLogWriteLoop, this);
    }

    // 启动模型名看护线程(仅一次):每秒扫磁盘 config.yaml,模型变化时置位
    // hr_rebuild_requested_,由写盘线程完成文件重建(不进 RT 线程)。
    if (!this->hr_model_watch_thread_.joinable())
    {
        this->hr_model_watch_stop_.store(false);
        this->hr_model_watch_thread_ = std::thread(&RL_Real::HrLogModelWatch, this);
    }
}

// 非实时看护线程体: 每秒检查磁盘 config.yaml 的 model_name 是否与当前日志文件一致。
// 只置位标志,不做任何文件重建(文件操作归写盘线程的 HrLogWriteLoop)。
void RL_Real::HrLogModelWatch()
{
    const std::string fallback_name = this->params.Get<std::string>("model_name");
    while (!this->hr_model_watch_stop_.load())
    {
        // 100ms 粒度睡眠:HrLogClose 最多等 100ms 即可 join
        for (int i = 0; i < 10 && !this->hr_model_watch_stop_.load(); ++i)
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if (this->hr_model_watch_stop_.load()) break;
        if (!this->hr_log_enabled_.load()) continue;
        if (this->hr_num_dofs <= 0) continue;              // HrLogInit 尚未完成
        if (this->hr_rebuilding_.load()) continue;         // 重建中,避开 hr_cur_path
        if (this->hr_rebuild_requested_.load()) continue;  // 待写盘线程消费,避开重建窗口

        std::string disk_name = this->HrLogReadModelNameFromDisk();
        if (disk_name.empty()) disk_name = fallback_name;
        std::string base = HrModelFileTag(disk_name);
        if (!base.empty() && this->hr_cur_path.find(base) == std::string::npos)
            this->hr_rebuild_requested_.store(true);
    }
}

// 写盘线程体: 独占 hr_file。出环写盘、定期 flush、消费模型切换重建标志。
// 环空时睡 1ms;200Hz 产速下单次最多积压 ~5 行,负载可忽略。
void RL_Real::HrLogWriteLoop()
{
    uint64_t tail = this->hr_ring_tail_.load(std::memory_order_relaxed);
    double last_flush = std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();

    while (!this->hr_writer_stop_.load(std::memory_order_relaxed))
    {
        const uint64_t head = this->hr_ring_head_.load(std::memory_order_acquire);
        bool wrote = false;
        while (tail != head)
        {
            const uint64_t slot = tail & (uint64_t)(kHrRingSlots - 1);
            this->hr_file.write(this->hr_ring_rows_[slot], this->hr_ring_len_[slot]);
            ++this->hr_rows;
            ++tail;
            wrote = true;
        }
        if (wrote)
            this->hr_ring_tail_.store(tail, std::memory_order_release);

        // 模型切换重建(看护线程置位):环内残留行已全部写入旧文件,安全换文件
        if (this->hr_rebuild_requested_.exchange(false))
        {
            this->hr_rebuilding_.store(true);  // 看护线程避开 hr_cur_path
            this->hr_file.flush();
            this->hr_file.close();
            this->hr_file.clear();
            if (!this->hr_named_by_model)
                std::remove(this->hr_cur_path.c_str());
            this->HrLogReopenFile();
            this->hr_rebuilding_.store(false);
            std::cout << LOGGER::INFO << "HR logger 已按生效模型重建: "
                      << this->hr_cur_path << std::endl;
        }

        // flush 策略(写盘线程内,阻塞无碍):每 2000 行或 30s 强制落盘
        const double now = std::chrono::duration<double>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        if (this->hr_rows > 0 &&
            (this->hr_rows % 2000 == 0 || now - last_flush > 30.0))
        {
            this->hr_file.flush();
            last_flush = now;
        }

        if (!wrote)
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    this->hr_writer_running_.store(false);
}

void RL_Real::HrLogPush()
{
    if (!this->hr_log_enabled_.load()) return;
    if (!this->hr_writer_running_.load()) return;   // 写盘线程未起(启动早期/已关)
    if (this->hr_num_dofs <= 0) return;

    const auto &imu = this->robot_state.imu;
    if (imu.gyroscope.size() < 3 || imu.accelerometer.size() < 3 || imu.quaternion.size() < 4)
        return;

    // 状态名取线程安全快照(SSO 短字符串,无堆分配)
    std::string state_name = this->fsm.GetCurrentStateName();
    if (state_name.empty()) state_name = "Unknown";

    const double now = std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();

    // 限频: RL 行走态 200Hz 全速, 其余状态 20Hz
    static double s_last_write = -1.0;
    if (state_name != "RLFSMStateRLLocomotion")
    {
        if (s_last_write > 0.0 && now - s_last_write < 0.05) return;
    }
    s_last_write = now;

    static RobotMotorManager::AllMotorFeedback fb = []{
        RobotMotorManager::AllMotorFeedback a;
        a.pos.resize(16); a.vel.resize(16); a.tau.resize(16); a.temp.resize(16);
        a.error.assign(16, 0.0f);
        return a;
    }();
    motor_manager.getAllMotorFeedback(fb);
    const auto &mc = this->robot_command.motor_command;

    // 行格式化进栈缓冲: RT 线程内零堆分配、零文件 I/O。
    // 字段格式与旧 iostream 版本一致(fixed 精度 5), 列布局不变。
    char row[kHrRowMax];
    size_t rlen = 0;
    int n = std::snprintf(row, sizeof(row),
        "%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,"
        "%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%s",
        now - this->hr_t0,
        imu.gyroscope[0], imu.gyroscope[1], imu.gyroscope[2],
        imu.accelerometer[0], imu.accelerometer[1], imu.accelerometer[2],
        imu.quaternion[0], imu.quaternion[1], imu.quaternion[2], imu.quaternion[3],
        this->control.x, this->control.y, this->control.yaw,
        state_name.c_str());
    if (n < 0 || (size_t)n >= sizeof(row)) return;
    rlen = (size_t)n;

    for (int i = 0; i < this->hr_num_dofs; ++i)
    {
        if ((size_t)i >= mc.q.size() || (size_t)i >= mc.dq.size()) break;
        int idx = (i < (int)this->hr_jm.size()) ? this->hr_jm[i] : i;
        float joff = (i < (int)this->hr_offset.size()) ? this->hr_offset[i] : 0.0f;
        float rp = (fb.pos.size() > idx) ? (fb.pos[idx] + joff) : 0.0f;
        float rv = (fb.vel.size() > idx) ? fb.vel[idx] : 0.0f;
        float rt = (fb.tau.size() > idx) ? fb.tau[idx] : 0.0f;
        float er = (fb.error.size() > (size_t)idx) ? fb.error[idx] : 0.0f;
        float resp_ms = (float)motor_manager.motorFeedbackAgeMs(i);
        float mtemp = (fb.temp.size() > idx) ? fb.temp[idx] : -1.0f;
        int m = std::snprintf(row + rlen, sizeof(row) - rlen,
                              ",%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f",
                              mc.q[i], rp, mc.dq[i], rv, rt, er, resp_ms, mtemp);
        if (m < 0 || (size_t)m >= sizeof(row) - rlen) return;
        rlen += (size_t)m;
    }

    // 入环(SPLC→SPSC): 环满丢行计数,绝不阻塞实时环
    const uint64_t head = this->hr_ring_head_.load(std::memory_order_relaxed);
    const uint64_t tail = this->hr_ring_tail_.load(std::memory_order_acquire);
    if (head - tail >= (uint64_t)kHrRingSlots)
    {
        this->hr_ring_dropped_.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    const uint64_t slot = head & (uint64_t)(kHrRingSlots - 1);
    std::memcpy(this->hr_ring_rows_[slot], row, rlen);
    this->hr_ring_len_[slot] = (uint16_t)rlen;
    this->hr_ring_head_.store(head + 1, std::memory_order_release);
}

void RL_Real::HrLogClose()
{
    // 先停看护线程与写盘线程(写盘线程 join 后 hr_file 无并发访问者)
    this->hr_model_watch_stop_.store(true);
    if (this->hr_model_watch_thread_.joinable())
        this->hr_model_watch_thread_.join();
    this->hr_writer_stop_.store(true);
    if (this->hr_writer_thread_.joinable())
        this->hr_writer_thread_.join();
    this->hr_writer_running_.store(false);
    if (this->hr_file.is_open())
    {
        this->hr_file.flush();
        this->hr_file.close();
        std::cout << LOGGER::INFO << "HR logger 已关闭, 共 " << this->hr_rows << " 行"
                  << (this->hr_ring_dropped_.load() > 0
                          ? (std::string(", 环满丢弃 ") +
                             std::to_string(this->hr_ring_dropped_.load()) + " 行")
                          : std::string())
                  << std::endl;
    }
}
