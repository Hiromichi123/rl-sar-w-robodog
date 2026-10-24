#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <functional>
#include <iostream>
#include <map>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>
#include <condition_variable> 
#include <unordered_map>
#include <deque>

#pragma pack(push, 1) // 防止结构体对齐填充

struct FreqCounter {
    int count = 0;
    std::chrono::steady_clock::time_point last_time;
    std::string name;
    std::mutex mtx;

    FreqCounter(const std::string& n) : name(n) {
        last_time = std::chrono::steady_clock::now();
    }

    void tick() {
        std::lock_guard<std::mutex> lock(mtx);
        count++;

        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_time).count();

        if (elapsed >= 1000) { // 每 1 秒打印一次
            double freq = count * 1000.0 / elapsed;  // 平滑计算
            std::cout << "[" << name << "_FREQ] " << freq << " frames/s" << std::endl;

            count = 0;
            last_time = now;
        }
    }
};

typedef struct
{
    uint8_t FrameHeader;
    uint8_t CMD; // 命令
    // canDataLen: 6 bits, canIde:1, canRtr:1
    uint8_t canDataLen: 6;
    uint8_t canIde: 1;
    uint8_t canRtr: 1;
    uint32_t canId; // 电机反馈的ID
    uint8_t canData[8];
    uint8_t frameEnd;
} CAN_Receive_Frame;


typedef struct can_send_frame
{
    uint8_t FrameHeader[2]; // {0x55, 0xAA}
    uint8_t FrameLen; // 0x1e
    uint8_t CMD; // 0x03
    uint32_t sendTimes;
    uint32_t timeInterval;
    uint8_t IDType;
    uint32_t canId;
    uint8_t frameType;
    uint8_t len;
    uint8_t idAcc;
    uint8_t dataAcc;
    uint8_t data[8];
    uint8_t crc;

    can_send_frame() {
        FrameHeader[0] = 0x55; FrameHeader[1] = 0xAA;
        FrameLen = 0x1e;
        CMD = 0x03;
        sendTimes = 1;
        timeInterval = 10;
        IDType = 0;
        canId = 0x01;
        frameType = 0;
        len = 0x08;
        idAcc = 0;
        dataAcc = 0;
        memset(data, 0, 8);
        crc = 0;
    }
    void modify(uint32_t id, const uint8_t* send_data) {
        canId = id;
        std::memcpy(data, send_data, 8);
    }
} CAN_Send_Frame;

#pragma pack(pop) // 恢复默认对齐

// ========== 常量配置 ==========
static const int NUM_DEVICES = 4;         // 串口数量（默认 4）
static const int MOTORS_PER_DEVICE = 3;   // 每个串口管理的电机数
static const uint8_t MASTER_IDS[NUM_DEVICES] = {11, 12, 13, 14}; // master id 列表

// 灵足 MIT 主从模式:反馈帧固定 can_id=0xFD,电机 ID 携带在 data[0]
static constexpr uint32_t CAN_FEEDBACK_ID = 0xFD;
// 无反馈自动停发阈值:超时未收到任何反馈帧则停发命令(收到反馈自动恢复)。
// 防止电机断电/总线故障时持续发送撑爆内核 TX 队列(ENOBUFS)拖垮 candleLight 固件。
// 两种传输模式通用:usb2can 模式下也能在物理串口正常但总线无应答时止血。
static constexpr int64_t kNoFeedbackStopMs = 2000;
// 弹跳自愈最小间隔:停发时自动 down/up 接口,清除 CAN 控制器卡死的重传帧与错误
// 计数(防错误帧洪水楔死 candleLight 通道 MCU,此前只能整盒断电 10s 复位)。
// 仅 socketcan 模式(dev_node_="canX")实际执行,usb2can 模式 bounce 退化为 no-op。
static constexpr int64_t kBounceIntervalMs = 30000;

// ========== 电机类型定义 ==========
enum class MotorType {
    LINGZU_02,  // 灵足02电机
    LINGZU_03,  // 灵足03电机
    LINGZU_04   // 灵足04电机
};

// 电机参数范围配置
struct MotorLimits {
    float p_min, p_max;      // 位置范围
    float v_min, v_max;      // 速度范围
    float t_min, t_max;      // 力矩范围
    float kp_min, kp_max;    // kp范围
    float kd_min, kd_max;    // kd范围

    // 预定义的电机型号配置
    static MotorLimits getLimits(MotorType type) {
        switch (type) {
            case MotorType::LINGZU_02:
                // 灵足02: 位置±12.57rad, 速度±44rad/s, 力矩±17Nm, kp 0-500, kd 0-5
                return {-12.57f, 12.57f, -44.0f, 44.0f, -17.0f, 17.0f, 0.0f, 500.0f, 0.0f, 5.0f};
            case MotorType::LINGZU_03:
                // 灵足03: 位置±12.57rad, 速度±20rad/s, 力矩±60Nm, kp 0-5000, kd 0-100
                return {-12.57f, 12.57f, -20.0f, 20.0f, -60.0f, 60.0f, 0.0f, 5000.0f, 0.0f, 100.0f};
            case MotorType::LINGZU_04:
                // 灵足04: 位置±12.57rad, 速度±15rad/s, 力矩±120Nm, kp 0-5000, kd 0-100
                return {-12.57f, 12.57f, -15.0f, 15.0f, -120.0f, 120.0f, 0.0f, 5000.0f, 0.0f, 100.0f};
            default:
                // 默认使用灵足03配置
                return {-12.57f, 12.57f, -20.0f, 20.0f, -60.0f, 60.0f, 0.0f, 5000.0f, 0.0f, 100.0f};
        }
    }
};

// ========== 电机命令/回馈数据类型 ==========
struct MotorCommand {
    uint32_t can_id; // 电机 CAN ID（1..N）
    float position;
    float speed;
    float torque;
    float kp;
    float kd;
    MotorType motor_type; // 电机型号
    // 埋点:入队时刻(steady_clock),TX 线程弹出时据此计算队列驻留时间,
    // 评估"丢弃超过 100ms 旧命令"方案的可行性。仅统计用,不影响发送逻辑。
    std::chrono::steady_clock::time_point enqueue_time{};

    MotorCommand() : can_id(0), position(0), speed(0), torque(0), kp(0), kd(0), motor_type(MotorType::LINGZU_03) {}
};

struct MotorFeedback {
    uint32_t can_id;
    uint16_t raw_pos;
    uint16_t raw_speed;
    uint16_t raw_torque;
    uint8_t mode;       // 灵足 MIT 反馈帧无独立模式字段,恒 0(保留供结构体兼容)
    uint8_t error_code; // 灵足 MIT 反馈帧无独立错误字段,恒 0(历史教训:勿把温度低字节 data[7] 误当错误码)
    float pos_f;
    float speed_f;
    float torque_f;
    float temperature;  // data[6:7] 温度 ×0.1℃: ((data[6]&0x7F)<<8 | data[7]);data[6] bit7 为固件状态标志,非温度符号位
    MotorType motor_type; // 电机型号
    // L1 遥测真实化:最近一次收到该电机"合法且校验通过"反馈帧的时间戳。
    // 默认 epoch(1970)表示从未收到——遥测侧据此 500ms 超时推 0。
    // 控制环读取 leg_feedbacks_ 的逻辑不依赖此字段,互不影响。
    std::chrono::steady_clock::time_point last_rx_time{};
    MotorFeedback() : can_id(0), raw_pos(0), raw_speed(0), raw_torque(0),
        mode(0), error_code(0), pos_f(0), speed_f(0), torque_f(0), temperature(-1.0f),
        motor_type(MotorType::LINGZU_03) {}
};

// ========== USB2CANDevice 类（每个串口一个实例） ==========
class USB2CANDevice {
public:
    // v7.2: 统一为裸 CAN 帧回调(can_id + 8 字节载荷),传输封装由注入方负责:
    //   usb2can 模式  : motor_manager 注入的 lambda 内部组/拆 HDSC 串口帧
    //   socketcan 模式: 直接绑定 SocketCANPort::sendCan/recvCan
    // send_can_fn: int (uint32_t can_id, const uint8_t data[8])  返回>0成功
    // recv_can_fn: int (uint32_t& can_id, uint8_t data[8])       返回>0收到一帧
    struct MotorStatus {
        float last_pos = 0.0f;
        float last_speed = 0.0f;
        float last_torque = 0.0f;
        int bad_count = 0;
        bool is_healthy = true;
    };

    USB2CANDevice(const std::string& dev_node,
                  std::function<int(uint32_t, const uint8_t*)> send_can_fn,
                  std::function<int(uint32_t&, uint8_t*)> recv_can_fn,
                  int device_idx);

    ~USB2CANDevice();

    
    void start();
    void stop();
    bool is_motor_healthy(uint32_t can_id);
    void enqueue_command(const MotorCommand& cmd);
    bool get_feedback(uint32_t can_id, MotorFeedback& out);
    void set_managed_ids(const std::vector<uint32_t>& ids);
    void set_motor_type(uint32_t can_id, MotorType type);

    // L1: 某电机距上次"合法有效"反馈帧的毫秒数;从未收到返回 -1
    int64_t feedback_age_ms(uint32_t can_id);
    // L2: 本设备(整条 CAN 链路)距上次收到任意合法帧的毫秒数;0=从未收到
    int64_t device_last_rx_ms() const { return last_rx_ms_.load(); }

    std::string device_name() const { return dev_node_; }

private:
    // 线程函数
    void tx_thread_func();
    void rx_thread_func();
    // force_probe=true 时绕过"无反馈停发"闸门强制上线(仅 TX 熔断态的链路探测帧
    // 使用):熔断态下普通帧全被闸门拦下,若探测帧也被拦则永远收不到电机 0xFD 应答,
    // 形成"停发→无应答→继续停发"的永久死锁。
    bool send_motor_command_frame(const MotorCommand& cmd, bool force_probe = false);
    void process_recv_frame(uint32_t frame_can_id, const uint8_t* data);
    // 无反馈保护:弹跳 socketcan 接口(down → bitrate/restart-ms → up);
    // usb2can 串口模式无 netdev,内部校验后 no-op
    bool bounce_interface();

private:
    std::string dev_node_;
    int device_idx_;
    std::function<int(uint32_t, const uint8_t*)> send_can_fn_;
    std::function<int(uint32_t&, uint8_t*)> recv_can_fn_;
    std::atomic<bool> running_;
    std::thread tx_thread_;
    std::thread rx_thread_;

    // TX 队列(L4): 有界 + 按 can_id 覆盖。设备管理的每个 can_id 在队列中至多一份
    // 待发帧,新指令入队时原地覆盖旧值,长度天然不超过该路电机数(≤4),永不膨胀。
    std::mutex tx_mtx_;
    std::deque<MotorCommand> tx_queue_;
    std::condition_variable tx_cv_;
    // L4 发送熔断: 连续失败计数 / 是否处于熔断(链路 TX 不通)状态
    std::atomic<int> tx_fail_streak_{0};
    std::atomic<bool> tx_broken_{false};

    // latest feedback map（can_id -> feedback）
    std::mutex fb_mtx_;
    std::map<uint32_t, MotorFeedback> feedback_map_;
    std::vector<uint32_t> managed_ids_;
    std::map<uint32_t, MotorType> motor_types_; // 电机型号映射

    // L2: 本设备最近收到任意合法帧的时刻(steady_clock since epoch,ms);0=从未收到。
    // rx 线程更新,遥测线程只读。
    std::atomic<int64_t> last_rx_ms_{0};

    // 无反馈停发防护基准时刻:start() 时初始化为当前时刻,给 kNoFeedbackStopMs
    // 启动宽限(电机尚未开始回帧时不会误判停发);rx 线程每收到 0xFD 反馈帧刷新。
    std::atomic<int64_t> last_feedback_ms_{0};
    // 弹跳自愈限频时刻
    std::atomic<int64_t> last_bounce_ms_{0};

    // 发送间隔（微秒），避免一次性把太多帧塞出去造成冲突
    int tx_spacing_us_ = 100;  // 帧间间隔:100us(4电机@500Hz 占空≈20%,4×100us=400us<2ms 不丢帧)

    std::unordered_map<uint32_t, MotorStatus> motor_status_;
    std::mutex status_mtx_;
    FreqCounter tx_counter_;  // 发送频率计数器
    FreqCounter rx_counter_;  // 接收频率计数器
    
};

// ========== 管理器（管理多个设备的便利类） ==========
class USB2CANManager {
public:
    USB2CANManager() {}
    ~USB2CANManager() { stop_all(); }

    // 添加设备（返回设备索引）
    int add_device(const std::string& dev_node,
                   std::function<int(uint32_t, const uint8_t*)> send_can_fn,
                   std::function<int(uint32_t&, uint8_t*)> recv_can_fn);

    void start_all();
    void stop_all();
    bool is_motor_healthy(int device_idx, uint32_t can_id);
    // 给某个设备设置管理的电机 id
    void set_device_motor_ids(int device_idx, const std::vector<uint32_t>& ids);
    void set_motor_type(int device_idx, uint32_t can_id, MotorType type);
    void send_command_to_device(int device_idx, const MotorCommand& cmd);
    bool get_feedback(int device_idx, uint32_t can_id, MotorFeedback& out);

    // L1/L2 只读透传
    int device_count() const { return static_cast<int>(devices_.size()); }
    // 返回某设备最近收帧的绝对时刻(ms);设备号非法返回 -1
    int64_t device_last_rx_ms(int device_idx);
    // 返回某电机反馈帧"年龄"(ms),从未收到返回 -1;设备号非法返回 -1
    int64_t feedback_age_ms(int device_idx, uint32_t can_id);

private:
    std::vector<std::unique_ptr<USB2CANDevice>> devices_;
};
