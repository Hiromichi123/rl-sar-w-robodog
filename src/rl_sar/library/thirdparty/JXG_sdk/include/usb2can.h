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

    MotorCommand() : can_id(0), position(0), speed(0), torque(0), kp(0), kd(0), motor_type(MotorType::LINGZU_03) {}
};

struct MotorFeedback {
    uint32_t can_id;
    uint16_t raw_pos;
    uint16_t raw_speed;
    uint16_t raw_torque;
    uint16_t raw_temp;  // 绕组温度原始值：温度(°C) x10
    float temperature;  // 绕组温度(°C)，MIT反馈帧 data[6..7]
    float pos_f;
    float speed_f;
    float torque_f;
    MotorType motor_type; // 电机型号
    MotorFeedback() : can_id(0), raw_pos(0), raw_speed(0), raw_torque(0),
        raw_temp(0), temperature(0), pos_f(0), speed_f(0), torque_f(0), motor_type(MotorType::LINGZU_03) {}
};

// ========== USB2CANDevice 类（每个串口一个实例） ==========
class USB2CANDevice {
public:
    // send_fn: int send(const uint8_t* buf, size_t len)
    // recv_fn: int recv(uint8_t* buf, uint8_t header, size_t len)
    struct MotorStatus {
        float last_pos = 0.0f;
        float last_speed = 0.0f;
        float last_torque = 0.0f;
        int bad_count = 0;
        bool is_healthy = true;
    };

    USB2CANDevice(const std::string& dev_node,
                  std::function<int(const uint8_t*, size_t)> send_fn,
                  std::function<int(uint8_t*, uint8_t, size_t)> recv_fn,
                  int device_idx);

    ~USB2CANDevice();

    
    void start();
    void stop();
    bool is_motor_healthy(uint32_t can_id);
    void enqueue_command(const MotorCommand& cmd);
    bool get_feedback(uint32_t can_id, MotorFeedback& out);
    void set_managed_ids(const std::vector<uint32_t>& ids);
    void set_motor_type(uint32_t can_id, MotorType type);

    std::string device_name() const { return dev_node_; }

private:
    // 线程函数
    void tx_thread_func();
    void rx_thread_func();
    bool send_motor_command_frame(const MotorCommand& cmd);
    void process_recv_frame(const CAN_Receive_Frame& frame);

private:
    std::string dev_node_;
    int device_idx_;
    std::function<int(const uint8_t*, size_t)> send_fn_;
    std::function<int(uint8_t*, uint8_t, size_t)> recv_fn_;
    std::atomic<bool> running_;
    std::thread tx_thread_;
    std::thread rx_thread_;

    // TX 队列
    std::mutex tx_mtx_;
    std::queue<MotorCommand> tx_queue_;
    std::condition_variable tx_cv_;

    // latest feedback map（can_id -> feedback）
    std::mutex fb_mtx_;
    std::map<uint32_t, MotorFeedback> feedback_map_;
    std::vector<uint32_t> managed_ids_;
    std::map<uint32_t, MotorType> motor_types_; // 电机型号映射

    // 发送间隔（微秒），避免一次性把太多帧塞出去造成冲突
    int tx_spacing_us_ = 60;

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
                   std::function<int(const uint8_t*, size_t)> send_fn,
                   std::function<int(uint8_t*, uint8_t, size_t)> recv_fn);

    void start_all();
    void stop_all();
    bool is_motor_healthy(int device_idx, uint32_t can_id);
    // 给某个设备设置管理的电机 id
    void set_device_motor_ids(int device_idx, const std::vector<uint32_t>& ids);
    void set_motor_type(int device_idx, uint32_t can_id, MotorType type);
    void send_command_to_device(int device_idx, const MotorCommand& cmd);
    bool get_feedback(int device_idx, uint32_t can_id, MotorFeedback& out);

private:
    std::vector<std::unique_ptr<USB2CANDevice>> devices_;
};
