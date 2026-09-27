#include "usb2can.h"
#include <cassert>
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
                             std::function<int(const uint8_t*, size_t)> send_fn,
                             std::function<int(uint8_t*, uint8_t, size_t)> recv_fn,
                             int device_idx)
    : dev_node_(dev_node),
      device_idx_(device_idx),
      send_fn_(send_fn),
      recv_fn_(recv_fn),
      running_(false),
      tx_counter_(dev_node + "_TX"),  
      rx_counter_(dev_node + "_RX")  
{
    // std::cout << "CAN_Send_Frame 大小: " << sizeof(CAN_Send_Frame) << " 字节（预期 30 字节）" << std::endl;
}

USB2CANDevice::~USB2CANDevice()
{
    stop();
}

void USB2CANDevice::start()
{
    if (running_.load()) return;
    running_.store(true);
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
    {
        std::lock_guard<std::mutex> lk(tx_mtx_);
        tx_queue_.push(cmd);
        //std::cout << "[enqueue] CAN ID=" << cmd.can_id << " 入队，队列大小=" << tx_queue_.size() << std::endl; // 新增
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
// 发送封装使用 CAN_Send_Frame
// 直接使用 send_fn_ 发送原始字节
void USB2CANDevice::tx_thread_func()
{
    apply_can_thread_rt(can_device_cpu(device_idx_));
    //std::cout << "[" << dev_node_ << "] tx_thread 启动" << std::endl; // 新增
    while (running_.load()) {
        MotorCommand cmd;
        {
            std::unique_lock<std::mutex> lk(tx_mtx_);
            if (tx_queue_.empty()) {
               // std::cout << "[" << dev_node_ << "] tx_queue 为空，等待命令..." << std::endl; // 新增
                tx_cv_.wait_for(lk, std::chrono::milliseconds(50));
                if (!running_.load()) break;
                continue;
            }
            cmd = tx_queue_.front();
            tx_queue_.pop();
            //std::cout << "[" << dev_node_ << "] 取出命令,CAN ID=" << cmd.can_id << std::endl; // 新增
        }

        // 发送该命令
        bool ok = send_motor_command_frame(cmd);
        // tx_counter.tick();
        
        if (!ok) {
           // std::cerr << "[" << dev_node_ << "] send frame failed for id=" << cmd.can_id << std::endl;
        } else {
            // std::cout << "[" << dev_node_ << "] 发送成功,CAN ID=" << cmd.can_id << std::endl; // 新增
        }

        std::this_thread::sleep_for(std::chrono::microseconds(tx_spacing_us_));
    }

   // std::cout << "[" << dev_node_ << "] tx_thread exit\n";
}

bool USB2CANDevice::is_motor_healthy(uint32_t can_id) {
    std::lock_guard<std::mutex> lk(status_mtx_);
    auto it = motor_status_.find(can_id);
    if (it == motor_status_.end()) {
        return false; // 未记录的电机视为不健康
    }
    return it->second.is_healthy;
}


bool USB2CANDevice::send_motor_command_frame(const MotorCommand& cmd)
{
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

    // 构造 CAN_Send_Frame
    CAN_Send_Frame frame;
    frame.modify(static_cast<uint32_t>(cmd.can_id), data);
    size_t frame_size = sizeof(CAN_Send_Frame);
   // std::cout << "发送的帧长度: " << frame_size << " 字节,send_fn_ 调用时使用该长度" << std::endl;

    int bytes = send_fn_(reinterpret_cast<const uint8_t*>(&frame), sizeof(CAN_Send_Frame));
    // if (bytes == (int)sizeof(CAN_Send_Frame)) {
    //     tx_counter_.tick();  // 仅在发送成功后更新计数器
    // }
    return bytes == (int)sizeof(CAN_Send_Frame);
}

// -------- RX 线程函数：阻塞接收并解析更新回馈 ----------
void USB2CANDevice::rx_thread_func()
{
    apply_can_thread_rt(can_device_cpu(device_idx_));
    CAN_Receive_Frame recv_frame;
    while (running_.load()) {
        int r = recv_fn_(reinterpret_cast<uint8_t*>(&recv_frame), 0xAA, sizeof(CAN_Receive_Frame));
        if (r <= 0) {
            continue;
        }

        // 检查接收到的帧长度是否完整
        if (r != sizeof(CAN_Receive_Frame)) {
            std::cerr << "[" << dev_node_ << "] rx incomplete frame: "
                      << r << "/" << sizeof(CAN_Receive_Frame) << " bytes" << std::endl;
            continue;
        }

        // 基本校验：帧头/帧尾/len 范围
       if (recv_frame.CMD != 0x11 || recv_frame.frameEnd != 0x55) {
        // std::cerr << "[" << dev_node_ << "] rx invalid CMD or frameEnd\n";
        continue;
        }

        // rx_counter_.tick();
        process_recv_frame(recv_frame);
    }

    std::cout << "[" << dev_node_ << "] rx_thread exit\n";
}

void USB2CANDevice::process_recv_frame(const CAN_Receive_Frame& frame)
{
    MotorFeedback fb;
    fb.can_id = frame.canData[0];
    uint32_t motor_id = fb.can_id;

    {
        std::lock_guard<std::mutex> lk(fb_mtx_);
        if (std::find(managed_ids_.begin(), managed_ids_.end(), motor_id) == managed_ids_.end()) {
            return; // 忽略未管理的ID
        }
        fb.motor_type = motor_types_[motor_id];
    }

    // 根据电机型号获取参数范围
    MotorLimits limits = MotorLimits::getLimits(fb.motor_type);

    uint8_t data[8];
    memcpy(data, frame.canData, 8);

    uint16_t q_uint = (uint16_t(data[1]) << 8) | data[2];  // 位置：data[1]高8位 + data[2]低8位
    uint16_t dq_uint = (uint16_t(data[3]) << 4) | (data[4] >> 4);  // 速度：data[3]高4位 + data[4]高4位
    uint16_t tau_uint = (uint16_t(data[4] & 0x0F) << 8) | data[5];  // 扭矩：data[4]低4位 + data[5]低8位

    // 灵足(RobStride) MIT 反馈帧 data[6..7] = 绕组温度，uint16 大端，单位 0.1°C。
    // 实测 RS04(小腿)固件会把最高位 bit15 置 1(状态/标志位，RS03 偶发也会出现一帧)，
    // 真实温度在低 15 位：0x8140&0x7FFF=320->32.0°C，与其余电机室温一致；
    // 直接按 uint16 解析会得到 3308.8°C 的荒谬值。故统一屏蔽 bit15。
    fb.raw_temp = static_cast<uint16_t>((uint16_t(data[6]) << 8 | data[7]) & 0x7FFF);
    fb.temperature = static_cast<float>(fb.raw_temp) * 0.1f;

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
                   std::function<int(const uint8_t*, size_t)> send_fn,
                   std::function<int(uint8_t*, uint8_t, size_t)> recv_fn)
{
    int idx = (int)devices_.size();
    devices_.emplace_back(new USB2CANDevice(dev_node, send_fn, recv_fn, idx));
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
