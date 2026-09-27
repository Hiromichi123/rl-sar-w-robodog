#include "motor_manager.h"
#include <iostream>
#include <unistd.h>
#include <iomanip>
#include <chrono>

RobotMotorManager::RobotMotorManager()
    : fb_mutex_(4),
      leg_feedbacks_(4) {
    for (int i = 0; i < 4; ++i) {
        leg_commands_[i].store(new LegCommand(), std::memory_order_relaxed);
    }

    // 电机映射表 - 每条腿4个电机，CAN ID 每条腿统一用4-1（通过can_idx区分腿）
    motor_map_ = {
        // FL腿 (CAN0) - leg0
        {0, 4, 0, -1.0f, MotorType::LINGZU_03},  // hip(灵足03电机)
        {0, 3, 1, -1.0f, MotorType::LINGZU_03},  // thigh(灵足03电机)
        {0, 2, 2, -1.0f, MotorType::LINGZU_04},  // calf (灵足04电机)
        {0, 1, 3, -1.0f, MotorType::LINGZU_02},  // foot (灵足02电机)
        // FR腿 (CAN1) - leg1
        {1, 4, 0,  -1.0f, MotorType::LINGZU_03},  // hip(灵足03电机)
        {1, 3, 1,  1.0f, MotorType::LINGZU_03},  // thigh(灵足03电机)
        {1, 2, 2,  1.0f, MotorType::LINGZU_04},  // calf (灵足04电机)
        {1, 1, 3,  1.0f, MotorType::LINGZU_02},  // foot (灵足02电机)
        // RL腿 (CAN2) - leg2
        {2, 4, 0, 1.0f, MotorType::LINGZU_03},  // hip(灵足03电机)
        {2, 3, 1, -1.0f, MotorType::LINGZU_03},  // thigh(灵足03电机)
        {2, 2, 2, -1.0f, MotorType::LINGZU_04},  // calf (灵足04电机)
        {2, 1, 3, -1.0f, MotorType::LINGZU_02},  // foot (灵足02电机)
        // RR腿 (CAN3) - leg3
        {3, 4, 0,  1.0f, MotorType::LINGZU_03},  // hip(灵足03电机)
        {3, 3, 1,  1.0f, MotorType::LINGZU_03},  // thigh(灵足03电机)
        {3, 2, 2,  1.0f, MotorType::LINGZU_04},  // calf (灵足04电机)
        {3, 1, 3,  1.0f, MotorType::LINGZU_02},  // foot (灵足02电机)
    };
}

RobotMotorManager::~RobotMotorManager() {
    stopThreads();
    for (int i = 0; i < 4; ++i) {
        delete leg_commands_[i].load();
    }
}


void RobotMotorManager::init() {
    std::vector<std::string> can_dev_paths = {
        "/dev/ttycan1","/dev/ttycan2","/dev/ttycan3","/dev/ttycan4"
    };

    serials_.reserve(can_dev_paths.size());

    for (const auto& path : can_dev_paths) {
        auto serial = std::make_shared<SerialPort>(path, B921600);
        if (!serial->isOpen()) {
            std::cerr << "[MotorManager] 串口 " << path << " 打开失败！" << std::endl;
            continue;
        }
        serials_.push_back(serial);
    }

    for (int i = 0; i < serials_.size(); ++i) {
        auto serial = serials_[i];

        int dev_idx = can_manager_.add_device(
            can_dev_paths[i],
            [serial](const uint8_t* data, size_t len) {
                return serial->send(data, len);
            },
            [serial](uint8_t* data, uint8_t header, size_t len) {
                ssize_t actual_len = serial->recv(data, header, len);
                return (actual_len > 0) ? static_cast<int>(actual_len) : 0;
            }
        );

        can_device_indices_.push_back(dev_idx);
        std::vector<uint32_t> ids;
        for (const auto& map : motor_map_) {
            if (map.can_idx == i) ids.push_back(map.can_id);
        }
        can_manager_.set_device_motor_ids(dev_idx, ids);

        // 设置每个电机的型号
        for (const auto& map : motor_map_) {
            if (map.can_idx == i) {
                can_manager_.set_motor_type(dev_idx, map.can_id, map.motor_type);
            }
        }

        std::cout << "[MotorManager] 1成功初始化 " << can_dev_paths[i]
                  << " (dev_idx=" << dev_idx << "), CAN IDs: ";
        for (auto id : ids) std::cout << id << " ";
        std::cout << "- 串口状态: " << (serial->isOpen() ? "打开" : "关闭") << std::endl;
    }

    std::cout << "[MotorManager] 底层CAN设备初始化完成" << std::endl;
}


// 启动4个腿线程
void RobotMotorManager::startThreads() {
    if (running_.load()) return;
    running_.store(true);
    can_manager_.start_all(); 
    for (int i = 0; i < 4; ++i) {
        leg_threads_.emplace_back(&RobotMotorManager::legThreadFunc, this, i);
    }
    //std::cout << "[MotorManager] 4个腿部控制线程和CAN设备线程启动" << std::endl;
}

// 停止线程
void RobotMotorManager::stopThreads() {
      if (!running_.load()) return;
      running_.store(false);

      for (auto& serial : serials_) {
          if (serial && serial->isOpen()) {
              serial->stop();
          }
      }
      can_manager_.stop_all();

      // 等待所有腿部控制线程退出
      for (auto& t : leg_threads_) {
          if (t.joinable()) {
              t.join();
          }
      }
      leg_threads_.clear();
      //std::cout << "[MotorManager] 4个腿部控制线程和CAN设备线程停止" << std::endl;
  }


void RobotMotorManager::sendAllMotorsCommand(
    const std::vector<float>& tau,
    const std::vector<float>& vel,
    const std::vector<float>& pos,
    const std::vector<float>& kp,
    const std::vector<float>& kd)
{
    if (tau.size() != 16) {
        std::cerr << "指令必须是 16 个" << std::endl;
        return;
    }

    for (int m = 0; m < 16; ++m) {
        int leg = m / 4;
        int j = m % 4;

        auto* cmd = leg_commands_[leg].load(std::memory_order_relaxed);

        cmd->tau[j] = tau[m];
        cmd->vel[j] = vel[m];
        cmd->pos[j] = pos[m];
        cmd->kp[j]  = kp[m];
        cmd->kd[j]  = kd[m];
    }
}

void RobotMotorManager::sendMotorCommand(int motor_global_idx,
                                         float tau, float vel, float pos,
                                         float kp, float kd) {

    if (motor_global_idx < 0 || motor_global_idx >= 16) {
        std::cerr << "[MotorManager] 无效电机索引: " << motor_global_idx << std::endl;
        return;
    }

    int leg_idx = motor_global_idx / 4;  // 腿索引：0~3
    int motor_in_leg = motor_global_idx % 4;  // 腿内电机索引：0~3

    LegCommand* cmd = leg_commands_[leg_idx].load(std::memory_order_relaxed);
    cmd->tau[motor_in_leg] = tau;
    cmd->vel[motor_in_leg] = vel;
    cmd->pos[motor_in_leg] = pos;
    cmd->kp[motor_in_leg] = kp;
    cmd->kd[motor_in_leg] = kd;
}
void RobotMotorManager::sendLegCommand(int leg_index,
                                      const Eigen::RowVector4f& tau,
                                      const Eigen::RowVector4f& vel,
                                      const Eigen::RowVector4f& pos,
                                      const Eigen::RowVector4f& kp,
                                      const Eigen::RowVector4f& kd) {
    if (leg_index < 0 || leg_index >= 4) {
        std::cerr << "[MotorManager] 无效腿索引: " << leg_index << std::endl;
        return;
    }
    
    LegCommand* cmd = leg_commands_[leg_index].load(std::memory_order_relaxed);
    cmd->tau = tau;
    cmd->vel = vel;
    cmd->pos = pos;
    cmd->kp = kp;
    cmd->kd = kd;
}

Eigen::RowVector4f RobotMotorManager::getMotorPos_leg(int leg_index) {
    if (leg_index < 0 || leg_index >= 4) return Eigen::RowVector4f::Zero();
    std::lock_guard<std::mutex> lock(fb_mutex_[leg_index]);
    return leg_feedbacks_[leg_index].pos;
}

Eigen::RowVector4f RobotMotorManager::getMotorVel_leg(int leg_index) {
    if (leg_index < 0 || leg_index >= 4) return Eigen::RowVector4f::Zero();
    std::lock_guard<std::mutex> lock(fb_mutex_[leg_index]);
    return leg_feedbacks_[leg_index].vel;
}

Eigen::RowVector4f RobotMotorManager::getMotorTorque_leg(int leg_index) {
    if (leg_index < 0 || leg_index >= 4) return Eigen::RowVector4f::Zero();
    std::lock_guard<std::mutex> lock(fb_mutex_[leg_index]);
    return leg_feedbacks_[leg_index].tau;
}
Eigen::VectorXf RobotMotorManager::getAllMotorPos() {
    Eigen::VectorXf all_pos(16);
    int index = 0;

    for (int leg = 0; leg < 4; ++leg) {
        if (leg < 0 || leg >= 4) {
            continue;
        }
        std::lock_guard<std::mutex> lock(fb_mutex_[leg]);
        for (int motor = 0; motor < 4; ++motor) {
            all_pos[index++] = leg_feedbacks_[leg].pos[motor];
        }
    }
    return all_pos;
}
Eigen::VectorXf RobotMotorManager::getAllMotorVel() {
    Eigen::VectorXf all_vel(16);
    int index = 0;

    for (int leg = 0; leg < 4; ++leg) {
        std::lock_guard<std::mutex> lock(fb_mutex_[leg]);
        for (int motor = 0; motor < 4; ++motor) {
            all_vel[index++] = leg_feedbacks_[leg].vel[motor];
        }
    }
    return all_vel;
}
Eigen::VectorXf RobotMotorManager::getAllMotorTorque() {
    Eigen::VectorXf all_tau(16);
    int index = 0;

    for (int leg = 0; leg < 4; ++leg) {
        std::lock_guard<std::mutex> lock(fb_mutex_[leg]);
        for (int motor = 0; motor < 4; ++motor) {
            all_tau[index++] = leg_feedbacks_[leg].tau[motor];
        }
    }
    return all_tau;
}

std::vector<float> RobotMotorManager::getAllMotorTemp() {
    std::vector<float> all_temp;
    all_temp.reserve(16);

    for (int leg = 0; leg < 4; ++leg) {
        std::lock_guard<std::mutex> lock(fb_mutex_[leg]);
        for (int motor = 0; motor < 4; ++motor) {
            all_temp.push_back(leg_feedbacks_[leg].temp[motor]);
        }
    }
    return all_temp;
}

// 一次性读取所有电机反馈
RobotMotorManager::AllMotorFeedback RobotMotorManager::getAllMotorFeedback() {
    AllMotorFeedback all;
    all.pos.resize(16);
    all.vel.resize(16);
    all.tau.resize(16);
    all.temp.reserve(16);

    int index = 0;
    for (int leg = 0; leg < 4; ++leg) {
        std::lock_guard<std::mutex> lock(fb_mutex_[leg]);
        for (int motor = 0; motor < 4; ++motor) {
            all.pos[index] = leg_feedbacks_[leg].pos[motor];
            all.vel[index] = leg_feedbacks_[leg].vel[motor];
            all.tau[index] = leg_feedbacks_[leg].tau[motor];
            all.temp.push_back(leg_feedbacks_[leg].temp[motor]);
            index++;
        }
    }

    return all;
}


void RobotMotorManager::legThreadFunc(int leg_index) {
    int can_idx = leg_index;
    int dev_idx = can_device_indices_[can_idx];
    std::vector<MotorMap> leg_motors;
    for (const auto& map : motor_map_) {
        if (map.can_idx == can_idx) leg_motors.push_back(map);
    }

    while (running_.load()) {
        // 读取当前腿的指令（无锁，直接读取原子指针指向的内容）
        LegCommand* cmd = leg_commands_[leg_index].load(std::memory_order_relaxed);


        for (const auto& motor : leg_motors) {
            int idx = motor.leg_motor_idx;
            MotorCommand m_cmd;
            m_cmd.can_id = motor.can_id;
            m_cmd.kp = cmd->kp[idx];
            m_cmd.kd = cmd->kd[idx];
            m_cmd.position = motor.dir * cmd->pos[idx];
            m_cmd.speed = motor.dir * cmd->vel[idx];
            m_cmd.torque = motor.dir * cmd->tau[idx];
            m_cmd.motor_type = motor.motor_type;  // 设置电机型号
            can_manager_.send_command_to_device(dev_idx, m_cmd);
        }

        LegFeedback fb;
        for (const auto& motor : leg_motors) {
            int idx = motor.leg_motor_idx;
            MotorFeedback m_fb;
            if (can_manager_.get_feedback(dev_idx, motor.can_id, m_fb)) {
                fb.pos[idx] = motor.dir * m_fb.pos_f;
                fb.vel[idx] = motor.dir * m_fb.speed_f;
                fb.tau[idx] = motor.dir * m_fb.torque_f;
                fb.temp[idx] = m_fb.temperature;  // 温度不需要方向系数
            }
        }
        {
            std::lock_guard<std::mutex> lock(fb_mutex_[leg_index]);
            leg_feedbacks_[leg_index] = fb;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void RobotMotorManager::sendControlCommand(int can_idx, uint32_t can_id, uint8_t cmd) {
    CAN_Send_Frame frame;
    frame.CMD = 0x03;  // 确认该CMD符合电机协议
    frame.len = 8;
    std::array<uint8_t, 8> data = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, cmd};
    frame.modify(can_id, data.data());

    // 直接通过 can_idx 定位对应串口发送
    if (can_idx >= 0 && can_idx < (int)serials_.size() && serials_[can_idx]->isOpen()) {
        serials_[can_idx]->send(reinterpret_cast<uint8_t*>(&frame), sizeof(frame));
    } else {
        std::cerr << "[sendControlCommand] 设备 " << can_idx << " 串口未打开!" << std::endl;
    }
}

void RobotMotorManager::enableMotor(int motor_id) {
    if (motor_id < 0 || motor_id >= 16) return;
    const auto& m = motor_map_[motor_id];
    sendControlCommand(m.can_idx, m.can_id, 0xFC);
    usleep(1000);
}

void RobotMotorManager::disableMotor(int motor_id) {
    if (motor_id < 0 || motor_id >= 16) return;
    const auto& m = motor_map_[motor_id];
    sendControlCommand(m.can_idx, m.can_id, 0xFD);
    usleep(1000);
}

void RobotMotorManager::setZero(int motor_id) {
    if (motor_id < 0 || motor_id >= 16) {
        std::cerr << "[setZero] 无效电机ID: " << motor_id << std::endl;
        return;
    }

    const auto& m = motor_map_[motor_id];
    sendControlCommand(m.can_idx, m.can_id, 0xFE);  // 0xFE=设零
    usleep(2000);
}

void RobotMotorManager::enableAllMotors() {
    std::lock_guard<std::mutex> lock(api_mutex_);
    for (int idx : connected_motors_) {
    //    std::cout << "[enableAllMotors] 准备使能索引=" << idx << std::endl;
        enableMotor(idx);  // 调用已验证有效的单个使能函数
        usleep(100000);  // 100ms间隔，确保电机响应
    }
}

void RobotMotorManager::disableAllMotors() {
    std::lock_guard<std::mutex> lock(api_mutex_);
    for (int idx : connected_motors_) {
        disableMotor(idx);
        usleep(100000);  // 100ms间隔，确保电机响应
    }
}

void RobotMotorManager::setZeroAllMotors() {
    std::lock_guard<std::mutex> lock(api_mutex_);
    for (int motor_id : connected_motors_) {
        setZero(motor_id);
        usleep(100000);  // 适当延长等待时间
    }
}

void RobotMotorManager::initAllMotors() {

std::cerr << "initAllMotors";
    //setZeroAllMotors();
    enableAllMotors();
    usleep(100000);
    //setZeroAllMotors();
}
