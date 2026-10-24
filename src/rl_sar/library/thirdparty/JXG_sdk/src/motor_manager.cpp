#include "motor_manager.h"
#include <cstdlib>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <unistd.h>
#include <iomanip>
#include <chrono>

RobotMotorManager::RobotMotorManager()
    : fb_mutex_(4),
      leg_feedbacks_(4) {
    for (int i = 0; i < 4; ++i) {
        active_cmd_idx_[i].store(0, std::memory_order_relaxed);
    }
    // 探针结果缓存初始化为无效(未检测)
    for (int i = 0; i < 16; ++i) {
        probe_results_valid_[i].store(false, std::memory_order_relaxed);
        probe_results_[i].store(false, std::memory_order_relaxed);
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
}


void RobotMotorManager::init(const std::string& can_transport) {
    can_transport_ = can_transport;
    // v7.2: 支持双传输层共存,通过参数选择
    //   usb2can  : /dev/ttycan1~4 串口(HDSC 0x55AA 帧协议),SerialPort + send/recv 回调
    //   socketcan: can0~can3 内核 PF_CAN 原始套接字,SocketCANPort + sendCan/recvCan
    if (can_transport == "socketcan") {
        // v7.7: C++ 版 CAN 接口配置（替代 scripts/can_setup.sh）
        // 在创建 SocketCANPort 前，先自动完成序列号识别、改名、bitrate 配置和 UP
        int can_ready = CanSetup::Run(1000000);
        if (can_ready < 4) {
            std::cerr << "[MotorManager] 警告: CAN 接口仅 " << can_ready
                      << "/4 就绪，继续尝试启动（缺失接口的电机可能不可用）" << std::endl;
        }

        std::vector<std::string> can_ifaces = {"can0","can1","can2","can3"};
        can_ports_.reserve(can_ifaces.size());

        std::vector<std::string> failed;
        for (const auto& iface : can_ifaces) {
            auto port = std::make_shared<SocketCANPort>(iface);
            if (!port->isOpen()) {
                std::cerr << "[MotorManager] SocketCAN 接口 " << iface << " 打开失败！" << std::endl;
                failed.push_back(iface);
                continue;
            }
            can_ports_.push_back(port);
        }

        if (can_ports_.size() != can_ifaces.size()) {
            std::cerr << "[MotorManager] 致命错误：SocketCAN 接口就绪 " << can_ports_.size()
                      << "/" << can_ifaces.size() << " 路，缺失：";
            for (const auto& f : failed) std::cerr << f << " ";
            std::cerr << std::endl;
            std::cerr << "[MotorManager] 请检查 can_setup.sh 执行结果与 udev 规则"
                         "（ip -details link show can0~can3），修复后重新启动。" << std::endl;
            std::fflush(nullptr);
            _Exit(EXIT_FAILURE);
        }

        for (size_t i = 0; i < can_ports_.size(); ++i) {
            auto port = can_ports_[i];
            // 裸 CAN 帧回调:直接绑定 SocketCANPort 收发(sendCan/recvCan 均为整帧原子读写)
            int dev_idx = can_manager_.add_device(
                can_ifaces[i],
                [port](uint32_t can_id, const uint8_t* data) {
                    return port->sendCan(can_id, data) > 0 ? 1 : -1;
                },
                [port](uint32_t& can_id, uint8_t* data) {
                    return port->recvCan(can_id, data) > 0 ? 1 : 0;
                }
            );
            can_device_indices_.push_back(dev_idx);
            std::vector<uint32_t> ids;
            for (const auto& map : motor_map_) {
                if (map.can_idx == (int)i) ids.push_back(map.can_id);
            }
            can_manager_.set_device_motor_ids(dev_idx, ids);
            for (const auto& map : motor_map_) {
                if (map.can_idx == (int)i) can_manager_.set_motor_type(dev_idx, map.can_id, map.motor_type);
            }
            std::cout << "[MotorManager] SocketCAN " << can_ifaces[i]
                      << " (dev_idx=" << dev_idx << ") 初始化成功" << std::endl;
        }
        std::cout << "[MotorManager] 底层 SocketCAN 设备初始化完成" << std::endl;
        return;
    }

    // ---- 默认: usb2can 串口模式 ----
    std::vector<std::string> can_dev_paths = {
        "/dev/ttycan1","/dev/ttycan2","/dev/ttycan3","/dev/ttycan4"
    };

    serials_.reserve(can_dev_paths.size());

    std::vector<std::string> failed_paths;
    for (const auto& path : can_dev_paths) {
        auto serial = std::make_shared<SerialPort>(path, B921600);
        if (!serial->isOpen()) {
            std::cerr << "[MotorManager] 串口 " << path << " 打开失败！" << std::endl;
            failed_paths.push_back(path);
            continue;
        }
        serials_.push_back(serial);
    }

    // 防护：4 路 CAN 必须全部就绪，否则后续按 4 路索引访问会越界崩溃(SEGV)。
    // 常见于 USB-CAN 枚举不全(如 /dev/ttyACM4 节点未生成)，此时直接退出，
    // 由上层(service.sh/人工)修复设备后再启动，避免带病运行。
    if (serials_.size() != can_dev_paths.size()) {
        std::cerr << "[MotorManager] 致命错误：CAN 设备就绪 " << serials_.size()
                  << "/" << can_dev_paths.size() << " 路，缺失：";
        for (const auto& p : failed_paths) std::cerr << p << " ";
        std::cerr << std::endl;
        std::cerr << "[MotorManager] 请检查 USB-CAN 连接与 /dev/ttyACM* 设备节点"
                     "（ls -la /dev/ttycan* /dev/ttyACM*），修复后重新启动。" << std::endl;
        // _Exit 而非 exit:避开 ROS/DDS 静态析构竞态,保证干净退出、日志完整
        std::fflush(nullptr);
        _Exit(EXIT_FAILURE);
    }

    for (int i = 0; i < serials_.size(); ++i) {
        auto serial = serials_[i];

        // 裸 CAN 帧回调 <-> HDSC 0x55AA 串口帧 适配:
        //   send: 组 CAN_Send_Frame 整体写入串口
        //   recv: 按 0xAA 同步读一帧 CAN_Receive_Frame,校验 CMD/帧尾后解出 canData
        int dev_idx = can_manager_.add_device(
            can_dev_paths[i],
            [serial](uint32_t can_id, const uint8_t* data) {
                CAN_Send_Frame frame;
                frame.modify(can_id, data);
                ssize_t n = serial->send(reinterpret_cast<const uint8_t*>(&frame), sizeof(frame));
                return n == (ssize_t)sizeof(frame) ? 1 : -1;
            },
            [serial](uint32_t& can_id, uint8_t* data) {
                CAN_Receive_Frame frame;
                ssize_t n = serial->recv(reinterpret_cast<uint8_t*>(&frame), 0xAA, sizeof(frame));
                if (n != (ssize_t)sizeof(frame)) return 0;
                if (frame.CMD != 0x11 || frame.frameEnd != 0x55) return 0;
                can_id = frame.canId;
                std::memcpy(data, frame.canData, 8);
                return 1;
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
      for (auto& port : can_ports_) {
          if (port) port->stop();
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

    // C1-fix: 按腿分组写入,整条腿 4 个电机全部写入 back buffer 后再切换 active,
    // 避免每写一个电机就切换导致同腿电机拆分到两个 buffer(active buffer 中
    // 部分电机读到初始零值,kp/kd 丢失)。
    for (int leg = 0; leg < 4; ++leg) {
        int back = 1 - active_cmd_idx_[leg].load(std::memory_order_relaxed);
        LegCommand& cmd = leg_cmd_buffers_[leg][back];
        for (int j = 0; j < 4; ++j) {
            int m = leg * 4 + j;
            cmd.tau[j] = tau[m];
            cmd.vel[j] = vel[m];
            cmd.pos[j] = pos[m];
            cmd.kp[j]  = kp[m];
            cmd.kd[j]  = kd[m];
        }
        active_cmd_idx_[leg].store(back, std::memory_order_release);
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

    // C1-fix: 先把 active buffer 完整复制到 back buffer,再修改目标电机,
    // 避免切换后其他 3 个电机读到零值。
    int front = active_cmd_idx_[leg_idx].load(std::memory_order_relaxed);
    int back = 1 - front;
    leg_cmd_buffers_[leg_idx][back] = leg_cmd_buffers_[leg_idx][front];

    LegCommand& cmd = leg_cmd_buffers_[leg_idx][back];
    cmd.tau[motor_in_leg] = tau;
    cmd.vel[motor_in_leg] = vel;
    cmd.pos[motor_in_leg] = pos;
    cmd.kp[motor_in_leg] = kp;
    cmd.kd[motor_in_leg] = kd;
    active_cmd_idx_[leg_idx].store(back, std::memory_order_release);
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
    
    int back = 1 - active_cmd_idx_[leg_index].load(std::memory_order_relaxed);
    LegCommand& cmd = leg_cmd_buffers_[leg_index][back];
    cmd.tau = tau;
    cmd.vel = vel;
    cmd.pos = pos;
    cmd.kp = kp;
    cmd.kd = kd;
    active_cmd_idx_[leg_index].store(back, std::memory_order_release);
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

Eigen::VectorXf RobotMotorManager::getAllMotorTemp() {
    Eigen::VectorXf all_temp(16);
    int index = 0;

    for (int leg = 0; leg < 4; ++leg) {
        std::lock_guard<std::mutex> lock(fb_mutex_[leg]);
        for (int motor = 0; motor < 4; ++motor) {
            all_temp[index++] = leg_feedbacks_[leg].temp[motor];
        }
    }
    return all_temp;
}

std::vector<float> RobotMotorManager::getAllMotorError() {
    std::vector<float> all_error;
    all_error.reserve(16);

    for (int leg = 0; leg < 4; ++leg) {
        std::lock_guard<std::mutex> lock(fb_mutex_[leg]);
        for (int motor = 0; motor < 4; ++motor) {
            all_error.push_back(leg_feedbacks_[leg].error[motor]);
        }
    }
    return all_error;
}

// 一次性读取所有电机反馈
RobotMotorManager::AllMotorFeedback RobotMotorManager::getAllMotorFeedback() {
    AllMotorFeedback all;
    getAllMotorFeedback(all);
    return all;
}

void RobotMotorManager::getAllMotorFeedback(AllMotorFeedback& out) {
    // 调用方负责预分配 out.pos/vel/tau/temp(16) 和 out.error(16);此处仅做数据拷贝,零堆分配。
    // 防御:尺寸已匹配时 Eigen resize / assign 为空操作(稳态零分配);
    // 若新增调用点漏配容量,此处一次性补齐,避免空向量 operator[] 越界 UB。
    if (out.pos.size() != 16)   out.pos.resize(16);
    if (out.vel.size() != 16)   out.vel.resize(16);
    if (out.tau.size() != 16)   out.tau.resize(16);
    if (out.temp.size() != 16)  out.temp.resize(16);
    if (out.error.size() != 16) out.error.assign(16, 0.0f);
    int index = 0;
    for (int leg = 0; leg < 4; ++leg) {
        std::lock_guard<std::mutex> lock(fb_mutex_[leg]);
        for (int motor = 0; motor < 4; ++motor) {
            out.pos[index] = leg_feedbacks_[leg].pos[motor];
            out.vel[index] = leg_feedbacks_[leg].vel[motor];
            out.tau[index] = leg_feedbacks_[leg].tau[motor];
            out.error[index] = leg_feedbacks_[leg].error[motor];
            out.temp[index] = leg_feedbacks_[leg].temp[motor];
            index++;
        }
    }
}

// ================= L1/L2 链路新鲜度只读接口 =================
bool RobotMotorManager::isMotorFeedbackFresh(int global_idx, int timeout_ms) {
    if (global_idx < 0 || global_idx >= (int)motor_map_.size()) return false;
    const auto& m = motor_map_[global_idx];
    if (m.can_idx < 0 || m.can_idx >= (int)can_device_indices_.size()) return false;
    int dev_idx = can_device_indices_[m.can_idx];
    int64_t age = can_manager_.feedback_age_ms(dev_idx, m.can_id);
    return age >= 0 && age <= timeout_ms;
}

int RobotMotorManager::canLinkCount() {
    return static_cast<int>(can_device_indices_.size());
}

int64_t RobotMotorManager::canLinkLastRxAgeMs(int can_idx) {
    if (can_idx < 0 || can_idx >= (int)can_device_indices_.size()) return -1;
    int dev_idx = can_device_indices_[can_idx];
    int64_t last = can_manager_.device_last_rx_ms(dev_idx);
    if (last <= 0) return -1;  // 从未收到合法帧
    int64_t now = std::chrono::duration_cast<std::chrono::milliseconds>(
                      std::chrono::steady_clock::now().time_since_epoch())
                      .count();
    return now - last;
}

int64_t RobotMotorManager::motorFeedbackAgeMs(int global_idx) {
    if (global_idx < 0 || global_idx >= (int)motor_map_.size()) return -1;
    const auto& m = motor_map_[global_idx];
    if (m.can_idx < 0 || m.can_idx >= (int)can_device_indices_.size()) return -1;
    int dev_idx = can_device_indices_[m.can_idx];
    return can_manager_.feedback_age_ms(dev_idx, m.can_id);  // -1 = 从未收到
}

int RobotMotorManager::motorCanIdx(int global_idx) {
    if (global_idx < 0 || global_idx >= (int)motor_map_.size()) return -1;
    return motor_map_[global_idx].can_idx;
}


void RobotMotorManager::legThreadFunc(int leg_index) {
    // 电机指令/反馈下发节拍。灵足 MIT 电机每收到一帧控制帧回一帧状态帧,
    // 故该频率同时决定 tx(write 系统调用)与 rx 负载。
    // 上层控制环仅 200Hz,1000Hz 会把同一条指令重复发 5 遍、每秒产生约 1.6 万次
    // 串口 write(占满 CPU5/7 的 CAN 线程且内核态为主)。500Hz 相对 200Hz 控制器
    // 仍有 2.5 倍余量,电机在帧间自持力矩;实测可把 CAN 线程总占用降低约一半。
    constexpr auto kLegPeriod = std::chrono::milliseconds(2);  // 500Hz(原 1ms=1000Hz)

    int can_idx = leg_index;
    int dev_idx = can_device_indices_[can_idx];
    // (全局电机序号, 映射) 对——探针按全局序号索引 enable_probes_
    std::vector<std::pair<int, MotorMap>> leg_motors;
    for (int g = 0; g < (int)motor_map_.size(); ++g) {
        if (motor_map_[g].can_idx == can_idx) leg_motors.emplace_back(g, motor_map_[g]);
    }

    // === 死链路发送抑制(防 CAN 错误风暴) ===
    // 电机断电/未接入时总线上无 ACK,500Hz 持续发帧会使控制器错误计数
    // 飙到 255 反复 bus-off,并把 candleLight(gs_usb) 适配器固件打到假死
    // (实机 2026-10-11 事故:四路同时挂,重启服务/电机均无效,仅重启主板
    // 重新枚举 USB 才恢复)。故链路从未收到合法帧(L1=-1)或超过
    // kTxSuppressMs 无接收时,发送降为每 kKeepaliveMs 一轮保活;
    // 灵足 MIT 电机每收一帧控制帧回一帧状态帧,保活帧即可探活,
    // 电机上电回帧后 L1 变小,自动恢复全速发送,无需人工干预。
    constexpr int64_t kTxSuppressMs = 3000;
    constexpr int64_t kKeepaliveMs  = 1000;
    int64_t last_keepalive_ms = -1;  // 首轮必发(兼作冷启动探活)

    while (running_.load()) {
        // C1-fix: 双缓冲读取,与写线程(sendAllMotorsCommand 等)无锁同步
        //   front buffer 在写线程 store(release) 后可见,保证读取到完整一致的一帧
        int front = active_cmd_idx_[leg_index].load(std::memory_order_acquire);
        const LegCommand& cmd = leg_cmd_buffers_[leg_index][front];

        // L1 链路新鲜度: -1=从未收到合法帧; 正常运行时约 2ms
        const int64_t rx_age = canLinkLastRxAgeMs(can_idx);
        const bool link_dead = (rx_age < 0) || (rx_age >= kTxSuppressMs);
        bool send_frames = true;
        if (link_dead) {
            const int64_t now_ms =
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now().time_since_epoch())
                    .count();
            if (now_ms - last_keepalive_ms >= kKeepaliveMs) {
                last_keepalive_ms = now_ms;  // 本轮按原速发出,兼作保活/探活帧
            } else {
                send_frames = false;         // 抑制本轮发送,防止错误风暴
            }
        }

        if (send_frames) {
            for (const auto& [gidx, motor] : leg_motors) {
                int idx = motor.leg_motor_idx;
                MotorCommand m_cmd;
                m_cmd.can_id = motor.can_id;
                const EnableProbe& probe = enable_probes_[gidx];
                if (probe.active.load(std::memory_order_acquire)) {
                    // 探针覆写:探针值替代上层指令槽(仅本电机,其余腿关节不受影响)
                    m_cmd.kp = probe.kp;
                    m_cmd.kd = probe.kd;
                    m_cmd.position = motor.dir * probe.pos;
                    m_cmd.speed = motor.dir * probe.vel;
                    m_cmd.torque = 0.0f;
                } else {
                    m_cmd.kp = cmd.kp[idx];
                    m_cmd.kd = cmd.kd[idx];
                    m_cmd.position = motor.dir * cmd.pos[idx];
                    m_cmd.speed = motor.dir * cmd.vel[idx];
                    m_cmd.torque = motor.dir * cmd.tau[idx];
                }
                m_cmd.motor_type = motor.motor_type;  // 设置电机型号
                can_manager_.send_command_to_device(dev_idx, m_cmd);
            }
        }

        LegFeedback fb;
        for (const auto& [gidx, motor] : leg_motors) {
            int idx = motor.leg_motor_idx;
            MotorFeedback m_fb;
            if (can_manager_.get_feedback(dev_idx, motor.can_id, m_fb)) {
                fb.pos[idx] = motor.dir * m_fb.pos_f;
                fb.vel[idx] = motor.dir * m_fb.speed_f;
                fb.tau[idx] = motor.dir * m_fb.torque_f;
                fb.error[idx] = m_fb.error_code;  // 协议无错误字段,恒 0
                fb.temp[idx] = m_fb.temperature;  // 温度无方向系数,-1=无反馈
            }
        }
        {
            std::lock_guard<std::mutex> lock(fb_mutex_[leg_index]);
            leg_feedbacks_[leg_index] = fb;
        }

        std::this_thread::sleep_for(kLegPeriod);
    }
}

void RobotMotorManager::sendControlCommand(int can_idx, uint32_t can_id, uint8_t cmd) {
    std::array<uint8_t, 8> data = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, cmd};

    if (can_transport_ == "socketcan") {
        // SocketCAN: 直接发裸 CAN 帧(使能/失能/设零均为 8 字节 0xFF..cmd)
        if (can_idx >= 0 && can_idx < (int)can_ports_.size() && can_ports_[can_idx]->isOpen()) {
            can_ports_[can_idx]->sendCan(can_id, data.data());
        } else {
            std::cerr << "[sendControlCommand] 设备 can" << can_idx << " 接口未打开!" << std::endl;
        }
        return;
    }

    // usb2can: 组 HDSC 串口帧发送
    CAN_Send_Frame frame;
    frame.CMD = 0x03;  // 确认该CMD符合电机协议
    frame.len = 8;
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

// ── 可靠置零:0xFE 写零 + 反馈确认 + 自动重发 ──
// 旧实现 0xFE 发一帧即返回,与 500Hz 腿线程控制帧争用总线时偶发丢帧/固件未处理,
// 且无任何确认手段——个别电机没置上只能靠人工肉眼发现并整轮重来。
namespace {
constexpr float  kZeroVerifyPosRad  = 0.05f;   // 清零成功判据:|pos| <= 0.05rad
constexpr int    kZeroVerifyTimeoutMs = 80;    // 单轮确认窗口(反馈 ~400fps,含 30+ 帧)
constexpr int    kZeroMaxAttempts   = 3;       // 首发 + 2 次重发
constexpr int kZeroResendGapUs  = 10000;  // 重发前间隔 10ms
constexpr int kZeroInterMotorUs = 8000;   // 同总线台间保底 8ms
constexpr int kZeroPollUs       = 3000;   // 反馈轮询节拍 3ms
constexpr int64_t kZeroFreshMs = 100;          // 确认时反馈必须新鲜
}  // namespace

float RobotMotorManager::readMotorPosOnce(int global_idx) {
    if (global_idx < 0 || global_idx >= (int)motor_map_.size()) return 0.0f;
    const auto& m = motor_map_[global_idx];
    if (m.can_idx < 0 || m.can_idx >= 4) return 0.0f;
    std::lock_guard<std::mutex> lock(fb_mutex_[m.can_idx]);
    return leg_feedbacks_[m.can_idx].pos[m.leg_motor_idx];
}

bool RobotMotorManager::verifyMotorZeroed(int global_idx, int timeout_ms) {
    // 置零前本就在零位(重复置零)会立即通过,语义等价:目标态即零位。
    const auto deadline = std::chrono::steady_clock::now()
                          + std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < deadline) {
        const int64_t age = motorFeedbackAgeMs(global_idx);
        if (age >= 0 && age <= kZeroFreshMs) {
            if (std::fabs(readMotorPosOnce(global_idx)) <= kZeroVerifyPosRad)
                return true;
        }
        usleep(kZeroPollUs);
    }
    return false;
}

std::vector<int> RobotMotorManager::setZeroMotorsVerified(const std::vector<int>& ids) {
    // 与 enableAll/旧 setZero 同一把锁:与参数使能/其它批量配置串行,
    // 避免 0xFE 与使能帧在同一总线上交叉(腿线程 500Hz 控制帧不持此锁,不受阻塞)。
    std::lock_guard<std::mutex> lock(api_mutex_);

    // 去重 + 合法性过滤(保持首次出现顺序)
    std::vector<int> targets;
    bool seen[16] = {};
    for (int id : ids) {
        if (id < 0 || id >= 16 || seen[id]) continue;
        seen[id] = true;
        targets.push_back(id);
    }
    std::vector<int> failed;
    if (targets.empty()) return failed;

    // 四条 CAN 总线物理独立(socketcan 各持 socket / usb2can 各持串口),按 can_idx
    // 分 4 路并行;同总线内串行,台间 8ms 保底让出发送间隙。
    std::vector<std::vector<int>> by_bus(4);
    for (int id : targets) {
        const int bus = motor_map_[id].can_idx;
        if (bus >= 0 && bus < 4)
            by_bus[bus].push_back(id);
        else
            failed.push_back(id);  // 映射无效,无法发送
    }
    std::mutex failed_mutex;
    std::vector<std::thread> workers;
    workers.reserve(4);
    for (int bus = 0; bus < 4; ++bus) {
        if (by_bus[bus].empty()) continue;
        workers.emplace_back([this, bus, &by_bus, &failed, &failed_mutex]() {
            std::vector<int> local_failed;
            for (int id : by_bus[bus]) {
                const uint32_t can_id = motor_map_[id].can_id;
                bool ok = false;
                for (int attempt = 0; attempt < kZeroMaxAttempts; ++attempt) {
                    sendControlCommand(bus, can_id, 0xFE);  // 0xFE=设零
                    if (verifyMotorZeroed(id, kZeroVerifyTimeoutMs)) {
                        ok = true;
                        break;
                    }
                    usleep(kZeroResendGapUs);
                }
                if (!ok) {
                    local_failed.push_back(id);
                    std::cerr << "[setZeroMotors] motor " << id << " 置零后 "
                              << kZeroVerifyTimeoutMs << "ms 内未确认(|pos|>"
                              << kZeroVerifyPosRad << "rad 或反馈停滞),已重发 "
                              << (kZeroMaxAttempts - 1) << " 次" << std::endl;
                }
                usleep(kZeroInterMotorUs);
            }
            if (!local_failed.empty()) {
                std::lock_guard<std::mutex> lk(failed_mutex);
                failed.insert(failed.end(), local_failed.begin(), local_failed.end());
            }
        });
    }
    for (auto& t : workers) t.join();
    std::sort(failed.begin(), failed.end());
    return failed;
}

void RobotMotorManager::setZeroMotor(int motor_id) {
    if (motor_id < 0 || motor_id >= 16) {
        std::cerr << "[setZeroMotor] 无效电机ID: " << motor_id << std::endl;
        return;
    }
    // 单台同样走"发送+确认+重发",保证 Web 逐个置零与批量路径可靠性一致。
    // 不再自行加 api_mutex_(verified 内部已加,std::mutex 不可重入)。
    if (!setZeroMotorsVerified({motor_id}).empty())
        std::cerr << "[setZeroMotor] motor " << motor_id << " 置零未确认,请检查 CAN 链路" << std::endl;
}

void RobotMotorManager::enableAllMotors() {
    std::lock_guard<std::mutex> lock(api_mutex_);
    // v7.2: 四条 CAN 总线物理独立(socketcan 各持独立 socket / usb2can 各持独立串口),
    // 按腿(can_idx)分 4 组并行使能;每组内部仍按 100ms 串行间隔发 0xFC,
    // 同一总线不拥塞。总耗时由 16×100ms=1.6s 降为 4×100ms≈0.4s。
    // 组内顺序沿用全局序号(每腿 hip→thigh→calf→foot),与原串行顺序一致。
    std::vector<std::vector<int>> by_leg(4);
    for (int idx : connected_motors_) {
        by_leg[motor_map_[idx].can_idx].push_back(idx);
    }
    std::vector<std::thread> workers;
    workers.reserve(4);
    for (int leg = 0; leg < 4; ++leg) {
        workers.emplace_back([this, &by_leg, leg]() {
            for (int idx : by_leg[leg]) {
                enableMotor(idx);       // 内部已含 1ms 间隔
                usleep(100000);         // 同总线 100ms 间隔,确保电机响应
            }
        });
    }
    for (auto& t : workers) t.join();
}

void RobotMotorManager::disableAllMotors() {
    std::lock_guard<std::mutex> lock(api_mutex_);
    for (int idx : connected_motors_) {
        disableMotor(idx);
        usleep(100000);  // 100ms间隔，确保电机响应
    }
}

void RobotMotorManager::setZeroAllMotors() {
    // 并行验证版:16 台 ~0.1~0.3s(旧串行 100ms/台需 1.6s,且无确认)。
    // 保持原"已连接电机"范围;connected_motors_ 为空时(异常)兜底全 16 台。
    std::vector<int> ids = connected_motors_;
    if (ids.empty()) {
        ids.resize(16);
        for (int i = 0; i < 16; ++i) ids[i] = i;
    }
    const std::vector<int> failed = setZeroMotorsVerified(ids);
    if (!failed.empty()) {
        std::cerr << "[setZeroAllMotors] 置零未确认电机:";
        for (int id : failed) std::cerr << ' ' << id;
        std::cerr << std::endl;
    }
}

// ── 可控性探针(只读参考):发小角度偏移,检测反馈是否跟踪,不触发恢复 ──
bool RobotMotorManager::restoreProbePosition(int global_idx, float target,
                                             int timeout_ms, float tol) {
    if (global_idx < 0 || global_idx >= 16) return false;
    const auto& m = motor_map_[global_idx];
    const int leg = m.can_idx;
    const int idx = m.leg_motor_idx;
    EnableProbe& probe = enable_probes_[global_idx];
    if (!probe.active.load(std::memory_order_acquire)) return true;  // 未抬过,无需回位

    probe.vel = 0.0f;
    if (idx == 3) {
        // 轮关节是速度探针(kp=0):给 0 速度即停转,无位置基线可回
        probe.kp = 0.0f;
        probe.kd = 4.0f;
        std::this_thread::sleep_for(std::chrono::milliseconds(80));
        return true;
    }

    // 位置关节:恢复位置环(与探针同参数 kp=80/kd=3),目标改回基线后闭环等收敛
    probe.kp = 80.0f;
    probe.kd = 3.0f;
    probe.pos = target;

    const auto deadline = std::chrono::steady_clock::now()
                        + std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < deadline) {
        float q;
        {
            std::lock_guard<std::mutex> lk(fb_mutex_[leg]);
            q = leg_feedbacks_[leg].pos[idx];
        }
        if (std::abs(q - target) <= tol) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return false;
}

bool RobotMotorManager::probeMotorCheck(int global_idx,
                                        bool keep_calf_lifted,
                                        bool calf_pre_lifted) {
    if (global_idx < 0 || global_idx >= 16) return false;
    const auto& m = motor_map_[global_idx];
    const int leg = m.can_idx;
    const int idx = m.leg_motor_idx;

    // 读当前反馈基准
    float q0;
    {
        std::lock_guard<std::mutex> lk(fb_mutex_[leg]);
        q0 = leg_feedbacks_[leg].pos[idx];
    }

    // 记录探针前基线(仅在首次):批量模式小腿探针→轮探针复用同一基线;
    // 探针槽释放时清 valid,失败重测会重取当时位为基线(此时已回位,偏差很小)
    if (!probe_home_valid_[global_idx]) {
        probe_home_[global_idx] = q0;
        probe_home_valid_[global_idx] = true;
    }

    // 小腿抬起参数:小腿探针与轮探针共用。轮探针需先抬小腿让轮子离地再转,
    // 否则趴姿轮子着地,阻尼/静摩擦可能压不住导致足端检测失败(实机反馈)。
    constexpr float kCalfLiftRad = 0.10f;
    const float calf_lift_sign = (leg < 2) ? -1.0f : 1.0f;  // 轮端上抬:前负后正

    EnableProbe& probe = enable_probes_[global_idx];
    const bool is_wheel = (idx == 3);  // foot = 轮关节
    const bool is_calf  = (idx == 2);  // calf = 小腿关节
    if (is_wheel) {
        // 第一步:先激活同腿小腿保持探针,把小腿抬起 0.10rad 让轮子离地。
        // 参数与小腿探针一致(kp=80/kd=3,方向依据 MJCF 运动学前负后正);
        // 等 400ms 抬到位后重读轮基准 q0,以离地后的位置为探测基准。
        // calf_pre_lifted(批量模式):小腿探针刚保持抬起未回落,直接复用,
        // 避免"抬两次"(小腿探针抬一次+轮探针再抬一次)。
        if (!calf_pre_lifted) {
            {
                float q0_calf;
                {
                    std::lock_guard<std::mutex> lk(fb_mutex_[leg]);
                    q0_calf = leg_feedbacks_[leg].pos[2];
                }
                // 自抬小腿时记下小腿基线,轮探针结束时用它闭环回位
                if (!probe_home_valid_[leg * 4 + 2]) {
                    probe_home_[leg * 4 + 2] = q0_calf;
                    probe_home_valid_[leg * 4 + 2] = true;
                }
                EnableProbe& calf_probe = enable_probes_[leg * 4 + 2];
                calf_probe.kp = 80.0f;
                calf_probe.kd = 3.0f;
                calf_probe.pos = q0_calf + kCalfLiftRad * calf_lift_sign;
                calf_probe.vel = 0.0f;
                calf_probe.active.store(true, std::memory_order_release);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(400));
            {
                std::lock_guard<std::mutex> lk(fb_mutex_[leg]);
                q0 = leg_feedbacks_[leg].pos[idx];  // 轮已离地,重读基准
            }
        }

        // 第二步:轮关节速度探针: kp=0 kd=4 vel=0.5rad/s(约 28.6 deg/s)
        // 2026-10-11 实机:原 kd=2/vel=0.3(峰值力矩仅 0.6Nm)轮子着地
        // 静摩擦压不住,足端检测失败;加大阻尼到 kd=4(峰值 2Nm)、速度到
        // 0.5rad/s,并把轮探针保持时长加到 800ms(见下),理论净位移 0.4rad。
        probe.kp = 0.0f;
        probe.kd = 4.0f;
        probe.pos = 0.0f;
        probe.vel = 0.5f * m.dir;
    } else if (is_calf) {
        // 小腿探针: 明确把小腿向上抬(收腿、轮端离地)0.10rad(约 5.7deg),动作可见。
        // 方向依据 MJCF 模型(web/model/2/JXG.xml)运动学: calf 轴 +Y,前腿轮端
        // 偏移 (0.246,0,0.047) → dz/dθ=-0.246,前腿 calf 正向=轮子下压(伸腿);
        // 后腿几何镜像,负向=下压。趴姿≈零位、站立=伸腿(前+1.68/后-1.68)。
        // 故轮端上抬(收腿)方向为: 前腿(FL/FR, leg 0/1)取负、后腿(RL/RR)取正。
        // (2026-10-11 实机验证: 前正后负时小腿下压不可见,已翻转为前负后正;
        //  0.15rad 实机反馈幅度偏大,压缩到 0.10rad)
        // pos/q0 同为逻辑系(m.dir 修正后的统一关节系),这里不要再乘 m.dir,
        // 否则会退化成左右腿反向而非前后腿反向。
        // kCalfLiftRad/calf_lift_sign 定义在函数头(与轮探针的抬腿保持共用)。
        probe.kp = 80.0f;
        probe.kd = 3.0f;
        probe.pos = q0 + kCalfLiftRad * calf_lift_sign;
        probe.vel = 0.0f;
    } else {
        // 腿关节(髋/大腿)位置探针: kp=80 kd=3。
        // 注1:fixed_kp 为 160~200,探针需约一半才能克服关节摩擦/阻力;
        //     原 kp=20 力矩不足导致腿关节不动、误判不可控。
        // 注2:逻辑偏移 A·m.dir 经"反馈乘dir/指令乘dir"两次映射后,物理偏移
        //     恒为 +A(与 dir 无关),四条腿探针同物理方向。MJCF 零位(趴姿)下
        //     前腿腿段质心在 thigh 关节后方(x≈-0.135m),物理正转=抬起=抗重力
        //     (重力矩≈4.2Nm);后腿质心在前方,物理正转=下放=顺重力。
        //     2026-10-11 实机:m1/m5(FL/FR大腿)有概率假阴性——0.05rad×kp80
        //     最大 4Nm 与前腿重力矩近似平衡,位移卡在判据 0.005rad 边缘。
        //     大腿偏移加大到 0.10rad(8Nm,重力裕量近 2 倍,稳态位移≈0.05rad);
        //     髋为侧摆轴、趴姿负载小,维持 0.05rad。
        // 注3:限位安全——前/后腿 thigh 物理行程至少一侧 ≥1.56rad,趴姿零位
        //     附近 +0.10rad 不会触限位。
        const float offset = (idx == 1) ? 0.10f : 0.05f;  // idx1=thigh
        probe.kp = 80.0f;
        probe.kd = 3.0f;
        probe.pos = q0 + offset * m.dir;
        probe.vel = 0.0f;
    }
    probe.active.store(true, std::memory_order_release);

    // 腿线程 500Hz 发探针指令(腿关节 500ms,加长避免摩擦/惯量大的关节响应慢
    // 导致假阴性;轮关节 800ms,配合 kd=4/vel=0.5 保证足够净位移)
    std::this_thread::sleep_for(std::chrono::milliseconds(is_wheel ? 800 : 500));

    // 读反馈
    float q1;
    {
        std::lock_guard<std::mutex> lk(fb_mutex_[leg]);
        q1 = leg_feedbacks_[leg].pos[idx];
    }

    // 回位:闭环把关节拉回探针前基线,确认到位后再释放探针槽。
    // 原实现只把目标改回 q0 并固定保持 150ms 就释放,上层 Passive 是
    // kp=0/kd=8 纯阻尼(无位置保持力),150ms 内没拉回到位的关节会在释放后
    // 靠重力回落,实机表现为探针结束后关节停在偏移位没有回到原来位置。
    // keep_calf_lifted 的小腿不在此回位,由随后同腿轮探针结束时统一回位。
    const bool keep_lifted = (is_calf && keep_calf_lifted);
    if (!keep_lifted) {
        restoreProbePosition(global_idx, probe_home_[global_idx]);
        probe.active.store(false, std::memory_order_release);
        probe_home_valid_[global_idx] = false;
    }

    // 轮探针结束:把被保持抬起的小腿闭环拉回其探针前基线,再释放探针槽
    // (原实现只释放不拉回,小腿靠重力回落不彻底,会停在抬起位)
    if (is_wheel) {
        const int calf_idx = leg * 4 + 2;
        restoreProbePosition(calf_idx, probe_home_[calf_idx]);
        enable_probes_[calf_idx].active.store(false, std::memory_order_release);
        probe_home_valid_[calf_idx] = false;
    }

    if (is_wheel) {
        // 轮关节判据用净位移而非瞬时速度:批量探针时 4 台轮毂电机同时启动,
        // 供电电压跌落导致加速变慢,瞬时 |dq| 在 500ms 采样点上常低于阈值造成
        // 假阴性(单台测试供电充足就能过)。位移是累积量,只要转过就累积,
        // 0.3rad/s×0.5s 理论 0.15rad,判据 0.03rad(平均 0.06rad/s)留 5 倍裕量,
        // 且远超编码器噪声(约 0.001rad)。
        bool ok = std::abs(q1 - q0) > 0.03f;
        probe_results_[global_idx].store(ok, std::memory_order_release);
        probe_results_valid_[global_idx].store(true, std::memory_order_release);
        return ok;
    } else {
        bool ok = std::abs(q1 - q0) > 0.005f;  // 0.05rad 探针,判据 0.005 留裕量
        probe_results_[global_idx].store(ok, std::memory_order_release);
        probe_results_valid_[global_idx].store(true, std::memory_order_release);
        return ok;
    }
}

std::vector<bool> RobotMotorManager::probeAllMotorsCheck() {
    std::vector<bool> result(16, false);
    // 第一轮:4 条腿 CAN 总线物理独立,并行探针;每条腿内 4 台串行。
    // 小腿探完保持抬起、轮探针直接复用(单次连续抬起,不回落再抬),
    // 单腿 650+650+500+(800+200) ≈ 2.8s。
    std::vector<std::thread> workers;
    workers.reserve(4);
    for (int leg = 0; leg < 4; ++leg) {
        workers.emplace_back([this, &result, leg]() {
            for (int j = 0; j < 4; ++j) {
                // j==2(小腿)探完保持抬起,j==3(轮)复用已抬的小腿
                result[leg * 4 + j] = probeMotorCheck(leg * 4 + j,
                                                      j == 2, j == 3);
            }
        });
    }
    for (auto& t : workers) t.join();

    // 第二轮:失败台自动重测。批量并行时多台轮毂电机同时启动存在供电跌落/
    // CAN 拥挤,慢加速轮电机的瞬时响应达不到判据造成假阴性(单台测试即通过);
    // 冷却 300ms 后逐台串行重测(等效单台条件,无并行干扰),以重测结果为准。
    std::vector<int> failed;
    for (int i = 0; i < 16; ++i) {
        if (!result[i]) failed.push_back(i);
    }
    if (!failed.empty()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        for (int gidx : failed) {
            result[gidx] = probeMotorCheck(gidx);
        }
    }
    return result;
}

std::vector<bool> RobotMotorManager::probeLegCheck(int leg) {
    std::vector<bool> result(4, false);
    if (leg < 0 || leg >= 4) return result;
    // 腿 leg 的 4 台电机 global_idx = leg*4 + 0..3
    // 小腿探完保持抬起、轮探针直接复用(与 probeAllMotorsCheck 同策略)
    for (int j = 0; j < 4; ++j) {
        result[j] = probeMotorCheck(leg * 4 + j, j == 2, j == 3);
    }
    // 失败台自动重测(与 probeAllMotorsCheck 同理,消除并行/时序假阴性)
    std::vector<int> failed;
    for (int j = 0; j < 4; ++j) {
        if (!result[j]) failed.push_back(leg * 4 + j);
    }
    if (!failed.empty()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        for (int gidx : failed) {
            result[gidx - leg * 4] = probeMotorCheck(gidx);
        }
    }
    return result;
}

bool RobotMotorManager::hasProbeResults() const {
    for (int i = 0; i < 16; ++i) {
        if (!probe_results_valid_[i].load(std::memory_order_acquire)) return false;
    }
    return true;
}

bool RobotMotorManager::areAllMotorsProbedOk() const {
    for (int i = 0; i < 16; ++i) {
        if (!probe_results_valid_[i].load(std::memory_order_acquire)) return false;
        if (!probe_results_[i].load(std::memory_order_acquire)) return false;
    }
    return true;
}

bool RobotMotorManager::invalidateProbeResults() {
    bool had_results = false;
    for (int i = 0; i < 16; ++i) {
        // 仅作废 valid 标志:hasProbeResults/areAllMotorsProbedOk 均先查 valid,
        // 置 false 即等效"从未探过";probe_results_ 保留原值无副作用。
        if (probe_results_valid_[i].exchange(false, std::memory_order_acq_rel)) had_results = true;
    }
    return had_results;
}

void RobotMotorManager::initAllMotors() {

std::cerr << "initAllMotors";
    //setZeroAllMotors();
    enableAllMotors();
    usleep(100000);
    //setZeroAllMotors();
}

void RobotMotorManager::periodicReEnable() {
    // 不锁 api_mutex_(避免与 setZeroAll 等长操作互斥阻塞控制环);
    // sendControlCommand 内部对小帧(~16B)的写在 tty/can socket 层原子,
    // 与 TX 线程的命令帧交错不会破坏帧边界。
    // v7.2: 同 enableAllMotors,按腿分 4 路并行,组内 20ms 间隔,
    // 一轮耗时由 16×20ms=320ms 降为 4×20ms=80ms,缩短旁路发送与 500Hz
    // 控制流争用同一 socket 的时间窗。
    std::vector<std::vector<int>> by_leg(4);
    for (int idx : connected_motors_) {
        by_leg[motor_map_[idx].can_idx].push_back(idx);
    }
    std::vector<std::thread> workers;
    workers.reserve(4);
    for (int leg = 0; leg < 4; ++leg) {
        workers.emplace_back([this, &by_leg, leg]() {
            for (int idx : by_leg[leg]) {
                enableMotor(idx);      // 0xFC,内部 usleep(1000)
                usleep(19000);         // 1ms + 19ms = 20ms 同总线间距
            }
        });
    }
    for (auto& t : workers) t.join();
}
