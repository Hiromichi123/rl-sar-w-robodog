#ifndef MOTOR_MANAGER_H
#define MOTOR_MANAGER_H

#include "usb2can.h"
#include "SerialPort.h"
#include <Eigen/Dense>
#include <thread>
#include <mutex>
#include <vector>
#include <memory>
#include <atomic>
#include <array>

// 电机映射表结构
struct MotorMap {
    int can_idx;      // CAN设备索引（0-3）
    uint32_t can_id;  // 电机CAN ID
    int leg_motor_idx;// 腿内索引（0-2）
    float dir;        // 方向系数（±1）
    MotorType motor_type; // 电机型号
};

// 腿部控制指令（使用可原子操作的结构）
struct LegCommand {
    Eigen::RowVector4f tau;
    Eigen::RowVector4f vel;
    Eigen::RowVector4f pos;
    Eigen::RowVector4f kp;
    Eigen::RowVector4f kd;
};

// 腿部反馈数据
struct LegFeedback {
    Eigen::RowVector4f pos;
    Eigen::RowVector4f vel;
    Eigen::RowVector4f tau;
    Eigen::RowVector4f temp; // 绕组温度(°C)
};

class RobotMotorManager {
public:
    RobotMotorManager();
    ~RobotMotorManager();

    void init();
    void initAllMotors();
    void startThreads();
    void stopThreads();
    void sendLegCommand(int leg_index,
                       const Eigen::RowVector4f& tau,
                       const Eigen::RowVector4f& vel,
                       const Eigen::RowVector4f& pos,
                       const Eigen::RowVector4f& kp,
                       const Eigen::RowVector4f& kd);
    void sendAllMotorsCommand(const std::vector<float>& tau,
                          const std::vector<float>& vel,
                          const std::vector<float>& pos,
                          const std::vector<float>& kp,
                          const std::vector<float>& kd);                         
    void sendMotorCommand(int motor_global_idx,
                          float tau, 
                          float vel, 
                          float pos,            
                          float kp, 
                          float kd);

    Eigen::RowVector4f getMotorPos_leg(int leg_index);
    Eigen::RowVector4f getMotorVel_leg(int leg_index);
    Eigen::RowVector4f getMotorTorque_leg(int leg_index);
    Eigen::VectorXf getAllMotorPos();
    Eigen::VectorXf getAllMotorVel();
    Eigen::VectorXf getAllMotorTorque();
    std::vector<float> getAllMotorTemp();  // 获取所有电机绕组温度(°C)

    // 新增：一次性读取所有电机反馈（原子操作，保证数据一致性）
    struct AllMotorFeedback {
        Eigen::VectorXf pos;   // 16个电机位置
        Eigen::VectorXf vel;   // 16个电机速度
        Eigen::VectorXf tau;   // 16个电机力矩
        std::vector<float> temp; // 16个电机绕组温度(°C)
    };
    AllMotorFeedback getAllMotorFeedback(); 




    void enableMotor(int motor_id);
    void disableMotor(int motor_id);
    void setZero(int motor_id);
    void enableAllMotors();
    void disableAllMotors();
    void setZeroAllMotors();

private:
    void legThreadFunc(int leg_index);
    void sendControlCommand(int can_idx, uint32_t can_id, uint8_t cmd);

    std::atomic<bool> running_{false};
    std::vector<std::thread> leg_threads_;
    std::vector<std::mutex> fb_mutex_;
    std::mutex global_fb_mutex_;  // 新增：全局反馈锁，用于getAllMotorFeedback
    std::vector<LegFeedback> leg_feedbacks_;
    std::array<std::atomic<LegCommand*>, 4> leg_commands_; 

    USB2CANManager can_manager_;
    std::vector<int> can_device_indices_;
    std::vector<std::shared_ptr<SerialPort>> serials_;
    std::vector<MotorMap> motor_map_;
    std::mutex api_mutex_;

    static constexpr float P_MIN = -12.57f;
    static constexpr float P_MAX = 12.57f;
    static constexpr float V_MIN = -20.0f;
    static constexpr float V_MAX = 20.0f;
    static constexpr float T_MIN = -60.0f;
    static constexpr float T_MAX = 60.0f;
    std::vector<int> connected_motors_ = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};  // 16个电机全部使能
};

#endif // MOTOR_MANAGER_H