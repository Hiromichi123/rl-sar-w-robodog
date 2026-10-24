#include "JXG_low_state.h"
#include "motor_manager.h"
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <atomic>

extern RobotMotorManager motor_manager;

static bool yaw_initialized = false;
static double yaw_initial = 0.0;
static std::atomic<bool> imu_stream_enabled{true};
// 缓存初始四元数的 yaw 补偿旋转（避免每帧重复构造）
static Eigen::Quaterniond yaw_offset_conj(1, 0, 0, 0);


void SetJXGIMUStreamEnabled(bool enabled) {
    const bool was_enabled = imu_stream_enabled.exchange(enabled);
    if (enabled && !was_enabled) {
        resetYawZero();
        std::cout << "[DM-IMU] ROS gate opened: real IMU stream enabled." << std::endl;
    } else if (!enabled && was_enabled) {
        std::cout << "[DM-IMU] ROS gate closed: IMU output frozen to zero." << std::endl;
    }
}

bool IsJXGIMUStreamEnabled() {
    return imu_stream_enabled.load();
}

// ROS can still close the stream for tests, but default startup uses real IMU data.
static bool ShouldFreezeIMU() {
    return !IsJXGIMUStreamEnabled();
}

static void FillFrozenIMUState(JXGLowState &state) {
    state.imu_state.quaternion[0] = 1.0;
    state.imu_state.quaternion[1] = 0.0;
    state.imu_state.quaternion[2] = 0.0;
    state.imu_state.quaternion[3] = 0.0;
    state.imu_state.gyroscope[0] = 0.0;
    state.imu_state.gyroscope[1] = 0.0;
    state.imu_state.gyroscope[2] = 0.0;
    state.imu_state.accelerometer[0] = 0.0;
    state.imu_state.accelerometer[1] = 0.0;
    state.imu_state.accelerometer[2] = 0.0;
    state.imu_state.position_north = 0.0;
    state.imu_state.position_east = 0.0;
    state.imu_state.position_down = 0.0;
    state.imu_state.velocity_body[0] = 0.0;
    state.imu_state.velocity_body[1] = 0.0;
    state.imu_state.velocity_body[2] = 0.0;
}

// 四元数 → RPY（定义在文件尾部，前向声明供 fillIMUDataToJXGLowState 使用）
inline void quaternionToRPY(float w, float x, float y, float z, double& roll, double& pitch, double& yaw);

// RPY → 四元数（w, x, y, z），ZYX 内旋顺序
void rpyToQuaternion(double roll, double pitch, double yaw, double quaternion[4]) {
    Eigen::Quaterniond q;
    q = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
      * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
    quaternion[0] = q.w();
    quaternion[1] = q.x();
    quaternion[2] = q.y();
    quaternion[3] = q.z();
}

void fillIMUDataToJXGLowState(JXGLowState &state) {
    if (ShouldFreezeIMU()) {
        FillFrozenIMUState(state);
        return;
    }

    Eigen::Matrix<float, 1, 3> gyro = getAngleVelocity();
    Eigen::Matrix<float, 1, 3> acc  = get_com_Acc();

    float raw_q[4];
    if (getQuaternion(raw_q)) {
        // ========== 四元数路径：直接用 DM-IMU 输出的四元数 ==========
        Eigen::Quaterniond q_raw(raw_q[0], raw_q[1], raw_q[2], raw_q[3]);
        q_raw.normalize();

        // 首次收到四元数：记录初始 yaw，构造补偿旋转
        if (!yaw_initialized) {
            // 从四元数提取 yaw (ZYX)
            double siny = 2.0 * (q_raw.w() * q_raw.z() + q_raw.x() * q_raw.y());
            double cosy = 1.0 - 2.0 * (q_raw.y() * q_raw.y() + q_raw.z() * q_raw.z());
            yaw_initial = std::atan2(siny, cosy);
            yaw_initialized = true;
            // 构造纯 yaw 旋转的四元数并取共轭（用于左乘消除初始航向）
            yaw_offset_conj = Eigen::Quaterniond(
                std::cos(-yaw_initial / 2.0), 0, 0, std::sin(-yaw_initial / 2.0));
            std::cout << "[DM-IMU] Yaw initialized to: " << yaw_initial << " rad ("
                      << yaw_initial * 180.0 / M_PI << " deg)" << std::endl;
        }

        // q_relative = q_yaw_offset_conj * q_raw  （消除初始航向，保留 roll/pitch）
        Eigen::Quaterniond q_relative = yaw_offset_conj * q_raw;
        q_relative.normalize();

        // 硬件四元数 yaw 无磁力计会随陀螺零偏持续漂移(实测约 -6°/min):
        // 这里只取重力对准可靠的 roll/pitch,yaw 强制改用本地积分器
        // (陀螺零偏补偿 + 静止冻结 ZUPT,上电为 0 相对航向),与下方欧拉路径同源。
        double roll, pitch, yaw_hw;
        quaternionToRPY(static_cast<float>(q_relative.w()), static_cast<float>(q_relative.x()),
                        static_cast<float>(q_relative.y()), static_cast<float>(q_relative.z()),
                        roll, pitch, yaw_hw);
        rpyToQuaternion(roll, pitch, static_cast<double>(getYawFused()), state.imu_state.quaternion);

    } else {
        // ========== 欧拉角路径（当前 DM-IMU 配置只输出 加速度/角速度/欧拉角） ==========
        // roll/pitch 用 IMU 融合输出（加速度计修正，可靠）；
        // yaw 改用本地积分器：陀螺零偏补偿 + 静止冻结(ZUPT)，上电为 0（相对航向）。
        // 原因：6 轴 IMU 无磁力计，其内部 yaw 会随陀螺零偏持续漂移（实测约 -6°/min）。
        Eigen::Matrix<float, 1, 3> rpy = getAngle();
        double roll  = rpy[0];
        double pitch = rpy[1];
        double yaw   = getYawFused();
        rpyToQuaternion(roll, pitch, yaw, state.imu_state.quaternion);
    }

    // 陀螺仪
    state.imu_state.gyroscope[0] = gyro[0];
    state.imu_state.gyroscope[1] = gyro[1];
    state.imu_state.gyroscope[2] = gyro[2];

    // 加速度计
    state.imu_state.accelerometer[0] = acc[0];
    state.imu_state.accelerometer[1] = acc[1];
    state.imu_state.accelerometer[2] = acc[2];

    // 注意:本函数由实时线程(GetState→200Hz控制环)高频调用,
    // 禁止在此 printf/stdout 写入——journal 管道慢写会持 stdio 锁阻塞,
    // 曾造成 loop_control 单帧迟 6.2ms 的孤立超期;IMU 数据经 ROS 话题进数据看板,无需控制台转储。
}

void resetYawZero() {
    yaw_initialized = false;
    yaw_offset_conj = Eigen::Quaterniond(1, 0, 0, 0);
    resetFusedYaw();
    std::cout << "[DM-IMU] Yaw zero reset. Will re-initialize on next update." << std::endl;
}

void updateJXGLowState(JXGLowState& state)
{
    fillIMUDataToJXGLowState(state);

    // 零分配:本函数仅由 200Hz 控制环调用,静态缓冲区线程安全。
    static RobotMotorManager::AllMotorFeedback all_feedback = []{
        RobotMotorManager::AllMotorFeedback a;
        a.pos.resize(16);
        a.vel.resize(16);
        a.tau.resize(16);
        a.temp.resize(16);
        a.error.assign(16, 0.0f);
        return a;
    }();
    motor_manager.getAllMotorFeedback(all_feedback);

    const int expected_motors = 16;
    const int n = std::min<int>(expected_motors, std::min({(int)all_feedback.pos.size(),
                                             (int)all_feedback.vel.size(),
                                             (int)all_feedback.tau.size()}));

    for (int i = 0; i < n; ++i) {
        state.motor_state[i].q       = all_feedback.pos[i];
        state.motor_state[i].dq      = all_feedback.vel[i];
        state.motor_state[i].tau_est = all_feedback.tau[i];
    }

    static bool warned = false;
    if (n < expected_motors && !warned) {
        std::cerr << "[updateJXGLowState] 警告：电机反馈数量只有 " << n
                  << " 个，期望 " << expected_motors << " 个！请检查 motor_manager 配置" << std::endl;
        warned = true;
    }
}

inline void quaternionToRPY(float w, float x, float y, float z, double& roll, double& pitch, double& yaw)
{
    double sinr_cosp = 2 * (w * x + y * z);
    double cosr_cosp = 1 - 2 * (x * x + y * y);
    roll = std::atan2(sinr_cosp, cosr_cosp);

    double sinp = 2 * (w * y - z * x);
    if (std::abs(sinp) >= 1)
        pitch = std::copysign(M_PI / 2, sinp);
    else
        pitch = std::asin(sinp);

    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    yaw = std::atan2(siny_cosp, cosy_cosp);
}
