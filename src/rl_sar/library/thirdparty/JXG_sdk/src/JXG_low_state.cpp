#include "JXG_low_state.h"
#include "motor_manager.h"
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <atomic>

extern RobotMotorManager motor_manager;

static std::atomic<bool> imu_stream_enabled{true};


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

    // roll/pitch 来源二选一，yaw 一律用本地积分器（getYawFused：陀螺零偏补偿 + 静止冻结 ZUPT）。
    // 原因：6 轴 IMU 无磁力计，固件内部 yaw 会随陀螺零偏持续漂移（实测约 -6°/min），不能直接用。
    double roll = 0.0, pitch = 0.0;
    float raw_q[4];
    if (getQuaternion(raw_q)) {
        // 四元数路径：从固件四元数只提取 roll/pitch（ZYX 内旋）
        Eigen::Quaterniond q_raw(raw_q[0], raw_q[1], raw_q[2], raw_q[3]);
        q_raw.normalize();
        const double sinr_cosp = 2.0 * (q_raw.w() * q_raw.x() + q_raw.y() * q_raw.z());
        const double cosr_cosp = 1.0 - 2.0 * (q_raw.x() * q_raw.x() + q_raw.y() * q_raw.y());
        roll = std::atan2(sinr_cosp, cosr_cosp);
        const double sinp = 2.0 * (q_raw.w() * q_raw.y() - q_raw.z() * q_raw.x());
        pitch = (std::abs(sinp) >= 1.0) ? std::copysign(M_PI / 2.0, sinp) : std::asin(sinp);
    } else {
        // 欧拉角路径：roll/pitch 用 IMU 融合输出（加速度计修正，可靠）
        Eigen::Matrix<float, 1, 3> rpy = getAngle();
        roll  = rpy[0];
        pitch = rpy[1];
    }
    const double yaw = getYawFused();
    rpyToQuaternion(roll, pitch, yaw, state.imu_state.quaternion);

    // 陀螺仪
    state.imu_state.gyroscope[0] = gyro[0];
    state.imu_state.gyroscope[1] = gyro[1];
    state.imu_state.gyroscope[2] = gyro[2];

    // 加速度计
    state.imu_state.accelerometer[0] = acc[0];
    state.imu_state.accelerometer[1] = acc[1];
    state.imu_state.accelerometer[2] = acc[2];

    // 每 500 次调用打印一次 IMU 数据（~1秒 @ 500Hz 控制频率）
    static int imu_print_cnt = 0;
    if (++imu_print_cnt >= 500) {
        imu_print_cnt = 0;
        printf("[DM-IMU] quat=[%.3f, %.3f, %.3f, %.3f]  gyro=[%.3f, %.3f, %.3f]  acc=[%.3f, %.3f, %.3f]\n",
               state.imu_state.quaternion[0], state.imu_state.quaternion[1],
               state.imu_state.quaternion[2], state.imu_state.quaternion[3],
               state.imu_state.gyroscope[0], state.imu_state.gyroscope[1], state.imu_state.gyroscope[2],
               state.imu_state.accelerometer[0], state.imu_state.accelerometer[1], state.imu_state.accelerometer[2]);
    }
}

void resetYawZero() {
    resetFusedYaw();
    std::cout << "[DM-IMU] Yaw zero reset." << std::endl;
}

void updateJXGLowState(JXGLowState& state)
{
    fillIMUDataToJXGLowState(state);

    auto all_feedback = motor_manager.getAllMotorFeedback();

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
