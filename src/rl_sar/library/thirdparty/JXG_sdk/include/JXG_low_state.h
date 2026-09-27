#ifndef JXG_LOW_STATE_H
#define JXG_LOW_STATE_H

#include <vector>
#include <Eigen/Dense>
#include "IMU_Read.h"
#include "motor_manager.h"

struct JXGIMUState {
    // 四元数（姿态表示）：[w, x, y, z]
    double quaternion[4] = {1.0, 0.0, 0.0, 0.0};

    // 陀螺仪数据（角速度）：[x, y, z]，单位：rad/s
    double gyroscope[3] = {0.0, 0.0, 0.0};

    // 加速度计：[x, y, z]，单位：m/s²
    double accelerometer[3] = {0.0, 0.0, 9.81};

    // w3 当前 DM-IMU 没有 INSGPS 输入，保留字段用于 ROS 遥测格式兼容。
    double position_north = 0.0;
    double position_east = 0.0;
    double position_down = 0.0;
    double velocity_body[3] = {0.0, 0.0, 0.0};
};

struct JXGMotorState {
    double q = 0.0;
    double dq = 0.0;
    double tau_est = 0.0;
};

struct JXGLowState {
    JXGIMUState imu_state;
    std::vector<JXGMotorState> motor_state;
    explicit JXGLowState(size_t num_motors) : motor_state(num_motors) {}
};

void rpyToQuaternion(double roll, double pitch, double yaw, double quaternion[4]);
void fillIMUDataToJXGLowState(JXGLowState &state);
void updateJXGLowState(JXGLowState &state);
void resetYawZero();
void SetJXGIMUStreamEnabled(bool enabled);
bool IsJXGIMUStreamEnabled();

#endif  // JXG_LOW_STATE_H
