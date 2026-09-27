#pragma once
#include <Eigen/Dense>
#include <vector>

struct JXGMotorCmd {
    float q;   // 位置
    float dq;  // 速度
    float tau; // 力矩
    float kp;  // 比例增益
    float kd;  // 微分增益
};

struct JXG_low_cmd {
    std::vector<JXGMotorCmd> motor_cmd;  // 长度16（12腿+4轮）
    JXG_low_cmd() {
        motor_cmd.resize(16);
    }
};
