// motor_diag.cpp - 灵足(RobStride)电机状态诊断工具
// 用途：读取 16 个电机的反馈（位置/速度/力矩/温度），判断在线状态
// 重点排查右侧腿 FR(m4~m7) / RR(m12~m15) 是否在线、是否过热
//
// 安全说明：本工具只读取反馈，发送 kp=0/kd=8 的阻尼命令（与 passive 一致），
// 不会使能电机、不会输出位置控制力矩。运行前必须停掉 rl_real_JXG，避免 CAN 冲突。
#include "motor_manager.h"
#include <iostream>
#include <iomanip>
#include <vector>
#include <thread>
#include <chrono>

static const char* kMotorName[16] = {
    "FL_hip",  "FL_thigh", "FL_calf", "FL_wheel",
    "FR_hip",  "FR_thigh", "FR_calf", "FR_wheel",
    "RL_hip",  "RL_thigh", "RL_calf", "RL_wheel",
    "RR_hip",  "RR_thigh", "RR_calf", "RR_wheel",
};

// 右侧腿电机索引：FR = 4..7, RR = 12..15
static bool isRightSide(int idx) {
    return (idx >= 4 && idx <= 7) || (idx >= 12 && idx <= 15);
}

int main() {
    RobotMotorManager mm;

    std::cout << "[diag] 打开 CAN 串口..." << std::endl;
    mm.init();

    // 启动前先写入安全命令（kp=0 无力矩，kd=8 阻尼，与 passive 一致），
    // 避免 legThreadFunc 读到未初始化的命令导致电机乱动。
    std::vector<float> tau(16, 0.0f), vel(16, 0.0f), pos(16, 0.0f);
    std::vector<float> kp(16, 0.0f), kd(16, 8.0f);
    mm.sendAllMotorsCommand(tau, vel, pos, kp, kd);

    std::cout << "[diag] 启动收发线程..." << std::endl;
    mm.startThreads();

    // 等待反馈稳定
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    const int kRounds = 6;
    const int kIntervalMs = 500;

    for (int r = 0; r < kRounds; ++r) {
        auto fb = mm.getAllMotorFeedback();

        std::cout << "\n===== 采样 " << (r + 1) << "/" << kRounds
                  << " =====（>>> 为右侧腿，temp<=0 视为无反馈/离线）" << std::endl;
        for (int i = 0; i < 16; ++i) {
            float temp = (i < (int)fb.temp.size()) ? fb.temp[i] : 0.0f;
            bool online = temp > 1.0f;  // 在线电机的 MOS 温度通常 > 环境温度

            std::cout << (isRightSide(i) ? ">>> " : "    ")
                      << std::setw(2) << i << " " << std::setw(9) << std::left << kMotorName[i] << std::right
                      << "  pos=" << std::fixed << std::setprecision(3) << std::setw(8) << fb.pos[i]
                      << "  vel=" << std::setw(8) << fb.vel[i]
                      << "  tau=" << std::setw(8) << fb.tau[i]
                      << "  temp=" << std::setprecision(1) << std::setw(5) << temp << "C"
                      << (online ? "" : "  [OFFLINE]")
                      << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(kIntervalMs));
    }

    std::cout << "\n[diag] 停止线程..." << std::endl;
    mm.stopThreads();
    return 0;
}
