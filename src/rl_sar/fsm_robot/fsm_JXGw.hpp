/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef JXGW_FSM_HPP
#define JXGW_FSM_HPP

#include "fsm.hpp"
#include "rl_sdk.hpp"
#include "motor_manager.h"
#include <unistd.h>  // isatty:仅交互终端打印RL进度,避免systemd下200Hz刷journald

extern RobotMotorManager motor_manager;  // 定义于 rl_real_JXG.cpp

namespace JXGw_fsm
{

class RLFSMStatePassive : public RLFSMState
{
public:
    RLFSMStatePassive(RL *rl) : RLFSMState(*rl, "RLFSMStatePassive") {}

    void Enter() override
    {
        rl.behavior_settled = true;  // passive 卸力即稳态
        // 每次进入趴姿即作废此前的起立门禁记录(置零校验/探针),与探针同语义:
        // 趴着时关节可能被人工搬动/零点漂移/电机断电重连,"已通过"不允许跨趴下
        // 周期沿用。下次起立前必须重新手动校验(「卧姿校验」check_stand + 探针),
        // posture 广播的 lie_done/probe_done 同步回落 false,前端胶囊转灰。
        rl.motors_ready_for_stand = false;
        const bool lie_invalidated = rl.lie_check_done_.exchange(false);
        rl.lie_check_ok_.store(false);
        // 仅当确有旧记录被作废时才提示(启动即 Passive/连续趴下不重复打扰)。
        if (motor_manager.invalidateProbeResults() || lie_invalidated)
        {
            rl.Notify("info", "已进入趴姿:此前探针/卧姿校验记录已作废,下次起立前请重新校验");
        }
        std::cout << LOGGER::NOTE << "Entered passive mode. Press '0' (Keyboard) or 'A' (Gamepad) to switch to RLFSMStateGetUp." << std::endl;
        rl.Notify("info", "Entered passive mode");
    }

    void Run() override
    {
        for (int i = 0; i < rl.params.Get<int>("num_of_dofs"); ++i)
        {
            fsm_command->motor_command.dq[i] = 0;
            fsm_command->motor_command.kp[i] = 0;
            fsm_command->motor_command.kd[i] = 8;
            fsm_command->motor_command.tau[i] = 0;
        }
    }

    void Exit() override {}

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::Num0)
        {
            // 起立置零校验门禁(与 Web/ROS getup 入口同一业务硬门禁,与探针同语义):
            // 仅信任「卧姿校验」(check_stand)写入的手动结果,趴下已作废,不跨趴下
            // 周期。不做当帧实时判定——趴姿下关节被搬动由"重新手动校验"兜底。
            // 通过路径仅 2 次原子读,200Hz 调用安全;失败明细由 HandleCheckStand
            // 写入 motors_not_ready_reason。
            const bool lie_check_missing = !rl.lie_check_done_.load();
            const bool zero_ok = !lie_check_missing && rl.lie_check_ok_.load();
            rl.motors_ready_for_stand = zero_ok;
            if (!zero_ok)
            {
                static int motor_warn_cnt = 0;
                if (++motor_warn_cnt % 20 == 1)
                {
                    const std::string why = lie_check_missing
                        ? std::string("未执行卧姿校验,请先执行「卧姿校验」")
                        : rl.motors_not_ready_reason;
                    std::cout << LOGGER::WARNING << why << std::endl;
                    rl.Notify("warning", why);
                }
                return state_name_;
            }
            if (rl.robot_state.imu.gyroscope[0] == 0.0f &&
                rl.robot_state.imu.gyroscope[1] == 0.0f &&
                rl.robot_state.imu.gyroscope[2] == 0.0f)
            {
                static int imu_warn_cnt = 0;
                if (++imu_warn_cnt % 20 == 1)
                {
                    std::cout << LOGGER::WARNING << "IMU no data! Cannot enter GetUp." << std::endl;
                    rl.Notify("warning", "IMU no data! Cannot enter GetUp.");
                }
                return state_name_;
            }
            return "RLFSMStateGetUp";
        }
        // Passive 态响应趴下请求: 直接切入主动趴下插值(GetDown 2s 插值完成后自动回
        // Passive)。修复: 原先 Num9 仅 locomotion/GetUp 响应, demo 的 stop(P)先切到
        // Passive 后, getdown(Num9) 成死命令, 机器人保持站立假 DONE。
        if (rl.control.current_keyboard == Input::Keyboard::Num9)
        {
            // 趴姿重趴=受控插值到已记录趴姿,本身安全;不再检查 motors_ready_for_stand——
            // 该标志已在进入趴姿时清除(Passive::Enter),此处若再要求会把"趴下后再趴下"
            // 误拒成死命令。CAN 反馈新鲜度已由 Web/ROS getdown 命令入口独立把关。
            return "RLFSMStateGetDown";
        }
        return state_name_;
    }
};

class RLFSMStateSafeHold : public RLFSMState
{
public:
    RLFSMStateSafeHold(RL *rl) : RLFSMState(*rl, "RLFSMStateSafeHold") {}

    // 接管时刻的关节角(状态帧坐标)与站立 PD 增益,Enter 时一次性抓取/缓存,
    // Run() 200Hz 只做 POD 写入,不在实时路径上解析 YAML。
    std::vector<float> hold_pos;
    std::vector<float> hold_kp;
    std::vector<float> hold_kd;

    void Enter() override
    {
        const int n = rl.params.Get<int>("num_of_dofs");
        hold_pos = fsm_state->motor_state.q;
        hold_kp  = rl.params.Get<std::vector<float>>("fixed_kp");
        hold_kd  = rl.params.Get<std::vector<float>>("fixed_kd");
        if ((int)hold_pos.size() != n) hold_pos.assign(n, 0.0f);
        if ((int)hold_kp.size()  != n) hold_kp.assign(n, 0.0f);
        if ((int)hold_kd.size()  != n) hold_kd.assign(n, 1.0f);
        rl.behavior_settled = true;  // 原地保持本身即稳态
        // 两个进入来源:①启动时检测到站立构型(崩溃重启接管) ②运行时 CAN 断线等
        // 安全事件主动退出 RL。行为一致:当前位 fixed PD 保持、速度清 0。
        std::cout << LOGGER::WARNING
                  << "[SafeHold] Motors now HOLD the current pose (fixed PD, dq=0, tau=0). "
                     "Send 'getdown' to lie down safely, or 'getup' to re-center to the "
                     "standing pose." << std::endl;
        rl.Notify("warning", "SafeHold: holding current pose. Send getdown (safe) or getup.");
    }

    void Run() override
    {
        for (int i = 0; i < (int)hold_pos.size(); ++i)
        {
            fsm_command->motor_command.q[i]   = hold_pos[i];
            fsm_command->motor_command.dq[i]  = 0.0f;
            fsm_command->motor_command.kp[i]  = hold_kp[i];
            fsm_command->motor_command.kd[i]  = hold_kd[i];
            fsm_command->motor_command.tau[i] = 0.0f;
        }
    }

    void Exit() override {}

    std::string CheckChange() override
    {
        // 受控趴下:2s 插值到收拢位后自动转 Passive
        if (rl.control.current_keyboard == Input::Keyboard::Num9)
            return "RLFSMStateGetDown";
        // 重新站立对中:从当前位 1s 插值到 default_dof_pos(GetUp 内部识别非 Passive 来源)
        if (rl.control.current_keyboard == Input::Keyboard::Num0)
            return "RLFSMStateGetUp";
        // 急停/卸载:绝不直接切 Passive(kp=0/kd=8 会让站立机器人砸落),
        // 重定向为受控 GetDown,插值结束后自然到达 Passive。
        if (rl.control.current_keyboard == Input::Keyboard::P)
        {
            static int stop_warn_cnt = 0;
            if (++stop_warn_cnt % 20 == 1)
            {
                std::cout << LOGGER::WARNING << "[SafeHold] passive/stop redirected to controlled GetDown (no free release while standing)" << std::endl;
                rl.Notify("warning", "SafeHold: executing controlled getdown instead of passive release");
            }
            return "RLFSMStateGetDown";
        }
        // 行走必须先 getup 对中,防止以接管姿态直接进策略
        if (rl.control.current_keyboard == Input::Keyboard::Num1)
        {
            static int loc_warn_cnt = 0;
            if (++loc_warn_cnt % 20 == 1)
            {
                std::cout << LOGGER::WARNING << "[SafeHold] locomotion rejected: send getup first" << std::endl;
                rl.Notify("warning", "Please getup first, then locomotion");
            }
        }
        return state_name_;
    }
};

class RLFSMStateGetUp : public RLFSMState
{
public:
    RLFSMStateGetUp(RL *rl) : RLFSMState(*rl, "RLFSMStateGetUp") {}

    float percent_pre_getup = 0.0f;
    float percent_getup = 0.0f;
    std::vector<float> pre_running_pos = {
        0.00, 0.0, 0.0, 0.00,
        0.00, 0.0, 0.0, 0.00,
        0.00, 0.0, 0.0, 0.00,
        0.00, 0.0, 0.0, 0.00
    };
    bool stand_from_passive = true;

    void Enter() override
    {
        percent_pre_getup = 0.0f;
        percent_getup = 0.0f;
        rl.behavior_settled = false;  // 站立插值未完成, 未到位
        if (rl.fsm.previous_state_->GetStateName() == "RLFSMStatePassive")
        {
            stand_from_passive = true;
        }
        else
        {
            stand_from_passive = false;
        }
        rl.now_state = *fsm_state;
        if (rl.fsm.previous_state_->GetStateName() != "RLFSMStateDiffDrive")
            rl.start_state = rl.now_state;
        rl.Notify("info", "Entered getup (standing up)");
    }

    void Run() override
    {
        if (stand_from_passive)
        {
            if (Interpolate(percent_pre_getup, rl.now_state.motor_state.q, pre_running_pos, 1.0f, "Pre Getting up", true)) return;
            if (Interpolate(percent_getup, pre_running_pos, rl.params.Get<std::vector<float>>("default_dof_pos"), 2.0f, "Getting up", true)) return;
        }
        else
        {
            if (Interpolate(percent_getup, rl.now_state.motor_state.q, rl.params.Get<std::vector<float>>("default_dof_pos"), 1.0f, "Getting up", true)) return;
        }
        if (percent_getup >= 1.0f) rl.behavior_settled = true;  // 已站稳, 可接受 locomotion/getdown
    }

    void Exit() override {}

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::P)
        {
            return "RLFSMStatePassive";
        }
        if (percent_getup >= 1.0f)
        {
            if (rl.DiffDriveRequest() == 1 && rl.CanEnterDiffDrive())
                return "RLFSMStateDiffDrive";
            if (rl.control.current_keyboard == Input::Keyboard::Num1)
            {
                return "RLFSMStateRLLocomotion";
            }
            else if (rl.control.current_keyboard == Input::Keyboard::Num9)
            {
                return "RLFSMStateGetDown";
            }
        }
        return state_name_;
    }
};

class RLFSMStateGetDown : public RLFSMState
{
public:
    RLFSMStateGetDown(RL *rl) : RLFSMState(*rl, "RLFSMStateGetDown") {}

    float percent_getdown = 0.0f;
    std::vector<float> target_pos;  // 本次趴下的目标姿态
    bool use_fallback = false;      // start_state 无有效趴姿记录时回退全零收拢位

    void Enter() override
    {
        percent_getdown = 0.0f;
        rl.behavior_settled = false;  // 趴下插值未完成; 结束自动转 Passive 时置 true
        rl.now_state = *fsm_state;
        // 目标=start_state(上次 GetUp::Enter 记录的真实趴姿)。但服务重启后未经过
        // GetUp 时 start_state≈当前站立姿态, 插值"站立→站立"等于原地不动(假趴下)。
        // 检测到目标与当前几乎一致时, 回退到全零收拢位(腿垂直收拢、重心降低);
        // 完成后把目标写回 start_state 自愈, 后续 GetUp/GetDown 即用真实姿态。
        float max_diff = 0.0f;
        const auto &cur = rl.now_state.motor_state.q;
        const auto &rec = rl.start_state.motor_state.q;
        for (size_t i = 0; i < cur.size() && i < rec.size(); ++i)
            max_diff = std::max(max_diff, std::abs(cur[i] - rec[i]));
        use_fallback = (max_diff < 0.3f);
        if (use_fallback)
        {
            const int n = rl.params.Get<int>("num_of_dofs");
            target_pos.assign(n, 0.0f);
            rl.Notify("info", "no recorded lying posture, fallback to crouch-zero getdown");
        }
        rl.Notify("info", "Entered getdown (lying down)");
    }

    void Run() override
    {
        const std::vector<float> &tgt = use_fallback ? target_pos : rl.start_state.motor_state.q;
        Interpolate(percent_getdown, rl.now_state.motor_state.q, tgt, 2.0f, "Getting down", true);
    }

    void Exit() override
    {
        // 自愈: 把本次实际达到的目标姿态记为趴姿, 供后续 GetUp 起点校验/GetDown 复用
        const std::vector<float> &tgt = use_fallback ? target_pos : rl.start_state.motor_state.q;
        rl.start_state.motor_state.q = tgt;
    }

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::P || percent_getdown >= 1.0f)
        {
            return "RLFSMStatePassive";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num0)
        {
            return "RLFSMStateGetUp";
        }
        return state_name_;
    }
};

class RLFSMStateDiffDrive : public RLFSMState
{
public:
    RLFSMStateDiffDrive(RL *rl) : RLFSMState(*rl, "RLFSMStateDiffDrive") {}
    void Enter() override { rl.EnterDiffDrive(); }
    void Run() override { rl.RunDiffDrive(fsm_command); }
    void Exit() override { rl.ExitDiffDrive(); }
    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::Num9 ||
            rl.control.current_keyboard == Input::Keyboard::P)
            return "RLFSMStateGetDown";
        if (rl.DiffDriveRequest() == 2)
            return "RLFSMStateGetUp";
        return state_name_;
    }
};

class RLFSMStateRLLocomotion : public RLFSMState
{
public:
    RLFSMStateRLLocomotion(RL *rl) : RLFSMState(*rl, "RLFSMStateRLLocomotion") {}

    float percent_transition = 0.0f;

    void Enter() override
    {
        percent_transition = 0.0f;
        rl.episode_length_buf = 0;
        rl.behavior_settled = false;  // 模型加载前未到位
        rl.Notify("info", "Entered RL locomotion");

        rl.config_name = "himloco";
        std::string robot_config_path = rl.robot_name + "/" + rl.config_name;
        try
        {
            rl.InitRL(robot_config_path);
            rl.now_state = *fsm_state;
            rl.behavior_settled = true;  // 策略接管, 到位
            rl.ArmActionWatchdog();      // 武装动作看门狗(推理卡死 100ms 自动 passive)
        }
        catch (const std::exception& e)
        {
            std::cout << LOGGER::ERROR << "InitRL() failed: " << e.what() << std::endl;
            rl.Notify("error", std::string("InitRL() failed: ") + e.what());
            rl.rl_init_done = false;
            rl.fsm.RequestStateChange("RLFSMStatePassive");
        }
    }

    void Run() override
    {
        if (!rl.rl_init_done) rl.rl_init_done = true;

        // 仅在交互终端打印进度行;systemd 下 stdout 是 journald 管道,200Hz 的 cout+flush
        // 会变成每秒 200 次 write()+journald IPC(实测周期性 ~3ms 抖动、抬高 loop_control)。
        static const bool stdout_is_tty = ::isatty(STDOUT_FILENO);
        if (stdout_is_tty)
        {
            std::cout << "\r\033[K" << std::flush << LOGGER::INFO << "RL Controller [" << rl.config_name << "] x:" << rl.control.x << " y:" << rl.control.y << " yaw:" << rl.control.yaw << std::flush;
        }
        RLControl();
    }

    void Exit() override
    {
        rl.rl_init_done = false;
    }

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::P)
        {
            return "RLFSMStatePassive";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num9)
        {
            return "RLFSMStateGetDown";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num0)
        {
            return "RLFSMStateGetUp";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num1)
        {
            return "RLFSMStateRLLocomotion";
        }
        return state_name_;
    }
};

} // namespace JXGw_fsm

class JXGwFSMFactory : public FSMFactory
{
public:
    JXGwFSMFactory(const std::string& initial) : initial_state_(initial) {}
    std::shared_ptr<FSMState> CreateState(void *context, const std::string &state_name) override
    {
        RL *rl = static_cast<RL *>(context);
        if (state_name == "RLFSMStatePassive")
            return std::make_shared<JXGw_fsm::RLFSMStatePassive>(rl);
        else if (state_name == "RLFSMStateSafeHold")
            return std::make_shared<JXGw_fsm::RLFSMStateSafeHold>(rl);
        else if (state_name == "RLFSMStateGetUp")
            return std::make_shared<JXGw_fsm::RLFSMStateGetUp>(rl);
        else if (state_name == "RLFSMStateGetDown")
            return std::make_shared<JXGw_fsm::RLFSMStateGetDown>(rl);
        else if (state_name == "RLFSMStateDiffDrive")
            return std::make_shared<JXGw_fsm::RLFSMStateDiffDrive>(rl);
        else if (state_name == "RLFSMStateRLLocomotion")
            return std::make_shared<JXGw_fsm::RLFSMStateRLLocomotion>(rl);
        return nullptr;
    }
    std::string GetType() const override { return "JXGw"; }
    std::vector<std::string> GetSupportedStates() const override
    {
        return {
            "RLFSMStatePassive",
            "RLFSMStateSafeHold",
            "RLFSMStateGetUp",
            "RLFSMStateGetDown",
            "RLFSMStateDiffDrive",
            "RLFSMStateRLLocomotion"
        };
    }
    std::string GetInitialState() const override { return initial_state_; }
private:
    std::string initial_state_;
};

REGISTER_FSM_FACTORY(JXGwFSMFactory, "RLFSMStatePassive")

#endif // JXGW_FSM_HPP
