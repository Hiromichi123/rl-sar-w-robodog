// C3-fix: 一键安全起立(safe_getup)模块(从 rl_real_JXG.cpp 拆分)
// 包含: SafeGetupStep / FinishSafeGetup / StartSafeGetup / SafeGetupEnterGap /
//       LaunchSafeGetupProbe / TickSafeGetup
// 依赖: rl_real_JXG.hpp 中声明的成员变量(sg_, fsm, rl_init_done, behavior_settled,
//       manual_rl_block_, control, robot_state, motors_not_ready_reason)
//       以及成员函数 Notify/PublishRosFeedback/CheckCanFeedbackFresh/CheckMotorInitPosition/
//       HandleCheckStand/CheckStandingPose/HandleCheckStanding

#include "rl_real_JXG.hpp"

// 编译层选择(必须与 rl_real_JXG.cpp 保持一致)
#if defined(USE_ROS2_REAL_CONTROL)
#define JXG_ROS_LAYER 1
#else
#define JXG_ROS_LAYER 0
#endif
#if defined(JXG_ENABLE_WEB_BRIDGE)
#define JXG_WEB_LAYER 1
#else
#define JXG_WEB_LAYER 0
#endif

static double SgSteadyNowSec()
{
    return std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

void RL_Real::SafeGetupStep(const std::string &step, const std::string &state,
                            const std::string &extra)
{
    // 保存步骤状态供页面刷新后回显:0=idle,1=run,2=ok,3=fail,4=skip
    static const char *step_names[5] = {"lie", "probe", "getup", "stand", "rl"};
    int step_idx = -1;
    for (int i = 0; i < 5; ++i) {
        if (step == step_names[i]) { step_idx = i; break; }
    }
    if (step_idx >= 0) {
        int s = 0;  // idle
        if (state == "run") s = 1;
        else if (state == "ok") s = 2;
        else if (state == "fail") s = 3;
        else if (state == "skip") s = 4;
        this->sg_.step_states[step_idx].store(s);
    }

    std::string content = "step " + step + " " + state;
    if (!extra.empty()) content += " " + extra;
    this->Notify("safe_getup", content);
}

// 推送当前 safe_getup 全量状态(供页面刷新/重连后回显)
void RL_Real::SafeGetupStatus()
{
    static const char *step_names[5] = {"lie", "probe", "getup", "stand", "rl"};
    static const char *state_names[5] = {"idle", "run", "ok", "fail", "skip"};
    const bool active = this->sg_.active.load();
    // 先推送是否活跃
    this->Notify("safe_getup", active ? "status active" : "status idle");
    // 推送每个步骤的当前状态
    for (int i = 0; i < 5; ++i) {
        int s = this->sg_.step_states[i].load();
        if (s < 0 || s > 4) s = 0;
        // 页面刷新回显:只推送非 idle 的步骤,减少噪音
        if (s != 0) {
            this->Notify("safe_getup", std::string("step ") + step_names[i] + " " + state_names[s]);
        }
    }
}

void RL_Real::FinishSafeGetup(bool ok, const std::string &step, const std::string &reason)
{
    if (!this->sg_.active.load()) return;
    if (ok)
    {
        this->Notify("safe_getup", "done ok");
        if (this->sg_.want_rl.load())
        {
            PublishRosFeedback("safe_getup: done (standing, locomotion active)");
            this->Notify("success", "安全起立完成：卧姿、探针、站姿校验通过，已进入强化学习 ✅");
        }
        else
        {
            PublishRosFeedback("safe_getup: done (standing)");
            this->Notify("success", "安全起立完成：卧姿、探针、站姿校验全部通过 ✅");
        }
    }
    else
    {
        this->SafeGetupStep(step, "fail");
        this->Notify("safe_getup", "done fail " + step + " " + reason);
        PublishRosFeedback("safe_getup: failed at " + step + " (" + reason + ")");
        this->Notify("error", "安全起立已中止（" + step + "）：" + reason);
    }
    this->sg_.phase.store(static_cast<int>(SafeGetupPhase::kIdle));
    this->sg_.probe_busy.store(false);
    this->sg_.active.store(false);
    // 重置步骤状态(下次启动时从全 idle 开始)
    for (int i = 0; i < 5; ++i) this->sg_.step_states[i].store(0);
}

std::string RL_Real::StartSafeGetup(bool want_rl)
{
    if (this->sg_.active.load()) return "safe getup already running";

    std::string fsm = this->fsm.GetCurrentStateName();
    if (fsm.empty()) fsm = "unknown";

    const bool locomotion_settled =
        (fsm == "RLFSMStateRLLocomotion" && this->rl_init_done);
    if (locomotion_settled)
        return "already in locomotion, no need to get up";
    if (fsm == "RLFSMStateRLLocomotion" && !this->rl_init_done)
        return "rl model is loading, please wait";
    if (fsm == "RLFSMStateGetDown")
        return "robot is getting down, wait until settled";
    if (fsm == "RLFSMStateGetUp" && !this->behavior_settled.load())
        return "robot is getting up, wait until settled";
    if (fsm == "unknown")
        return "fsm not ready";

    std::string can_reason;
    if (!this->CheckCanFeedbackFresh(500, &can_reason))
        return can_reason;

    const bool standing_settled = (fsm == "RLFSMStateSafeHold") ||
        (fsm == "RLFSMStateGetUp" && this->behavior_settled.load());

    this->sg_.want_rl.store(want_rl);
    this->sg_.probe_busy.store(false);
    this->sg_.probe_fails.store(-1);
    this->sg_.kicked.store(false);
    this->sg_.active.store(true);
    // 重置所有步骤状态为 idle
    for (int i = 0; i < 5; ++i) this->sg_.step_states[i].store(0);

    this->Notify("safe_getup", want_rl ? "start rl" : "start");

    const double now = SgSteadyNowSec();
    if (standing_settled)
    {
        this->SafeGetupStep("lie", "skip");
        this->SafeGetupStep("probe", "skip");
        this->SafeGetupStep("getup", "skip");
        this->sg_.phase.store(static_cast<int>(SafeGetupPhase::kStandCheck));
    }
    else
    {
        this->sg_.phase.store(static_cast<int>(SafeGetupPhase::kLieCheck));
        this->sg_.deadline.store(now + 6.0);
        this->SafeGetupStep("lie", "run");
    }
    return "";
}

void RL_Real::SafeGetupEnterGap(SafeGetupPhase next, double now, double delay)
{
    this->sg_.kicked.store(false);
    this->sg_.gap_next.store(static_cast<int>(next));
    this->sg_.gap_until.store(now + delay);
    this->sg_.phase.store(static_cast<int>(SafeGetupPhase::kGap));
}

void RL_Real::LaunchSafeGetupProbe()
{
    this->sg_.probe_busy.store(true);
    this->sg_.probe_fails.store(-1);
    this->Notify("info", "开始电机可控性探针(约 2s),期间请勿操作...");
    this->Notify("probe", "start");
    PublishRosFeedback("safe_getup: probe motors executing");
    std::thread([this]() {
        static const char *names[16] = {
            "FL_hip","FL_thigh","FL_calf","FL_wheel",
            "FR_hip","FR_thigh","FR_calf","FR_wheel",
            "RL_hip","RL_thigh","RL_calf","RL_wheel",
            "RR_hip","RR_thigh","RR_calf","RR_wheel"
        };
        std::vector<bool> ok = motor_manager.probeAllMotorsCheck();
        int fail_count = 0;
        for (int i = 0; i < 16; ++i)
        {
            this->Notify("probe", "result m" + std::to_string(i) + " " + names[i]
                         + " " + (ok[i] ? "ok" : "fail"));
            if (!ok[i]) ++fail_count;
        }
        this->Notify("probe", "done " + std::to_string(fail_count));
        this->sg_.probe_fails.store(fail_count);
        this->sg_.probe_busy.store(false);
    }).detach();
}

void RL_Real::TickSafeGetup()
{
    if (!this->sg_.active.load()) return;

    using P = SafeGetupPhase;
    const auto ph = static_cast<P>(this->sg_.phase.load());
    const double now = SgSteadyNowSec();
    std::string fsm = this->fsm.GetCurrentStateName();
    if (fsm.empty()) fsm = "unknown";

    const auto eff = (ph == P::kGap)
        ? static_cast<P>(this->sg_.gap_next.load()) : ph;
    const bool getup_kicked = this->sg_.kicked.load();
    const bool down_or_passive = (fsm == "RLFSMStatePassive"
                                  || fsm == "RLFSMStateGetDown");
    if (((eff == P::kGetup && getup_kicked)
         || eff == P::kStandCheck || eff == P::kEnterRl)
        && down_or_passive)
    {
        const char* step = (eff == P::kEnterRl) ? "rl"
                         : (eff == P::kStandCheck) ? "stand" : "getup";
        this->FinishSafeGetup(false, step, "interrupted by getdown/stop");
        return;
    }
    if (eff == P::kEnterRl && fsm == "RLFSMStateSafeHold")
    {
        this->FinishSafeGetup(false, "rl", "interrupted by RL block");
        return;
    }

    switch (ph)
    {
    case P::kLieCheck:
    {
        if (now > this->sg_.deadline.load())
        {
            this->FinishSafeGetup(false, "lie", "timeout");
            return;
        }
        std::string can_reason;
        const bool can_ok = this->CheckCanFeedbackFresh(500, &can_reason);
        const bool motors_ok = this->CheckMotorInitPosition(1.0f);
        const bool ready = this->HandleCheckStand();
        if (!ready)
        {
            const bool imu_dead = (this->robot_state.imu.gyroscope[0] == 0.0f &&
                                   this->robot_state.imu.gyroscope[1] == 0.0f &&
                                   this->robot_state.imu.gyroscope[2] == 0.0f);
            const std::string reason = !can_ok
                ? can_reason
                : (!motors_ok ? this->motors_not_ready_reason
                              : (imu_dead ? std::string("IMU no data")
                                          : std::string("lie check failed")));
            this->FinishSafeGetup(false, "lie",
                reason.empty() ? "lie check failed" : reason);
            return;
        }
        this->SafeGetupStep("lie", "ok");
        this->SafeGetupEnterGap(P::kProbe, now);
        return;
    }

    case P::kGap:
    {
        if (now < this->sg_.gap_until.load()) return;
        this->sg_.phase.store(this->sg_.gap_next.load());
        return;
    }

    case P::kProbe:
    {
        if (!this->sg_.probe_busy.load() && this->sg_.probe_fails.load() < 0)
        {
            this->SafeGetupStep("probe", "run");
            this->sg_.deadline.store(now + 30.0);
            this->LaunchSafeGetupProbe();
            return;
        }
        if (now > this->sg_.deadline.load())
        {
            this->FinishSafeGetup(false, "probe", "timeout");
            return;
        }
        if (this->sg_.probe_busy.load()) return;
        const int fails = this->sg_.probe_fails.load();
        if (fails < 0)
        {
            this->FinishSafeGetup(false, "probe", "internal error");
            return;
        }
        if (fails > 0)
        {
            this->FinishSafeGetup(false, "probe",
                std::to_string(fails) + " motors not responding");
            return;
        }
        this->SafeGetupStep("probe", "ok");
        this->SafeGetupEnterGap(P::kGetup, now);
        return;
    }

    case P::kGetup:
    {
        if (!this->sg_.kicked.load())
        {
            this->sg_.kicked.store(true);
            this->manual_rl_block_.exchange(false);
            this->control.SetKeyboard(Input::Keyboard::Num0);
            PublishRosFeedback("safe_getup: getup accepted");
            this->SafeGetupStep("getup", "run");
            this->sg_.deadline.store(now + 15.0);
            return;
        }
        if (now > this->sg_.deadline.load())
        {
            this->FinishSafeGetup(false, "getup",
                "timeout, robot did not reach standing pose");
            return;
        }
        const bool stood = (fsm == "RLFSMStateSafeHold") ||
            (fsm == "RLFSMStateGetUp" && this->behavior_settled.load());
        if (!stood) return;
        this->SafeGetupStep("getup", "ok");
        this->SafeGetupEnterGap(P::kStandCheck, now);
        return;
    }

    case P::kStandCheck:
    {
        if (!this->sg_.kicked.load())
        {
            this->sg_.kicked.store(true);
            this->SafeGetupStep("stand", "run");
            this->sg_.deadline.store(now + 4.0);
        }
        if (now > this->sg_.deadline.load())
        {
            this->FinishSafeGetup(false, "stand", "timeout");
            return;
        }
        std::string reason;
        const bool at_pose = this->CheckStandingPose(&reason);
        this->HandleCheckStanding();
        if (!at_pose)
        {
            this->FinishSafeGetup(false, "stand",
                reason.empty() ? "not at expected standing pose" : reason);
            return;
        }
        this->SafeGetupStep("stand", "ok");
        if (!this->sg_.want_rl.load())
        {
            this->FinishSafeGetup(true, "", "");
            return;
        }
        this->SafeGetupEnterGap(P::kEnterRl, now);
        return;
    }

    case P::kEnterRl:
    {
        if (!this->sg_.kicked.load())
        {
            this->sg_.kicked.store(true);
            this->control.SetKeyboard(Input::Keyboard::Num1);
            PublishRosFeedback("safe_getup: locomotion accepted");
            this->SafeGetupStep("rl", "run");
            this->sg_.deadline.store(now + 20.0);
            return;
        }
        if (now > this->sg_.deadline.load())
        {
            this->FinishSafeGetup(false, "rl",
                "timeout, did not enter locomotion");
            return;
        }
        if (fsm == "RLFSMStateRLLocomotion" && this->rl_init_done)
        {
            this->SafeGetupStep("rl", "ok");
            this->FinishSafeGetup(true, "", "");
        }
        return;
    }

    case P::kIdle:
    default:
        return;
    }
}
