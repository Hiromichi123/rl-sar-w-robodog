// C3-fix: 姿态检测模块(从 rl_real_JXG.cpp 拆分)
// 包含: CheckStandCallback / BuildPostureJson / PublishPosture /
//       CheckStandingCallback / CheckStandingPose / HandleCheckStanding / HandleCheckStand
// 依赖: rl_real_JXG.hpp 中声明的成员变量(fsm, behavior_settled, rl_init_done,
//       lie_check_done_, lie_check_ok_, motors_ready_for_stand, can_feedback_live_,
//       robot_state, params, posture_publisher, web_bridge_, check_standing_result_publisher,
//       check_stand_result_publisher, motors_not_ready_reason)
//       以及成员函数 CheckStandingPose/CheckCanFeedbackFresh/CheckMotorInitPosition

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

static std::string PostureJsonEscape(const std::string &s)
{
    std::string out = s;
    for (size_t p = 0; (p = out.find('\\', p)) != std::string::npos; p += 2)
        out.replace(p, 1, "\\\\");
    for (size_t p = 0; (p = out.find('"', p)) != std::string::npos; p += 2)
        out.replace(p, 1, "\\\"");
    return out;
}

#if JXG_ROS_LAYER
void RL_Real::CheckStandCallback(const std_msgs::msg::String::SharedPtr msg)
{
    (void)msg;
    this->HandleCheckStand();
}
#endif

std::string RL_Real::BuildPostureJson()
{
    std::string fsm = this->fsm.GetCurrentStateName();
    if (fsm.empty()) fsm = "unknown";

    std::string posture = "unknown";
    bool settled = false;
    if (fsm == "RLFSMStatePassive")
    {
        posture = "lying";
        settled = true;
    }
    else if (fsm == "RLFSMStateSafeHold")
    {
        posture = "standing";
        settled = true;
    }
    else if (fsm == "RLFSMStateGetUp")
    {
        settled = this->behavior_settled.load();
        posture = settled ? "standing" : "transition";
    }
    else if (fsm == "RLFSMStateGetDown")
    {
        posture = "transition";
    }
    else if (fsm == "RLFSMStateDiffDrive")
    {
        settled = this->diff_active_.load();
        posture = settled ? "diff_drive" : "transition";
    }
    else if (fsm == "RLFSMStateRLLocomotion")
    {
        settled = this->rl_init_done;
        posture = settled ? "locomotion" : "transition";
    }

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3);
    std::string stand_reason;
    float stand_err = -1.0f;
    int stand_idx = -1;
    const bool stand_ready = this->CheckStandingPose(&stand_reason, &stand_err, &stand_idx);
    const bool lie_done = this->lie_check_done_.load();
    const bool lie_ready = lie_done && this->lie_check_ok_.load();
    const bool probe_done = motor_manager.hasProbeResults();
    const bool probe_ready = probe_done && motor_manager.areAllMotorsProbedOk();
    oss << "{\"posture\":\"" << posture << "\""
        << ",\"fsm\":\"" << fsm << "\""
        << ",\"settled\":" << (settled ? "true" : "false")
        << ",\"motors_ready\":" << (this->motors_ready_for_stand ? "true" : "false")
        << ",\"can_ready\":" << (this->can_feedback_live_.load() ? "true" : "false")
        << ",\"stand_ready\":" << (stand_ready ? "true" : "false")
        << ",\"stand_err\":" << stand_err
        << ",\"stand_joint\":" << stand_idx
        << ",\"stand_reason\":\"" << PostureJsonEscape(stand_reason) << "\""
        << ",\"lie_done\":" << (lie_done ? "true" : "false")
        << ",\"lie_ready\":" << (lie_ready ? "true" : "false")
        << ",\"probe_done\":" << (probe_done ? "true" : "false")
        << ",\"probe_ready\":" << (probe_ready ? "true" : "false")
        << ",\"diff_capable\":" << (this->diff_controller_ ? "true" : "false")
        << ",\"diff_enabled\":" << (this->diff_enabled_.load() ? "true" : "false")
        << ",\"diff_mission_idle\":" << (this->DiffMissionIdle() ? "true" : "false")
        << ",\"diff_nav\":" << (this->diff_nav_.load() ? "true" : "false")
        << ",\"gamepad_override\":false"
        << "}";
    return oss.str();
}

std::string RL_Real::PublishPosture()
{
    const std::string json = this->BuildPostureJson();
#if JXG_ROS_LAYER
    if (this->posture_publisher)
    {
        std_msgs::msg::String out;
        out.data = json;
        this->posture_publisher->publish(out);
    }
#endif
#if JXG_WEB_LAYER
    if (this->web_bridge_) this->web_bridge_->publishPosture(json);
#endif
    return json;
}

#if JXG_ROS_LAYER
void RL_Real::CheckStandingCallback(const std_msgs::msg::String::SharedPtr msg)
{
    (void)msg;
    this->HandleCheckStanding();
}
#endif

bool RL_Real::CheckStandingPose(std::string* reason, float* worst_err_out, int* worst_idx_out)
{
    constexpr float kStandTolRad = 0.30f;
    constexpr float kVelEps = 0.5f;
    static const std::vector<int> wheel_indices = {3, 7, 11, 15};

    bool imu_ok = !(this->robot_state.imu.gyroscope[0] == 0.0f &&
                    this->robot_state.imu.gyroscope[1] == 0.0f &&
                    this->robot_state.imu.gyroscope[2] == 0.0f);

    bool joints_ok = false;
    float worst_err = -1.0f;
    int worst_idx = -1;
    std::string reason_local;

    std::vector<float> dflt;
    try
    {
        dflt = this->params.Get<std::vector<float>>("default_dof_pos");
    }
    catch (const std::exception& e)
    {
        reason_local = std::string("read default_dof_pos failed: ") + e.what();
    }

    if (dflt.size() >= 16)
    {
        joints_ok = true;
        for (int i = 0; i < 16; ++i)
        {
            if (std::find(wheel_indices.begin(), wheel_indices.end(), i) != wheel_indices.end())
                continue;
            const float q  = this->robot_state.motor_state.q[i];
            const float dq = this->robot_state.motor_state.dq[i];
            if (!std::isfinite(q) || !std::isfinite(dq))
            {
                joints_ok = false;
                reason_local = "joint feedback invalid (NaN/Inf)";
                break;
            }
            const float err = std::abs(q - dflt[i]);
            if (err > worst_err) { worst_err = err; worst_idx = i; }
            if (err > kStandTolRad)
            {
                joints_ok = false;
            }
            else if (std::abs(dq) > kVelEps)
            {
                joints_ok = false;
            }
        }
        if (!joints_ok && reason_local.empty())
        {
            char buf[160];
            snprintf(buf, sizeof(buf),
                     "not at standing pose: joint[%d] err=%.3frad (tol=%.2f) or joints moving",
                     worst_idx, worst_err, kStandTolRad);
            reason_local = buf;
        }
    }

    std::string can_reason;
    constexpr int kCanFreshMs = 500;
    const bool can_ok = this->CheckCanFeedbackFresh(kCanFreshMs, &can_reason);

    const bool at_stand = joints_ok && imu_ok && can_ok;
    if (!can_ok) reason_local = can_reason;
    else if (!imu_ok && reason_local.empty()) reason_local = "IMU no data";

    if (reason) *reason = reason_local;
    if (worst_err_out) *worst_err_out = worst_err;
    if (worst_idx_out) *worst_idx_out = worst_idx;
    return at_stand;
}

bool RL_Real::HandleCheckStanding()
{
    std::string reason;
    float worst_err = -1.0f;
    int worst_idx = -1;
    const bool at_stand = this->CheckStandingPose(&reason, &worst_err, &worst_idx);

    constexpr float kStandTolRad = 0.30f;
    const bool imu_ok = !(this->robot_state.imu.gyroscope[0] == 0.0f &&
                          this->robot_state.imu.gyroscope[1] == 0.0f &&
                          this->robot_state.imu.gyroscope[2] == 0.0f);
    std::string can_reason;
    const bool can_ok = this->CheckCanFeedbackFresh(500, &can_reason);
    const bool joints_ok = at_stand;

    const std::string pjson = this->BuildPostureJson();
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3);
    oss << "{\"at_stand\":" << (at_stand ? "true" : "false")
        << ",\"joints_ready\":" << (joints_ok ? "true" : "false")
        << ",\"imu_ready\":" << (imu_ok ? "true" : "false")
        << ",\"can_ready\":" << (can_ok ? "true" : "false")
        << ",\"worst_err\":" << worst_err
        << ",\"worst_joint\":" << worst_idx
        << ",\"tol\":" << kStandTolRad
        << ",\"reason\":\"" << PostureJsonEscape(reason) << "\""
        << ",\"posture\":" << pjson
        << "}";
    const std::string out_json = oss.str();

#if JXG_ROS_LAYER
    if (this->check_standing_result_publisher)
    {
        std_msgs::msg::String out;
        out.data = out_json;
        this->check_standing_result_publisher->publish(out);
    }
#endif
#if JXG_WEB_LAYER
    if (this->web_bridge_) this->web_bridge_->publishCheckStandingResult(out_json);
#endif
    return at_stand;
}

bool RL_Real::HandleCheckStand()
{
    bool motors_ok = this->CheckMotorInitPosition(1.0f);
    bool imu_ok = !(this->robot_state.imu.gyroscope[0] == 0.0f &&
                    this->robot_state.imu.gyroscope[1] == 0.0f &&
                    this->robot_state.imu.gyroscope[2] == 0.0f);
    std::string can_reason;
    constexpr int kCanFreshMs = 500;
    const bool can_ok = this->CheckCanFeedbackFresh(kCanFreshMs, &can_reason);

    bool ready = motors_ok && imu_ok && can_ok;
    std::string reason;
    if (!can_ok)
        reason = can_reason;
    else if (!motors_ok)
        reason = this->motors_not_ready_reason;
    else if (!imu_ok)
        reason = "IMU no data";

    this->motors_ready_for_stand = (motors_ok && can_ok);
    if (this->motors_ready_for_stand)
        this->motors_not_ready_reason.clear();
    this->lie_check_done_.store(true);
    this->lie_check_ok_.store(motors_ok && can_ok);

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3);
    oss << "{\"ready\":" << (ready ? "true" : "false")
        << ",\"motors_ready\":" << (motors_ok ? "true" : "false")
        << ",\"imu_ready\":" << (imu_ok ? "true" : "false")
        << ",\"can_ready\":" << (can_ok ? "true" : "false")
        << ",\"reason\":\"" << PostureJsonEscape(reason) << "\"}";

#if JXG_ROS_LAYER
    if (this->check_stand_result_publisher)
    {
        std_msgs::msg::String out;
        out.data = oss.str();
        this->check_stand_result_publisher->publish(out);
    }
#endif
#if JXG_WEB_LAYER
    if (this->web_bridge_) this->web_bridge_->publishCheckStandResult(oss.str());
#endif
    return ready;
}
