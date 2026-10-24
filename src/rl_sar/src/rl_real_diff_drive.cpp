#include "rl_real_JXG.hpp"

namespace {
double DiffNow() {
    return std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}
}

void RL_Real::InitDiffDrive() {
    jxg::DiffDriveConfig c;
    c.radius = params.Get<float>("diff_radius");
    c.track = params.Get<float>("diff_track");
    c.max_linear = params.Get<float>("diff_max_linear");
    c.max_angular = params.Get<float>("diff_max_angular");
    c.max_wheel = params.Get<float>("diff_max_wheel");
    c.wheel_acceleration = params.Get<float>("diff_wheel_acceleration");
    c.timeout = params.Get<float>("diff_timeout");
    const auto signs = params.Get<std::vector<float>>("diff_wheel_signs");
    if (signs.size() != 4) throw std::invalid_argument("diff_wheel_signs requires four values");
    std::copy(signs.begin(), signs.end(), c.signs.begin());
    diff_pose_ = params.Get<std::vector<float>>("default_dof_pos");
    diff_kp_ = params.Get<std::vector<float>>("fixed_kp");
    diff_kd_ = params.Get<std::vector<float>>("fixed_kd");
    diff_wheel_kd_ = params.Get<float>("diff_wheel_kd");
    diff_dt_ = params.Get<float>("dt");
    if (cfg_num_dofs != 16 || diff_pose_.size() != 16 || diff_kp_.size() != 16 ||
        diff_kd_.size() != 16 || !std::isfinite(diff_dt_) || diff_dt_ <= 0 || diff_dt_ > .05 ||
        !std::isfinite(diff_wheel_kd_) || diff_wheel_kd_ <= 0 || diff_wheel_kd_ > 2)
        throw std::invalid_argument("invalid differential drive gains/DOFs/dt");
    for (int i = 0; i < 16; ++i) {
        if (!std::isfinite(diff_pose_[i]) || !std::isfinite(diff_kp_[i]) ||
            !std::isfinite(diff_kd_[i]) || diff_kp_[i] < 0 || diff_kd_[i] < 0)
            throw std::invalid_argument("invalid differential drive standing pose");
    }
    diff_controller_ = std::make_unique<jxg::DiffDriveController>(c);
#if defined(USE_ROS2_REAL_CONTROL)
    diff_web_command_sub_ = create_subscription<std_msgs::msg::String>(
        "/rl_real/web_command", rclcpp::QoS(10).reliable(),
        [this](std_msgs::msg::String::SharedPtr msg) { ApplyRosCommand(msg->data); });
    diff_mission_sub_ = create_subscription<std_msgs::msg::String>(
        "/dog/mission_status", rclcpp::QoS(10).reliable(),
        [this](std_msgs::msg::String::SharedPtr msg) {
            try {
                const auto status = YAML::Load(msg->data);
                const auto phase = status["phase"].as<std::string>();
                std::lock_guard<std::mutex> lock(diff_mission_mutex_);
                diff_mission_phase_ = phase;
                diff_mission_received_ = DiffNow();
            } catch (...) { /* malformed status does not renew the lease */ }
        });
#endif
}

void RL_Real::ClearDiffVelocity() {
    std::lock_guard<std::mutex> lock(diff_velocity_mutex_);
    diff_velocity_ = {};
}

void RL_Real::ReceiveDiffVelocity(float vx, float vy, float wz) {
    std::lock_guard<std::mutex> lock(diff_velocity_mutex_);
    diff_velocity_ = {vx, vy, wz, DiffNow(),
        diff_active_.load() && std::isfinite(vx) && std::isfinite(vy) &&
        std::isfinite(wz) && vy == 0.0f};
}

bool RL_Real::DiffMissionIdle() {
    std::lock_guard<std::mutex> lock(diff_mission_mutex_);
    // Local manual control needs no brain. A fresh busy task still blocks entry.
    return DiffNow() - diff_mission_received_ >= 1.5 ||
        (diff_mission_phase_ == "idle" || diff_mission_phase_ == "done" ||
         diff_mission_phase_ == "aborted" || diff_mission_phase_ == "failed" ||
         diff_mission_phase_ == "rejected");
}

bool RL_Real::CanEnterDiffDrive() {
    return diff_controller_ && diff_enabled_.load() && !manual_rl_block_.load() &&
        !sg_.active.load() && DiffMissionIdle() &&
        fsm.GetCurrentStateName() == "RLFSMStateGetUp" && behavior_settled.load() &&
        CheckCanFeedbackFresh(100) && CheckStandingPose();
}

int RL_Real::DiffDriveRequest() {
    return diff_request_.exchange(0);
}

void RL_Real::EnterDiffDrive() {
    ClearDiffVelocity();
    diff_controller_->enter(DiffNow());
    diff_request_.store(0);
    diff_active_.store(true);
    behavior_settled.store(true);
    Notify("info", "四轮差速已进入，等待新的速度指令；仅适用于平整地面");
}

void RL_Real::RunDiffDrive(RobotCommand<float>* command) {
    jxg::DiffDriveCommand snapshot;
    {
        std::lock_guard<std::mutex> lock(diff_velocity_mutex_);
        snapshot = diff_velocity_;
    }
    const bool permitted = diff_enabled_.load() &&
        !manual_rl_block_.load() && diff_request_.load() != 2;
    if (!permitted) {
        ClearDiffVelocity();
        diff_request_.store(2);  // losing the task lease requires explicit re-entry
    }
    const auto wheels = diff_controller_->step(snapshot, DiffNow(), diff_dt_, permitted);
    for (int i = 0; i < 16; ++i) {
        command->motor_command.q[i] = diff_pose_[i];
        command->motor_command.dq[i] = 0;
        command->motor_command.kp[i] = diff_kp_[i];
        command->motor_command.kd[i] = diff_kd_[i];
        command->motor_command.tau[i] = 0;
    }
    for (int leg = 0; leg < 4; ++leg) {
        const int i = leg * 4 + 3;
        command->motor_command.q[i] = 0;
        command->motor_command.dq[i] = wheels[leg];
        command->motor_command.kp[i] = 0;
        command->motor_command.kd[i] = diff_wheel_kd_;
    }
}

void RL_Real::ExitDiffDrive() {
    diff_active_.store(false);
    ClearDiffVelocity();
    diff_controller_->exit();
}
