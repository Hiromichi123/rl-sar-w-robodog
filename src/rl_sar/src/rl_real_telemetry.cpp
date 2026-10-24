// C3-fix: 遥测发布模块(从 rl_real_JXG.cpp 拆分)
// 包含: PublishMotorState / PublishImuState / PublishPose2D / PublishTelemetryBundle
// 依赖: rl_real_JXG.hpp 中声明的成员变量与函数

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

static double TelSteadyNowSec()
{
    return std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

void RL_Real::PublishMotorState(int num_dofs,
                                const std::vector<int> &joint_mapping,
                                const std::vector<float> &lying_down_offset,
                                const Eigen::VectorXf &all_pos,
                                const Eigen::VectorXf &all_vel,
                                const Eigen::VectorXf &all_tau,
                                const Eigen::VectorXf &all_temp)
{
#if JXG_ROS_LAYER && !JXG_WEB_LAYER
    if (!this->motor_state_publisher) return;
#endif

    constexpr int fields_per_motor = 11;
#if JXG_ROS_LAYER
    std_msgs::msg::Float32MultiArray msg;
    msg.layout.dim.resize(2);
    msg.layout.dim[0].label = "motor";
    msg.layout.dim[0].size = static_cast<uint32_t>(num_dofs);
    msg.layout.dim[0].stride = static_cast<uint32_t>(num_dofs * fields_per_motor);
    msg.layout.dim[1].label = "field:idx,target_pos,real_pos,target_vel,real_vel,target_tau,real_tau,kp,kd,temp_c,feedback_age_ms";
    msg.layout.dim[1].size = fields_per_motor;
    msg.layout.dim[1].stride = fields_per_motor;
    msg.layout.data_offset = 0;
#endif
    std::vector<float> data;
    data.reserve(static_cast<size_t>(num_dofs * fields_per_motor));

    for (int i = 0; i < num_dofs; ++i)
    {
        int idx = joint_mapping[i];
        const int64_t fb_age_ms = motor_manager.motorFeedbackAgeMs(idx);
        data.push_back(static_cast<float>(idx));
        data.push_back(this->robot_command.motor_command.q[i]);
        data.push_back(all_pos[idx] + lying_down_offset[i]);
        data.push_back(this->robot_command.motor_command.dq[i]);
        data.push_back(all_vel[idx]);
        data.push_back(this->robot_command.motor_command.tau[i]);
        data.push_back(all_tau[idx]);
        data.push_back(this->robot_command.motor_command.kp[i]);
        data.push_back(this->robot_command.motor_command.kd[i]);
        data.push_back(all_temp.size() > idx ? all_temp[idx] : -1.0f);
        data.push_back(static_cast<float>(fb_age_ms));
    }

    // L2 CAN 链路静默检测 + Notify 告警
    {
        static bool ever_rx[4]     = {false, false, false, false};
        static bool alarmed[4]     = {false, false, false, false};
        static int  recover_cnt[4] = {0, 0, 0, 0};
        constexpr int64_t kSilentMs    = 500;
        constexpr int     kRecoverTicks = 3;

        const int n_link = motor_manager.canLinkCount();
        bool new_alarm[4]   = {false, false, false, false};
        bool new_recover[4] = {false, false, false, false};

        for (int l = 0; l < n_link && l < 4; ++l) {
            const int64_t age = motor_manager.canLinkLastRxAgeMs(l);
            const bool healthy = (age >= 0 && age <= kSilentMs);
            if (age >= 0) ever_rx[l] = true;
            if (!alarmed[l]) {
                if (ever_rx[l] && !healthy) {
                    alarmed[l] = true; recover_cnt[l] = 0; new_alarm[l] = true;
                }
            } else {
                if (healthy) {
                    if (++recover_cnt[l] >= kRecoverTicks) {
                        alarmed[l] = false; recover_cnt[l] = 0; new_recover[l] = true;
                    }
                } else {
                    recover_cnt[l] = 0;
                }
            }
        }

        int alarmed_total = 0, new_alarm_total = 0;
        for (int l = 0; l < n_link && l < 4; ++l) {
            alarmed_total   += alarmed[l]   ? 1 : 0;
            new_alarm_total += new_alarm[l] ? 1 : 0;
        }
        if (new_alarm_total > 0) {
            if (n_link >= 4 && alarmed_total >= 4) {
                this->Notify("error",
                             "全部 CAN 链路断线，电机遥测已冻结；请检查 USB-CAN 连接，"
                             "恢复后需人工确认重新起立");
            } else {
                for (int l = 0; l < n_link && l < 4; ++l) {
                    if (new_alarm[l])
                        this->Notify("error",
                                     "CAN " + std::to_string(l + 1) + " 号链路断线");
                }
            }
        }
        for (int l = 0; l < n_link && l < 4; ++l) {
            if (new_recover[l])
                this->Notify("note",
                             "CAN " + std::to_string(l + 1) +
                             " 号链路已恢复；控制不自动恢复，请人工确认后重新起立");
        }
    }

    // L2(电机级) 单电机遥测超时通知
    {
        static bool m_ever[16]  = {false};
        static bool m_alarm[16] = {false};
        static int  m_rec[16]   = {0};
        constexpr int64_t kMotSilentMs   = 500;
        constexpr int     kMotRecTicks   = 3;

        std::vector<int> mot_timeout, mot_recover;
        const int n_link = motor_manager.canLinkCount();
        for (int i = 0; i < num_dofs && i < 16; ++i) {
            const int idx = joint_mapping[i];
            const int64_t age = motor_manager.motorFeedbackAgeMs(idx);
            const bool healthy = (age >= 0 && age <= kMotSilentMs);
            if (age >= 0) m_ever[i] = true;
            bool link_bad = false;
            const int link = motor_manager.motorCanIdx(idx);
            if (link >= 0 && link < n_link) {
                const int64_t lage = motor_manager.canLinkLastRxAgeMs(link);
                link_bad = !(lage >= 0 && lage <= kMotSilentMs);
            }
            if (!m_alarm[i]) {
                if (m_ever[i] && !healthy && !link_bad) {
                    m_alarm[i] = true; m_rec[i] = 0; mot_timeout.push_back(i);
                }
            } else {
                if (healthy && !link_bad) {
                    if (++m_rec[i] >= kMotRecTicks) {
                        m_alarm[i] = false; m_rec[i] = 0; mot_recover.push_back(i);
                    }
                } else {
                    m_rec[i] = 0;
                }
            }
        }
        auto join_ids = [](const std::vector<int> &v) {
            std::string s;
            for (size_t k = 0; k < v.size(); ++k) {
                if (k) s += "、";
                s += "#" + std::to_string(v[k]);
            }
            return s;
        };
        if (!mot_timeout.empty()) {
            this->Notify("warning",
                         (mot_timeout.size() == 1 ? "电机 " : "多个电机 ") + join_ids(mot_timeout) +
                         (mot_timeout.size() == 1 ? " 遥测超时：" : " 遥测超时：") +
                         std::to_string(kMotSilentMs) + "ms 未收到有效反馈，请检查该电机/接线");
        }
        if (!mot_recover.empty()) {
            this->Notify("note", "电机 " + join_ids(mot_recover) + " 遥测已恢复");
        }
    }

#if JXG_ROS_LAYER
    if (this->motor_state_publisher)
    {
        msg.data = data;
        this->motor_state_publisher->publish(msg);
    }
#endif
#if JXG_WEB_LAYER
    if (this->web_bridge_) this->web_bridge_->publishMotorState(data);
#endif
}

void RL_Real::PublishImuState()
{
#if JXG_ROS_LAYER && !JXG_WEB_LAYER
    if (!this->imu_state_publisher) return;
#endif

    // 跨线程保护:控制环 GetState 持 jxg_state_mutex_ 写 JXG_state,
    // 遥测环此处需持同一锁读取,保证拿到完整一致的一帧 IMU 快照。
    JXGIMUState imu_snapshot;
    {
        std::lock_guard<std::mutex> lk(this->jxg_state_mutex_);
        imu_snapshot = this->JXG_state.imu_state;
    }
    const auto &imu = imu_snapshot;
    const float qw = static_cast<float>(imu.quaternion[0]);
    const float qx = static_cast<float>(imu.quaternion[1]);
    const float qy = static_cast<float>(imu.quaternion[2]);
    const float qz = static_cast<float>(imu.quaternion[3]);

    const float roll = static_cast<float>(std::atan2(2.0f * (qw * qx + qy * qz),
                                                     1.0f - 2.0f * (qx * qx + qy * qy)));
    float pitch_arg = 2.0f * (qw * qy - qz * qx);
    pitch_arg = std::max(-1.0f, std::min(1.0f, pitch_arg));
    const float pitch = static_cast<float>(std::asin(pitch_arg));
    const float yaw = static_cast<float>(std::atan2(2.0f * (qw * qz + qx * qy),
                                                   1.0f - 2.0f * (qy * qy + qz * qz)));

    constexpr int field_count = 19;
    std::vector<float> data = {
        qw, qx, qy, qz,
        roll, pitch, yaw,
        static_cast<float>(imu.accelerometer[0]),
        static_cast<float>(imu.accelerometer[1]),
        static_cast<float>(imu.accelerometer[2]),
        static_cast<float>(imu.gyroscope[0]),
        static_cast<float>(imu.gyroscope[1]),
        static_cast<float>(imu.gyroscope[2]),
        static_cast<float>(imu.position_north),
        static_cast<float>(imu.position_east),
        static_cast<float>(imu.position_down),
        static_cast<float>(imu.velocity_body[0]),
        static_cast<float>(imu.velocity_body[1]),
        static_cast<float>(imu.velocity_body[2])
    };

#if JXG_ROS_LAYER
    if (this->imu_state_publisher)
    {
        std_msgs::msg::Float32MultiArray msg;
        msg.layout.dim.resize(1);
        msg.layout.dim[0].label = "field:quat_w,quat_x,quat_y,quat_z,roll,pitch,yaw,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z,pos_north,pos_east,pos_down,vel_body_x,vel_body_y,vel_body_z";
        msg.layout.dim[0].size = field_count;
        msg.layout.dim[0].stride = field_count;
        msg.layout.data_offset = 0;
        msg.data = data;
        this->imu_state_publisher->publish(msg);
    }
#endif
#if JXG_WEB_LAYER
    if (this->web_bridge_) this->web_bridge_->publishImuState(data);
#endif
}

void RL_Real::PublishPose2D()
{
#if JXG_ROS_LAYER && !JXG_WEB_LAYER
    if (!this->pose2d_publisher) return;
#endif

    constexpr int field_count = 7;
    std::vector<float> data = {
        static_cast<float>(this->odometry_x),
        static_cast<float>(this->odometry_y),
        static_cast<float>(this->odometry_z),
        static_cast<float>(this->odometry_yaw),
        static_cast<float>(this->velocity_x),
        static_cast<float>(this->velocity_y),
        static_cast<float>(this->control.yaw)
    };

#if JXG_ROS_LAYER
    if (this->pose2d_publisher)
    {
        std_msgs::msg::Float32MultiArray msg;
        msg.layout.dim.resize(1);
        msg.layout.dim[0].label = "field:x,y,z,yaw,vel_x,vel_y,yaw_rate";
        msg.layout.dim[0].size = field_count;
        msg.layout.dim[0].stride = field_count;
        msg.layout.data_offset = 0;
        msg.data = data;
        this->pose2d_publisher->publish(msg);
    }
#endif
#if JXG_WEB_LAYER
    if (this->web_bridge_) this->web_bridge_->publishPose2D(data);
#endif
}

void RL_Real::PublishTelemetryBundle()
{
    int num_dofs = this->params.Get<int>("num_of_dofs");
    auto jm = this->params.Get<std::vector<int>>("joint_mapping");
    auto offset = this->params.Get<std::vector<float>>("lying_down_offset");

    auto all_pos = motor_manager.getAllMotorPos();
    auto all_vel = motor_manager.getAllMotorVel();
    auto all_tau = motor_manager.getAllMotorTorque();
    auto all_temp = motor_manager.getAllMotorTemp();

    this->PublishMotorState(num_dofs, jm, offset, all_pos, all_vel, all_tau, all_temp);
    this->PublishImuState();
    this->PublishPose2D();
    this->PublishPosture();
    this->MaybePublishHeartbeat();
    // C2-fix: 批量发送异步通知队列(非实时线程,与遥测同节奏 8Hz)
    this->FlushNotifyQueue();
#if defined(USE_ROS2_REAL_CONTROL)
    this->ProcessNotifyRetries(this->now().seconds());
#else
    this->ProcessNotifyRetries(TelSteadyNowSec());
#endif
}
