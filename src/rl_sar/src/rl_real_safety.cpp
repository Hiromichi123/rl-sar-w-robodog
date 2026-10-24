// C3-fix: 安全检查模块(从 rl_real_JXG.cpp 拆分)
// 包含: CheckMotorInitPosition / SafetyHoldActive / CheckCanFeedbackFresh / DetectStandingStartup
// 依赖: rl_real_JXG.hpp 中声明的成员变量(motors_not_ready_reason, params)与全局 motor_manager

#include "rl_real_JXG.hpp"

#include <algorithm>
#include <sstream>
#include <iomanip>

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

bool RL_Real::CheckMotorInitPosition(float threshold_rad)
{
    static const std::vector<int> wheel_indices = {3, 7, 11, 15};
    constexpr int64_t kFeedbackFreshMs = 200;  // 反馈超过 200ms 视为无效

    auto all_pos = motor_manager.getAllMotorPos();

    bool all_ok = true;
    std::string bad_motors;

    for (int i = 0; i < 16; ++i)
    {
        if (std::find(wheel_indices.begin(), wheel_indices.end(), i) != wheel_indices.end())
            continue;

        // 先检查反馈新鲜度:无反馈或反馈过旧的电机位置不可信,直接判失败
        int64_t age_ms = motor_manager.motorFeedbackAgeMs(i);
        if (age_ms < 0 || age_ms > kFeedbackFreshMs)
        {
            all_ok = false;
            char buf[64];
            if (age_ms < 0)
                snprintf(buf, sizeof(buf), " motor[%d]=NO_FEEDBACK", i);
            else
                snprintf(buf, sizeof(buf), " motor[%d]=STALE(%lldms)", i,
                         static_cast<long long>(age_ms));
            bad_motors += buf;
            continue;
        }

        float pos = all_pos[i];
        if (std::abs(pos) > threshold_rad)
        {
            all_ok = false;
            char buf[64];
            snprintf(buf, sizeof(buf), " motor[%d]=%.2frad", i, pos);
            bad_motors += buf;
        }
    }

    if (!all_ok)
    {
        this->motors_not_ready_reason = "Motor position check failed:" + bad_motors
            + " (threshold=" + std::to_string(threshold_rad) + "rad). "
            + "Please check CAN connection and re-zero motors.";
    }

    return all_ok;
}

bool RL_Real::SafetyHoldActive()
{
    constexpr int64_t kSafetyHoldFreshMs = 100;
    for (int i = 0; i < 16; ++i)
    {
        const int64_t age = motor_manager.motorFeedbackAgeMs(i);
        if (age < 0 || age > kSafetyHoldFreshMs)
            return true;
    }
    return false;
}

bool RL_Real::CheckCanFeedbackFresh(int timeout_ms, std::string* reason)
{
    constexpr int kNumMotors = 16;
    std::string bad;
    int n_bad = 0;
    for (int i = 0; i < kNumMotors; ++i)
    {
        const int64_t age = motor_manager.motorFeedbackAgeMs(i);
        if (age >= 0 && age <= timeout_ms) continue;
        ++n_bad;
        if (n_bad <= 8)
        {
            char buf[48];
            if (age < 0)
                std::snprintf(buf, sizeof(buf), " #%d(无反馈)", i);
            else
                std::snprintf(buf, sizeof(buf), " #%d(%lldms)", i, static_cast<long long>(age));
            bad += buf;
        }
    }
    if (n_bad == 0) return true;
    if (reason)
    {
        char head[96];
        std::snprintf(head, sizeof(head),
                      "CAN 反馈超时：%d/%d 个电机超过 %dms 无有效反馈",
                      n_bad, kNumMotors, timeout_ms);
        *reason = std::string(head) + bad + (n_bad > 8 ? " …" : "") +
                  "，请检查 USB-CAN 接线后重试";
    }
    return false;
}

bool RL_Real::DetectStandingStartup(std::vector<float>& raw_hold_q)
{
    constexpr float kStandTolRad = 0.35f;
    constexpr float kVelEps = 0.5f;
    constexpr int64_t kFeedbackFreshMs = 200;
    static const std::vector<int> wheel_indices = {3, 7, 11, 15};

    raw_hold_q.assign(16, 0.0f);

    Eigen::VectorXf pos = motor_manager.getAllMotorPos();
    Eigen::VectorXf vel = motor_manager.getAllMotorVel();
    if (pos.size() < 16 || vel.size() < 16)
        return false;

    // 先检查所有关节电机反馈新鲜度:无反馈时 pos 为默认零值,
    // 会被误判为"接近站立位默认角"。反馈不全则不判定站立。
    for (int i = 0; i < 16; ++i)
    {
        if (std::find(wheel_indices.begin(), wheel_indices.end(), i) != wheel_indices.end())
            continue;
        int64_t age = motor_manager.motorFeedbackAgeMs(i);
        if (age < 0 || age > kFeedbackFreshMs)
        {
            std::cout << "[INIT] DetectStandingStartup: motor[" << i
                      << "] no fresh feedback (age=" << age << "ms), skip standing detect"
                      << std::endl;
            return false;
        }
    }

    std::vector<float> dflt, off;
    try
    {
        dflt = this->params.Get<std::vector<float>>("default_dof_pos");
        off  = this->params.Get<std::vector<float>>("lying_down_offset");
    }
    catch (const std::exception& e)
    {
        std::cout << LOGGER::WARNING << "[INIT] DetectStandingStartup: read yaml failed: " << e.what() << std::endl;
        return false;
    }
    if (dflt.size() < 16) return false;
    if (off.size() < 16) off.assign(16, 0.0f);

    float worst_err = 0.0f;
    for (int i = 0; i < 16; ++i)
    {
        const float p = pos[i];
        const float v = vel[i];
        raw_hold_q[i] = p;
        if (!std::isfinite(p) || !std::isfinite(v))
            return false;
        if (std::find(wheel_indices.begin(), wheel_indices.end(), i) != wheel_indices.end())
            continue;
        const float q_state = p + off[i];
        const float err = std::abs(q_state - dflt[i]);
        worst_err = std::max(worst_err, err);
        if (err > kStandTolRad || std::abs(v) > kVelEps)
        {
            std::cout << "[INIT] Pre-enable posture NOT standing (joint[" << i
                      << "] q=" << q_state << " target=" << dflt[i]
                      << " err=" << err << " dq=" << v << ", worst_err=" << worst_err << ")" << std::endl;
            return false;
        }
    }

    std::cout << LOGGER::WARNING << "[INIT] Pre-enable posture MATCHES standing (worst joint err="
              << worst_err << " rad, tol=" << kStandTolRad << ")" << std::endl;
    return true;
}

// ============================================================================
// BMS 低电量监测
// ============================================================================

void RL_Real::BmsStateCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    // data=[soc%, voltage_V, current_A, alarm_bits, online(0/1)]
    if (msg->data.size() < 5) return;
    this->bms_soc_.store(static_cast<int>(msg->data[0]));
    this->bms_voltage_.store(msg->data[1]);
    this->bms_current_.store(msg->data[2]);
    this->bms_alarm_.store(static_cast<int>(msg->data[3]));
    this->bms_online_.store(msg->data[4] > 0.5f);
    this->bms_ts_.store(std::chrono::duration_cast<std::chrono::milliseconds>(
                            std::chrono::steady_clock::now().time_since_epoch()).count());
}

// 低电量检测与持续报警:由控制环每帧调用。
// 返回 true 表示当前处于低电量状态(已触发报警)。
bool RL_Real::CheckLowBattery()
{
    const int soc = this->bms_soc_.load();
    const float voltage = this->bms_voltage_.load();
    const bool online = this->bms_online_.load();

    // BMS 未上线或无数据:不报警(避免误报)
    if (!online || soc < 0) return false;

    const bool low_soc = (soc >= 0 && soc < this->low_battery_soc_threshold_);
    const bool low_voltage = (voltage > 0.0f && voltage < this->low_battery_voltage_threshold_);
    if (!low_soc && !low_voltage) return false;

    // 持续报警:每 low_battery_notify_interval_sec_ 秒发送一次
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                       now - this->last_low_bat_notify_time_).count();
    if (elapsed >= this->low_battery_notify_interval_sec_) {
        this->last_low_bat_notify_time_ = now;
        std::ostringstream oss;
        oss << "⚠️ 低电量报警: SOC=" << soc << "%, 电压=" << std::fixed << std::setprecision(1)
            << voltage << "V";
        if (low_soc) oss << " (SOC<" << this->low_battery_soc_threshold_ << "%)";
        if (low_voltage) oss << " (电压<" << this->low_battery_voltage_threshold_ << "V)";
        this->Notify("error", oss.str());
        std::cout << LOGGER::WARNING << "[BMS] " << oss.str() << std::endl;
    }
    return true;
}

// ============================================================================
// 电机温度提醒(仅告警,不干预控制)
// ============================================================================

// 电机温度提醒:由控制环每帧调用。
// 背景: 温度反馈有两类脏数据——间歇 0/-1(无效)与高电流机动时的单帧假尖峰
// (历史上 4 个髋同时瞬时 82-115°C 后又立即回落,热质量不可能这么快变化)。
// 因此每电机维护最近 5 个有效温度的滑动中值: 单帧尖峰会被中值滤除,
// 间歇 0/-1 不入窗(窗口保留旧有效值),有效样本不足时跳过判定。
// 中值超过阈值(默认 75°C)时按 30s 周期提醒一次。返回 true 表示告警持续中。
bool RL_Real::CheckMotorTempAlert()
{
    static RobotMotorManager::AllMotorFeedback fb = []{
        RobotMotorManager::AllMotorFeedback a;
        a.pos.resize(16); a.vel.resize(16); a.tau.resize(16); a.temp.resize(16);
        a.error.assign(16, 0.0f);
        return a;
    }();
    motor_manager.getAllMotorFeedback(fb);

    constexpr float kTempValidMin = 5.0f;    // <5°C 视为无效(未上电/坏点)
    constexpr float kTempValidMax = 150.0f;  // >150°C 视为坏点
    constexpr int   kMinValidSamples = 3;    // 窗口内至少 3 个有效样本才判定

    std::string hot;
    int n_hot = 0;
    for (int i = 0; i < 16; ++i)
    {
        const float t = fb.temp[i];
        if (t > kTempValidMin && t < kTempValidMax)
        {
            this->motor_temp_win_[i][this->motor_temp_win_idx_[i]] = t;
            this->motor_temp_win_idx_[i] = (this->motor_temp_win_idx_[i] + 1) % kMotorTempWin;
            if (this->motor_temp_win_cnt_[i] < kMotorTempWin) ++this->motor_temp_win_cnt_[i];
        }
        if (this->motor_temp_win_cnt_[i] < kMinValidSamples) continue;

        // 小窗口中值: 拷贝后排序取中位(≤5 个元素,~百 ns 级)
        float w[kMotorTempWin];
        const int n = this->motor_temp_win_cnt_[i];
        for (int j = 0; j < n; ++j) w[j] = this->motor_temp_win_[i][j];
        std::sort(w, w + n);
        const float med = w[n / 2];
        if (med > this->motor_temp_alert_threshold_)
        {
            ++n_hot;
            if (n_hot <= 8)
            {
                char buf[48];
                std::snprintf(buf, sizeof(buf), " m%d=%.0f°C", i, med);
                hot += buf;
            }
        }
    }
    if (n_hot == 0) return false;

    // 持续提醒:按 motor_temp_alert_interval_sec_ 周期一条(Notify 为异步队列推送,RT 安全)
    auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::seconds>(
            now - this->last_temp_notify_time_).count() >= this->motor_temp_alert_interval_sec_)
    {
        this->last_temp_notify_time_ = now;
        std::ostringstream oss;
        oss << "⚠️ 电机温度提醒(滑动中值>" << this->motor_temp_alert_threshold_ << "°C):"
            << hot << (n_hot > 8 ? " …" : "")
            << ",建议停机散热或降低负载/换向(仅提醒,不干预控制)";
        this->Notify("warning", oss.str());
        std::cout << LOGGER::WARNING << "[TEMP] " << oss.str() << std::endl;
    }
    return true;
}
