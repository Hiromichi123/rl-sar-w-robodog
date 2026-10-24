#pragma once

#include "rl_sdk.hpp"
#include "diff_drive_controller.hpp"
#include "observation_buffer.hpp"
#include "loop.hpp"
#include "fsm.hpp"
#include "fsm_JXGw.hpp"
#include "JXG_low_cmd.h"
#include "JXG_low_state.h"
#include <memory>
#include <string>
#include <vector>
#include <fstream>
#include <atomic>
#include <mutex>
#include <map>
#include <cstdint>
#include <chrono>
#include <thread>
#include "matplotlibcpp.h"
#include "web_bridge.hpp"

// Web 调试台默认静态文件根目录（由 CMake 注入，可用 --web-root 覆盖）
#ifndef WEB_ROOT
#define WEB_ROOT "./web"
#endif

// 主程序 Web/音频相关配置（main() 从命令行解析后传入构造函数）
struct WebConfig
{
    int web_port = 8088;
    std::string web_root = WEB_ROOT;
};

// 异步CSV数据记录器开关（取消注释以启用）
// #define ENABLE_ASYNC_CSV_LOGGER

#ifdef ENABLE_ASYNC_CSV_LOGGER
#include "async_csv_logger.hpp"
#endif

// ===== 高频(200Hz)真机数据记录器 =====
// 用途: 肢体/轮速抖动与频谱分析(需 >100Hz 采样才能看到 8Hz 量级振荡)
// 位置: <cwd>/logs/<robot>_hr_<时间戳>.csv  (cwd = cmake_build/bin)
// 列: t,gyro(3),acc(3),quat(4),cmd(3), 每电机(tgt_pos,real_pos,tgt_vel,real_vel,real_tau,temp)
// 采集点: RobotControl() (loop_control, 200Hz)
// 关闭方式: 注释掉下面这行后重新编译
#define ENABLE_HR_LOGGER

namespace plt = matplotlibcpp;

#if defined(USE_ROS1) && defined(USE_ROS)
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#elif defined(USE_ROS2_REAL_CONTROL)
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#elif defined(USE_ROS2) && defined(USE_ROS)
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#endif


class RL_Real : public RL
#if defined(USE_ROS2_REAL_CONTROL) || (defined(USE_ROS2) && defined(USE_ROS))
    , public rclcpp::Node
#endif
{
public:
    explicit RL_Real(const WebConfig &web_cfg = WebConfig());
    ~RL_Real();

private:
    bool CanEnterDiffDrive() override;
    void EnterDiffDrive() override;
    void RunDiffDrive(RobotCommand<float>* command) override;
    void ExitDiffDrive() override;
    int DiffDriveRequest() override;
    void InitDiffDrive();
    void ClearDiffVelocity();
    void ReceiveDiffVelocity(float vx, float vy, float wz);
    std::atomic<bool> diff_enabled_{false}, diff_active_{false};
    std::atomic<int> diff_request_{0};
    std::mutex diff_velocity_mutex_;
    jxg::DiffDriveCommand diff_velocity_;
    std::unique_ptr<jxg::DiffDriveController> diff_controller_;
    std::vector<float> diff_pose_, diff_kp_, diff_kd_;
    float diff_wheel_kd_ = 0.5f;
    double diff_dt_ = 0.005;
    std::atomic<bool> diff_nav_{false};
#if defined(USE_ROS2_REAL_CONTROL)
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr diff_mission_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr diff_web_command_sub_;
#endif
    std::mutex diff_mission_mutex_;
    std::string diff_mission_phase_;
    double diff_mission_received_ = 0;
    bool DiffMissionIdle();

    std::vector<float> Forward() override;
    void GetState(RobotState<float> *state) override;
    void SetCommand(const RobotCommand<float> *command) override;
    void RunModel();
    void RobotControl();

    // 循环
    std::shared_ptr<LoopFunc> loop_keyboard;
    std::shared_ptr<LoopFunc> loop_control;
    std::shared_ptr<LoopFunc> loop_rl;
    std::shared_ptr<LoopFunc> loop_imu;          // 1kHz IMU读取线程
    std::shared_ptr<LoopFunc> loop_plot;


    
    JXG_low_cmd    JXG_cmd;
    JXGLowState    JXG_state{16};
    // 保护 JXG_state:控制环(200Hz)写、遥测环(~8Hz)读,跨线程需互斥。
    // 控制环 GetState 内 updateJXGLowState 写入期间,遥测 PublishImuState 不得并发读。
    std::mutex     jxg_state_mutex_;

    // CAN 反馈实时健康状态:控制环 ~20Hz 刷新(16 电机反馈均在 500ms 内才 true)。
    // 姿态帧广播 + 动作指令入口实时拦截共用,跨线程故用 atomic。
    std::atomic<bool> can_feedback_live_{false};

    // 手动强化学习阻断:触发后控制环当帧切 SafeHold 并阻断模型输出(不依赖 CAN 状态)。
    // 与 CAN 保护走同一套当帧 SafeHold 逻辑;区别在于 CAN 胶囊只反映真实 CAN 状态,
    // 手动阻断不应让前端误报 CAN 故障。需人工 getup/getdown 解除。
    std::atomic<bool> manual_rl_block_{false};

    // ---- BMS 低电量监测 ----
    // web_server 1Hz 发布 /rl_real/bms_state,回调写、控制环读,跨线程用 atomic。
    // soc=-1 表示尚未收到任何 BMS 数据。
    std::atomic<int>      bms_soc_{-1};
    std::atomic<float>    bms_voltage_{0.0f};
    std::atomic<float>    bms_current_{0.0f};
    std::atomic<int>      bms_alarm_{0};
    std::atomic<bool>     bms_online_{false};
    std::atomic<long long> bms_ts_{0};  // 数据时间戳 ms
    // 低电量阈值(可配置)
    float low_battery_soc_threshold_ = 15.0f;     // SOC < 15% 触发
    float low_battery_voltage_threshold_ = 20.0f; // 电压 < 20V 触发(16S 锂电约 3.2V/cell)
    // 低电量报警续报周期(秒,可配置):低于阈值后每隔该周期推送一次报警通知
    int low_battery_notify_interval_sec_ = 10;

    // ===== 电机温度提醒(仅告警,不干预控制) =====
    static constexpr int kMotorTempWin = 5;              // 滑动中值窗口长度
    float motor_temp_win_[16][kMotorTempWin] = {};       // 各电机最近 N 个有效温度(环形)
    int   motor_temp_win_cnt_[16] = {0};                 // 各电机窗口内有效样本数
    int   motor_temp_win_idx_[16] = {0};                 // 各电机环形写入位置
    float motor_temp_alert_threshold_ = 75.0f;           // 中值告警阈值(°C),可经 temp_threshold 命令在线调整
    int   motor_temp_alert_interval_sec_ = 30;           // 温度告警续报周期(秒),可经 temp_threshold interval 在线调整
    std::chrono::steady_clock::time_point last_temp_notify_time_{};
    // 上次低电量报警时间戳(控制续报间隔)
    std::chrono::steady_clock::time_point last_low_bat_notify_time_{};

    void InitLowCmd();
    bool CheckMotorInitPosition(float threshold_rad) override;
    // 起立前 CAN 反馈新鲜度校验:16 个电机(含轮毂)最近一次"有效"反馈均在 timeout_ms 内
    // 才通过;失败时 *reason 填入超时电机明细。防止 CAN 断线时关节角冻结、近零位/站姿
    // 校验被冻结旧值误通过而异常起立。只读时间戳,不触碰控制环反馈。
    bool CheckCanFeedbackFresh(int timeout_ms, std::string* reason = nullptr);
    // 200Hz 控制环实时安全保持:任一电机 CAN 反馈超时(>100ms)或从未收到即 true。
    // RL 态触发时当帧退出 RLLocomotion→SafeHold(无一帧模型输出);
    // 非 RL 态触发时直接发当前位 fixed PD 保持帧并跳过 StateController。
    bool SafetyHoldActive();
    // BMS 低电量检测与持续报警:控制环每帧调用,SOC/电压低于阈值时每 10s 续报一次。
    // 返回 true 表示当前处于低电量状态。
    bool CheckLowBattery();
    // 电机温度提醒(仅告警,不干预控制):控制环每帧调用,每电机滑动中值(窗口5)
    // 超过阈值时每 30s 提醒一次。返回 true 表示有温度告警在持续。
    bool CheckMotorTempAlert();
    // 崩溃重启安全:使能前依据关节反馈判定机器人是否处于站立构型。
    // 命中时 raw_hold_q 填入 16 个当前关节角(电机原始坐标),供使能后 PD 保持。
    bool DetectStandingStartup(std::vector<float>& raw_hold_q);
    // 本次启动是否检测到站立姿态(决定 FSM 初始进 SafeHold 还是 Passive)
    bool startup_standing_ = false;

    // 电机反馈异常连续计数器(16 个电机各一个初值 0)
    // 注意:C++17 类内初始化不能用圆括号 (16,0)(会被当成函数声明),也不能用 {16,0}
    // (initializer_list 解析成仅 2 个元素 [16,0],进 RL 态后下标 i>=2 越界 → SIGABRT)。
    std::vector<int> motor_dead_count = std::vector<int>(16, 0);

    // 200Hz 实时环静态参数缓存:ReadYaml 后只解析一次,避免在 GetState/SetCommand
    // 里以 200Hz 反复做 YAML 字符串键查找 + std::vector 按值返回(每次堆分配)。
    int cfg_num_dofs = 0;                   // 缓存 num_of_dofs
    std::vector<int>   cfg_joint_mapping;   // 缓存 joint_mapping
    std::vector<float> cfg_lying_offset;    // 缓存 lying_down_offset
    // SetCommand 复用缓冲区(避免每周期 5 次 vector<float>(16) 堆分配)
    std::vector<float> cmd_tau_buf, cmd_vel_buf, cmd_pos_buf, cmd_kp_buf, cmd_kd_buf;


    const int plot_size = 500;
    std::vector<int> plot_t;
    std::vector<std::vector<double>> plot_real_joint_pos, plot_target_joint_pos;
    void Plot();


    // 里程计积分（从 IMU 加速度估算位置）
    double odometry_x = 0.0;
    double odometry_y = 0.0;
    double odometry_z = 0.28;  // 初始高度（默认趴下姿态），可通过 --height 参数修改
    double odometry_yaw = 0.0;
    double velocity_x = 0.0;
    double velocity_y = 0.0;
    double velocity_z = 0.0;
    double last_imu_time = 0.0;

#ifdef ENABLE_ASYNC_CSV_LOGGER
    // 异步CSV数据记录器
    std::unique_ptr<AsyncCSVLogger> csv_logger;
    std::shared_ptr<LoopFunc> loop_csv_logger;  // 低频采集线程
    double csv_logger_start_time = 0.0;         // 记录开始时间
#endif

#ifdef ENABLE_HR_LOGGER
    // 高频(200Hz)记录器(运行时可经 Web/命令 "log on|off|status" 开关)
    void HrLogInit();
    void HrLogPush();
    void HrLogClose();
    std::string HrLogReadModelNameFromDisk();  // 磁盘 config.yaml 的 model_name(启动预读/运行中轮询共用)
    void HrLogReopenFile();             // 按磁盘当前模型名重开文件并写表头(写盘线程/启动时调用,RT 线程不碰)
    void HrLogModelWatch();             // 看护线程体:每秒扫磁盘 model_name(替代 RT 线程内轮询)
    void HrLogWriteLoop();              // 写盘线程体:独占 hr_file,RT 线程零文件 I/O
    std::ofstream hr_file;
    std::string hr_cur_path;            // 当前 HR 文件路径(模型名就绪后重建时删除旧文件用)
    bool hr_named_by_model = false;     // 当前文件名是否含模型名(回退文件在重建时可删,模型文件保留)
    std::atomic<bool> hr_log_enabled_{true};
    std::atomic<bool> hr_rebuild_requested_{false};  // 看护线程检测到模型切换,写盘线程消费重建
    std::thread hr_model_watch_thread_;  // 非实时看护线程:每秒扫磁盘 model_name(不进 RT 线程)
    std::atomic<bool> hr_model_watch_stop_{false};
    // ===== 文件 I/O 与格式化分离:RT 线程只把行快照写入无锁 SPSC 环 =====
    // 写盘/flush/文件重建全部由写盘线程完成;环满则丢行(计数),绝不阻塞实时环。
    std::thread hr_writer_thread_;       // 写盘线程(HrLogInit 启动一次, HrLogClose 停止)
    std::atomic<bool> hr_writer_stop_{false};
    std::atomic<bool> hr_writer_running_{false};
    std::atomic<bool> hr_rebuilding_{false};  // 写盘线程重建文件期间置位(看护线程据此避开 hr_cur_path)
    std::atomic<uint64_t> hr_ring_head_{0};   // SPSC 环头(RT 生产者推进)
    std::atomic<uint64_t> hr_ring_tail_{0};   // SPSC 环尾(写盘消费者推进)
    std::atomic<long> hr_ring_dropped_{0};    // 环满丢弃行数
    static constexpr int kHrRingSlots = 1024;   // 必须为 2 的幂
    static constexpr size_t kHrRowMax = 2304;   // 单行最坏宽度上限(143 字段)
    double hr_t0 = 0.0;                 // 记录起点(steady_clock)
    long hr_rows = 0;                   // 已写行数(用于定期 flush)
    int hr_num_dofs = 0;                // 缓存的 num_of_dofs
    std::vector<int> hr_jm;             // 缓存的 joint_mapping
    std::vector<float> hr_offset;       // 缓存的 lying_down_offset
    char hr_ring_rows_[kHrRingSlots][kHrRowMax];  // SPSC 环数据区(约2.3MB,对象在堆上)
    uint16_t hr_ring_len_[kHrRingSlots];
#endif

#if defined(USE_ROS1) && defined(USE_ROS)
    geometry_msgs::Twist cmd_vel;
    ros::Subscriber cmd_vel_subscriber;
    void CmdvelCallback(const geometry_msgs::Twist::ConstPtr &msg);
#elif defined(USE_ROS2_REAL_CONTROL)
    geometry_msgs::msg::Twist cmd_vel;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber;
    // 网页独占速度通道:web_server 把浏览器 WS cmd_vel 转到此话题(与 ROS 大脑的
    // /rl_real/cmd_vel 区分),用于实现 Web 对 ROS 的事件触发式速度独占。
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr web_cmd_vel_subscriber;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_subscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr imu_stream_subscriber;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr feedback_publisher;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr motor_state_publisher;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr imu_state_publisher;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pose2d_publisher;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr notify_publisher;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr notify_ack_subscriber;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr check_stand_subscriber;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr check_stand_result_publisher;
    // 姿态接口(/rl_real/posture: lying/standing/locomotion/transition)与站立期望值校验
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr posture_publisher;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr check_standing_subscriber;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr check_standing_result_publisher;
    // 命令生命周期与心跳(大脑下游 HAL/桥硬依赖; 合成层: 由 8Hz posture/settled 边沿合成)
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cmd_state_publisher;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr heartbeat_publisher;
    // safe_getup 一键安全起立编排:20Hz 推进(与命令回调同处 executor 主线程,串行无锁)
    rclcpp::TimerBase::SharedPtr safe_getup_timer_;
    void CmdvelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    // /rl_real/web_cmd_vel:网页独占速度源;非零速度触发/续期独占,压制 ROS cmd_vel。
    void WebCmdVelRosCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void CommandCallback(const std_msgs::msg::String::SharedPtr msg);
    void ImuStreamCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void NotifyAckCallback(const std_msgs::msg::String::SharedPtr msg);
    void CheckStandCallback(const std_msgs::msg::String::SharedPtr msg);
    void CheckStandingCallback(const std_msgs::msg::String::SharedPtr msg);
    // BMS 电池状态订阅(低电量判断)
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr bms_state_subscriber;
    void BmsStateCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
#elif defined(USE_ROS2) && defined(USE_ROS)
    geometry_msgs::msg::Twist cmd_vel;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber;
    void CmdvelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
#endif

    // ===== 两种模式共用的收发接口（ROS 模式发话题,USE_CMAKE 裸机模式发 WebBridge） =====
    // 无论 seq>0(ws_demo) 还是 seq==0(ROS /rl_real/command、Web 页面), 每条命令都合成
    // cmd_state 生命周期: 即时命令立即 DONE; 行为命令由 8Hz posture/settled 边沿合成。
    void ApplyRosCommand(const std::string &command, int seq = 0);
    void PublishRosFeedback(const std::string &message);

    // ---- 命令生命周期(cmd_state) + 2Hz 心跳(heartbeat) 合成层 ----
    // code: 0=RECEIVED 1=EXECUTING 2=DONE 3=FAILED 4=REJECTED
    enum CmdCode { kReceived = 0, kExecuting = 1, kDone = 2, kFailed = 3, kRejected = 4 };
    // 构造并双发(ROS /rl_real/cmd_state 与 WS cmd_state)同一 JSON 帧
    void EmitCommandState(int seq, const std::string &cmd, int code,
                          const std::string &target, double elapsed_ms);
    // 控制环周期调用: 每 0.5s 双发心跳(ROS /rl_real/heartbeat 与 WS heartbeat)
    void MaybePublishHeartbeat();
    struct PendingCommand {
        int seq = 0;
        std::string cmd;           // 规范命令名(getup/getdown/locomotion/passive)
        std::string target_state;  // 完成时应处的 FSM 态
        double start_sec = 0.0;
        double timeout_sec = 6.0;
        double min_sec = 0.0;      // 最小执行时长: 防止发出时已在 target 态导致假 DONE
        bool executing_sent = false;
    };
    void TrackFsmCommand(int seq, const std::string &cmd,
                         const std::string &target_state, double timeout_sec,
                         double min_sec = 0.0);
    void UpdatePendingCommand();  // 控制环每周期调用: 发 EXECUTING/DONE/FAILED
    std::mutex pending_cmd_mutex_;
    std::unique_ptr<PendingCommand> pending_cmd_;

    // ---- 一键安全起立编排(safe_getup[ rl]) ----
    // 后端串行执行:卧姿校验 → 16台探针 → 起立 → 站姿确认 →(可选)进入强化学习,
    // 任一环节失败立即中止。任何 ROS 客户端只需发一条 /rl_real/command 字符串,
    // 进度通过 notify(type="safe_getup")推送:
    //   "start[ rl]" / "step <lie|probe|getup|stand|rl> <run|ok|fail|skip>[ N]"
    //   / "done ok" / "done fail <step> <reason>"
    enum class SafeGetupPhase {
        kIdle = 0, kLieCheck, kGap, kProbe, kGetup, kStandCheck, kEnterRl
    };
    struct SafeGetupState {
        std::atomic<bool> active{false};
        std::atomic<bool> want_rl{false};
        std::atomic<int>  phase{static_cast<int>(SafeGetupPhase::kIdle)};
        std::atomic<double> deadline{0.0};  // 阶段超时(SteadyNowSec)
        std::atomic<bool> probe_busy{false};
        std::atomic<int>  probe_fails{-1};  // -1=尚未跑完
        // 步骤间固定停顿(kGap):到 gap_until 后切到 gap_next,不阻塞 executor
        std::atomic<double> gap_until{0.0};
        std::atomic<int>    gap_next{static_cast<int>(SafeGetupPhase::kIdle)};
        // 动作类阶段(getup/enterRl/standCheck)是否已发出首拍指令
        std::atomic<bool> kicked{false};
        // 各步骤最新状态(供页面刷新后回显):0=idle,1=run,2=ok,3=fail,4=skip
        // 索引:lie=0, probe=1, getup=2, stand=3, rl=4
        std::atomic<int> step_states[5]{{0},{0},{0},{0},{0}};
    };
    SafeGetupState sg_;
    static constexpr double kSafeGetupGapSec = 1.0;  // 每个流程操作间隔 1000ms
    // 命令入口:返回空串=已受理;非空=拒绝原因(调用方据此回 rejected)
    std::string StartSafeGetup(bool want_rl);
    void TickSafeGetup();                // 20Hz wall_timer,推进状态机
    // 进入步骤间停顿:kStepDelay 后由 kGap 切到 next(期间不发任何动作指令)
    void SafeGetupEnterGap(SafeGetupPhase next, double now,
                           double delay = kSafeGetupGapSec);
    void LaunchSafeGetupProbe();         // 异步 16 台探针(与手动探针同源通知)
    void SafeGetupStep(const std::string& step, const std::string& state,
                       const std::string& extra = "");
    void SafeGetupStatus();                 // 推送全量状态(供页面刷新后回显)
    void FinishSafeGetup(bool ok, const std::string& step, const std::string& reason);
    int last_cmd_seq_ = -1;      // 最近已受理的命令 seq
    double last_cmd_time_ = -100.0;  // 该 seq 的受理时刻(SteadyNowSec), 配合 seq 做时间窗去重
    // 2Hz 心跳状态(单调时钟): 序号/首次节拍时刻/上次节拍时刻
    uint32_t heartbeat_seq_ = 0;
    double heartbeat_start_sec_ = 0.0;
    double last_heartbeat_sec_ = -100.0;

    void PublishMotorState(int num_dofs,
                           const std::vector<int> &joint_mapping,
                           const std::vector<float> &lying_down_offset,
                           const Eigen::VectorXf &all_pos,
                           const Eigen::VectorXf &all_vel,
                           const Eigen::VectorXf &all_tau,
                           const Eigen::VectorXf &all_temp);
    void PublishImuState();
    void PublishPose2D();
    // 完整遥测束:motor_state(含每台 feedback_age_ms)+ imu + pose2d + posture +
    // heartbeat + Notify 重试。正常 8Hz 遥测段与 SafetyHold 拦截分支共用——
    // 单路 CAN 断线时其余 3 路电机与 IMU 数据必须继续流动,不能整页冻结。
    void PublishTelemetryBundle();
    void Notify(const std::string &type, const std::string &content) override;
    void PublishNotify(const std::string &type, const std::string &content);
    void ProcessNotifyRetries(double now_sec);
    std::chrono::steady_clock::time_point last_cmd_vel_time_;  // 最后收到有效速度的时间(断联超时归零)
    // ── Web 事件触发式速度独占(standalone 经 /rl_real/web_cmd_vel,内嵌经 WebBridge) ──
    // 最近一次「非零」Web 速度时间 = 独占锁看门狗。非零 web 速度是"用户主动操控"事件:
    // 到达即获取/续期独占(kWebExclusiveHoldSec 内 ROS cmd_vel 全丢弃);松手停发后锁自动
    // 过期,ROS 恢复控制。零速尾帧不续期(保证尽快还权),但仍写 control 让机器人停车。
    std::chrono::steady_clock::time_point last_web_active_time_{};
    bool IsWebExclusiveActive() const;                       // 独占锁是否在保持窗口内
    void ApplyWebVelocity(float x, float y, float yaw);      // Web 速度统一入口(死区/续期/写入)

    // 轮速目标低通滤波状态(热修改 2, 2026-10-02):
    // wheel_vel_lpf_ 保存上一拍滤波后的轮速目标;last_runmodel_time_ 用于
    // 检测策略退出/重入(中断过久则状态作废,首拍直接跟踪原始目标)。
    std::vector<float> wheel_vel_lpf_;
    std::chrono::steady_clock::time_point last_runmodel_time_;

    // C2-fix: 异步通知队列(SPSC无锁ring buffer),消除实时线程中 ROS publish / WebSocket send / mutex lock 的阻塞
    //   - Notify() 仅构建 JSON 并 Push,立即返回
    //   - FlushNotifyQueue() 由非实时线程(50ms timer)批量 Pop 并实际发送
    class AsyncNotifyQueue {
    public:
        void Push(std::string json) {
            size_t head = head_.load(std::memory_order_relaxed);
            size_t next = (head + 1) % kCapacity;
            if (next == tail_.load(std::memory_order_acquire)) {
                return;  // 队列满,丢弃(低频事件,可接受)
            }
            buffer_[head] = std::move(json);
            head_.store(next, std::memory_order_release);
        }
        bool Pop(std::string& json) {
            size_t tail = tail_.load(std::memory_order_relaxed);
            if (tail == head_.load(std::memory_order_acquire)) {
                return false;
            }
            json = std::move(buffer_[tail]);
            tail_.store((tail + 1) % kCapacity, std::memory_order_release);
            return true;
        }
    private:
        static constexpr size_t kCapacity = 64;
        std::array<std::string, kCapacity> buffer_;
        std::atomic<size_t> head_{0};
        std::atomic<size_t> tail_{0};
    };
    AsyncNotifyQueue async_notify_queue_;

    // 通知事件 ack 跟踪（通用 JSON 协议）
    struct NotifyRecord {
        std::string json;        // 已构建的 JSON 负载，重发时直接复用
        double timestamp = 0.0;
        int retry_count = 0;
    };
    std::atomic<uint32_t> notify_seq_{0};
    std::mutex notify_mutex_;
    std::map<uint32_t, NotifyRecord> notify_pending_;
    double notify_last_retry_ts_ = 0.0;

    // ===== 内置 Web 调试台（USE_CMAKE 裸机构建启用,替代 ROS2 话题层,参考 v1） =====
    std::unique_ptr<WebBridge> web_bridge_;
    // Web 层入口（仅非 ROS 构建使用）
    void WebCmdvelCallback(const WebBridge::CmdVel &v);
    void HandleNotifyAck(const std::string &s);
    bool HandleCheckStand();  // 站立前检查:趴姿近零位+IMU,广播 check_stand_result, 返回 ready
    // 站立期望值校验核心:12 个腿关节当前角是否在 default_dof_pos(站立期望)邻域内且静止,
    // 同时校验 IMU 有数据、CAN 反馈新鲜。返回 at_stand;若 reason 非空则写入失败原因。
    // 不广播结果(供 locomotion 门禁等内部调用),广播由 HandleCheckStanding() 负责。
    bool CheckStandingPose(std::string* reason = nullptr,
                           float* worst_err = nullptr, int* worst_idx = nullptr);
    // 站立期望值校验(对外命令):调用 CheckStandingPose() 后广播 check_standing_result,
    // 返回 at_stand。与 HandleCheckStand(趴姿检查)对称。
    bool HandleCheckStanding();
    // 姿态归类:lying(趴下)/standing(站立)/locomotion(强化学习)/transition(插值中)/unknown,
    // 返回 JSON 串 {"posture":..,"fsm":..,"settled":..,"motors_ready":..}
    std::string BuildPostureJson();
    // 广播当前姿态(ROS /rl_real/posture + WebSocket posture),返回 JSON 串
    std::string PublishPosture();
    // C2-fix: 批量处理异步通知队列(非实时线程调用,50ms timer)
    void FlushNotifyQueue();
};