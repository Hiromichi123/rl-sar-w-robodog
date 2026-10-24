#include "rl_real_JXG.hpp"
#include "crash_dump.hpp"
#include "motor_manager.h"
#include <csignal>
#include <atomic>
#include <thread>
#include <functional>
#include <string>
#include <sstream>
#include <cctype>
#include <algorithm>

// 版本/构建时间宏兜底(正常由 CMake target_compile_definitions 提供;
// 仅当绕过 CMake 直接编译时启用,避免未定义报错)
#ifndef APP_VERSION
#define APP_VERSION "0.0.0-dev"
#define BUILD_NUMBER "unknown"
#define BUILD_TIMESTAMP "unknown"
#endif
#include <chrono>
#include <cmath>
#include <cstdio>
#include <iomanip>  // For std::setw, std::fixed, std::setprecision
#include <queue>
#include <mutex>
#include <fstream>
#include <ctime>
#include <sys/stat.h>
#include <sys/mman.h>
#include <cstring>
#include <cerrno>
#include <cstdlib>  // std::getenv
#include <sys/resource.h>  // 常驻分段诊断:getrusage(RUSAGE_THREAD)
#include <sched.h>         // sched_setaffinity/CPU_SET:大小核布局
#include <dirent.h>        // opendir/readdir:遍历 /proc/self/task 纠正预存线程 affinity
#include <unistd.h>        // getpid
#include <sys/syscall.h>   // SYS_gettid

RobotMotorManager motor_manager;

std::atomic<bool> g_shutdown_flag(false);
std::atomic<bool> g_affinity_watchdog_exit(false);

// ===== 编译层选择(v4: 两层解耦,可独立开关,也可共存) =====
// JXG_ROS_LAYER: ROS2 真实控制(USE_ROS2_REAL_CONTROL)模式,话题/ROS 消息收发
// JXG_WEB_LAYER: 内置 Web 调试台(HTTP 静态页 + WebSocket),由 CMake 选项
//                ENABLE_WEB_BRIDGE(宏 JXG_ENABLE_WEB_BRIDGE)独立控制。
// v3 及之前 WEB == !ROS 互斥; v4 起两宏可同时为 1: 同一进程内 ROS2 话题与
// WebSocket 同时收发(发送点双发、输入点经统一入口合流)。
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

// 单调时钟当前秒数(非 ROS 模式下替代 rclcpp Node::now(),用于 notify 重试/超时计时)
static double SteadyNowSec()
{
    return std::chrono::duration<double>(
               std::chrono::steady_clock::now().time_since_epoch()).count();
}

// 执行 shell 命令并返回 stdout(失败返回空串)。仅用于系统管理类命令(svc/ps/sysinfo),
// 调用方必须对参数做白名单校验,禁止把用户原始字符串直接拼进命令。
static std::string ExecShell(const std::string &cmd)
{
    std::string out;
    FILE *fp = popen(cmd.c_str(), "r");
    if (!fp) return out;
    char buf[1024];
    while (fgets(buf, sizeof(buf), fp)) out.append(buf);
    pclose(fp);
    return out;
}

// systemd unit 名称白名单:只允许 字母/数字/.-@_ ,防注入(svc 命令的 unit 参数)
static bool IsValidUnitName(const std::string &s)
{
    if (s.empty() || s.size() > 128) return false;
    for (char c : s)
        if (!(isalnum((unsigned char)c) || c == '.' || c == '-' || c == '_' || c == '@'))
            return false;
    return true;
}

// 上位机 cmd_vel 命令超时保护：超过该时长未收到新的 cmd_vel，自动归零速度，
// 防止上位机(rosbridge websocket)断开后机器狗保持最后一条速度指令持续前冲。
// 上位机若以低于 2Hz 发送 cmd_vel，请适当调大该值。
static constexpr double kCmdVelTimeoutSec = 0.5;

// Web 速度独占保持时长:收到「非零」Web 速度后,该时间内 ROS cmd_vel 被丢弃(Web 独占)。
// 网页按住摇杆以 20Hz 持续发非零帧(间隔 50ms),故操控期间独占一直续期;松手补发 5 帧
// 零速(约 250ms)后停发,本窗口(1s)过后自动释放,ROS 恢复控制。取 1s 远大于发送间隔、
// 又略晚于 0.5s 断联归零,保证"先停车、后还权"。
static constexpr double kWebExclusiveHoldSec = 1.0;

// cmd_vel 死区(JXG 轮速失控热修改 1, 2026-10-02):
// 手柄/摇杆零位漂移(经 commands_scale 放大后)会被策略当成有效指令,
// 零命令时仍输出大幅轮速。|v| < 0.05 一律归零,ROS / Web 两条指令链路共用。
static constexpr float kCmdVelDeadzone = 0.05f;
static inline float ApplyDeadzone(float v)
{
    return std::abs(v) < kCmdVelDeadzone ? 0.0f : v;
}

// 轮速目标低通滤波系数(JXG 轮速失控热修改 2):
// loop_rl 以 dt*decimation = 0.005*4 = 20ms(50Hz)运行,
// alpha=0.3 时等效时间常数 tau≈2.8*20ms≈56ms,截止频率≈2.8Hz,
// 用于压掉策略轮速目标的高频抖动;若觉得影响平衡响应可调大(趋向 1=不滤波)。
static constexpr float kWheelVelLpfAlpha = 0.3f;
// RunModel 中断超过该时长(策略退出/重入)后,滤波器状态作废,首拍直接跟踪原始值,
// 避免用退出前的陈旧轮速做插值造成起步突变。
static constexpr double kWheelLpfResetGapSec = 0.1;

// 最小 JSON 转义（反斜杠、双引号），用于无第三方 JSON 库时手动拼 JSON
static std::string JsonEscape(const std::string &s)
{
    std::string out = s;
    for (size_t p = 0; (p = out.find('\\', p)) != std::string::npos; p += 2)
        out.replace(p, 1, "\\\\");
    for (size_t p = 0; (p = out.find('"', p)) != std::string::npos; p += 2)
        out.replace(p, 1, "\\\"");
    return out;
}



  void signalHandler(int signum) {
      std::cout << "\nReceived interrupt signal. Shutting down..." <<
  std::endl;
      g_shutdown_flag.store(true);
  }


RL_Real::RL_Real(const WebConfig &web_cfg)
#if defined(USE_ROS2_REAL_CONTROL) || (defined(USE_ROS2) && defined(USE_ROS))
    : rclcpp::Node("rl_real_node")
#endif
{
    this->ang_vel_axis = "body";
    // v5: 仅保留 JXGw 轮腿模型,启动即执行,不再通过命令行参数选择
    this->robot_name = "JXGw";
    this->ReadYaml(this->robot_name, "base.yaml");
    this->InitJointNum(this->params.Get<int>("num_of_dofs"));

    // 一次性缓存 200Hz 实时环用到的静态参数(此后 GetState/SetCommand 不再解析 YAML/分配 vector)
    this->cfg_num_dofs = this->params.Get<int>("num_of_dofs");
    this->cfg_joint_mapping = this->params.Get<std::vector<int>>("joint_mapping");
    this->cfg_lying_offset = this->params.Get<std::vector<float>>("lying_down_offset");
    this->InitDiffDrive();
    // 下发缓冲区按 CAN 电机总数 16 分配(sendAllMotorsCommand 以 CAN idx 索引且要求 size==16)
    this->cmd_tau_buf.assign(16, 0.0f);
    this->cmd_vel_buf.assign(16, 0.0f);
    this->cmd_pos_buf.assign(16, 0.0f);
    this->cmd_kp_buf.assign(16, 0.0f);
    this->cmd_kd_buf.assign(16, 0.0f);

    // FSM
    if (FSMManager::GetInstance().IsTypeSupported(this->robot_name))
    {
        auto fsm_ptr = FSMManager::GetInstance().CreateFSM(this->robot_name, this);
        if (fsm_ptr) this->fsm = *fsm_ptr;
    }

    // 硬件初始化(v7.2: 传输层由 base.yaml 的 can_transport 决定: usb2can|socketcan)
    motor_manager.init(this->params.Get<std::string>("can_transport"));

    // 先装填首帧安全指令,再启动 500Hz 腿线程:这样线程从第一拍起发送的就是
    // kp=0/kd=20 纯阻尼帧(配合 LegCommand 零初始化),杜绝"使能瞬间槽内是
    // 未初始化增益"的砸地窗口。电机未使能时这些帧不产生力矩,仅驱动状态帧回流。
    this->InitLowCmd();
    {
        std::vector<float> tau_z(16, 0.0f), vel_z(16, 0.0f);
        std::vector<float> kp_z(16, 0.0f), kd_damp(16, 20.0f), q_init(16, 0.0f);
        for (int i = 0; i < 16; ++i) q_init[i] = this->JXG_cmd.motor_cmd[i].q;
        motor_manager.sendAllMotorsCommand(tau_z, vel_z, q_init, kp_z, kd_damp);
    }

    motor_manager.startThreads();

    // 等待状态帧回流(崩溃前仍处使能的电机会持续应答;冷机未使能则为零反馈)
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    // IMU 提前到使能之前启动:SafeHold 首拍及 RobotControl 的 IMU 掉线保护立即可用
    open_imu();

    this->loop_imu = std::make_shared<LoopFunc>("loop_imu", 0.001, []() {
        read_imu();  // 1kHz
    }, 4 /*CPU*/, 90 /*FIFO 最高:IMU 采样*/);
    this->loop_imu->start();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // ★ 崩溃重启安全核心:使能前判定当前是否为站立构型。
    std::vector<float> startup_hold_q;
    this->startup_standing_ = this->DetectStandingStartup(startup_hold_q);
    if (this->startup_standing_)
    {
        // 预装填"当前位 PD 保持"帧:每个电机被 0xFC 使能后的第一拍即收到
        // kp=fixed_kp/q=实测角 的保持指令,而不是默认 kp=0 阻尼(站立会砸地)。
        std::vector<float> tau_z(16, 0.0f), vel_z(16, 0.0f);
        auto fkp = this->params.Get<std::vector<float>>("fixed_kp");
        auto fkd = this->params.Get<std::vector<float>>("fixed_kd");
        if (fkp.size() != 16) fkp.assign(16, 0.0f);
        if (fkd.size() != 16) fkd.assign(16, 1.0f);
        motor_manager.sendAllMotorsCommand(tau_z, vel_z, startup_hold_q, fkp, fkd);
        this->motors_ready_for_stand = true;  // 已在站立位,无需近零位门禁
        std::cout << LOGGER::WARNING << "[INIT] STANDING posture detected BEFORE motor enable: "
                     "motors will hold current pose during/after enable (SafeHold)." << std::endl;
    }

    // 逐个使能 16 台电机(16x100ms≈1.6s)。使能瞬间 500Hz 指令流已是保持帧/阻尼帧。
    motor_manager.initAllMotors();
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    // ★ 使能后反馈验证:等待所有电机反馈回流,避免 CAN 链路未恢复时
    //   CheckMotorInitPosition 读到默认零值误判通过。
    {
        constexpr int kWaitFeedbackTimeoutMs = 3000;   // 最多等 3s
        constexpr int kWaitFeedbackStepMs    = 50;     // 50ms 轮询
        constexpr int64_t kFeedbackFreshMs   = 200;    // 反馈新鲜度阈值
        int waited = 0;
        while (waited < kWaitFeedbackTimeoutMs)
        {
            int n_no_fb = 0;
            for (int i = 0; i < 16; ++i)
            {
                int64_t age = motor_manager.motorFeedbackAgeMs(i);
                if (age < 0 || age > kFeedbackFreshMs) ++n_no_fb;
            }
            if (n_no_fb == 0) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(kWaitFeedbackStepMs));
            waited += kWaitFeedbackStepMs;
        }
        // 报告仍无反馈的电机(不阻塞启动,但标记 motors_ready_for_stand=false)
        std::vector<int> no_fb_ids;
        for (int i = 0; i < 16; ++i)
        {
            int64_t age = motor_manager.motorFeedbackAgeMs(i);
            if (age < 0 || age > kFeedbackFreshMs) no_fb_ids.push_back(i);
        }
        if (!no_fb_ids.empty())
        {
            std::ostringstream oss;
            oss << "[INIT] WARNING: " << no_fb_ids.size()
                << " motor(s) still no feedback after enable:";
            for (int id : no_fb_ids) oss << " m" << id;
            std::cout << oss.str() << std::endl;
            this->motors_not_ready_reason =
                "Motor(s) no feedback after enable:" + oss.str().substr(oss.str().find(":"));
            this->motors_ready_for_stand = false;
        }
        else
        {
            std::cout << "[INIT] All 16 motors feedback confirmed after enable ("
                      << waited << "ms)" << std::endl;
        }
    }

    if (!this->startup_standing_)
    {
        // 非站立路径:沿用原一次性零位检查语义(使能、反馈稳定后判定)
        this->motors_ready_for_stand = this->CheckMotorInitPosition(1.0f);  // 阈值1.0rad
        if (this->motors_ready_for_stand) {
            std::cout << "[INIT] Motor position check PASSED" << std::endl;
        } else {
            std::cout << "[INIT] Motor position check FAILED: " << this->motors_not_ready_reason << std::endl;
        }
    }

    // 使能后 500ms 安全桥接(100 拍):
    //  - 站立:重新抓取使能后的最新关节角,以 fixed_kp/fixed_kd 持续保持;
    //  - 其他:kp=0/kd=20 纯阻尼。
    // 腿线程会以 500Hz 持续重放最后一帧,故本桥接结束到控制环首拍之间
    // (InitOutputs/InitControl/ROS/Web 初始化,数百 ms)电机也不会失控。
    {
        std::vector<float> tau(16, 0.0f), vel(16, 0.0f), kp(16, 0.0f), kd(16, 20.0f), pos(16, 0.0f);
        if (this->startup_standing_)
        {
            Eigen::VectorXf cur_pos = motor_manager.getAllMotorPos();
            for (int i = 0; i < 16; ++i) startup_hold_q[i] = cur_pos[i];
            auto fkp = this->params.Get<std::vector<float>>("fixed_kp");
            auto fkd = this->params.Get<std::vector<float>>("fixed_kd");
            if (fkp.size() != 16) fkp.assign(16, 0.0f);
            if (fkd.size() != 16) fkd.assign(16, 1.0f);
            pos = startup_hold_q;
            kp  = fkp;
            kd  = fkd;
        }
        else
        {
            for (int i = 0; i < 16; ++i) pos[i] = this->JXG_cmd.motor_cmd[i].q;
        }
        for (int i = 0; i < 100; ++i) {
            motor_manager.sendAllMotorsCommand(tau, vel, pos, kp, kd);
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }

    // 控制环首拍前预置 FSM:站立→SafeHold(200Hz 接管保持),否则保持 Passive。
    // 首拍 CHANGE 分支直接 Passive.Exit→SafeHold.Enter,Passive.Run 的 kp=0
    // 卸力帧一帧都不会发出。
    if (this->startup_standing_)
    {
        this->fsm.RequestStateChange("RLFSMStateSafeHold");
    }


    this->InitOutputs();
    this->InitControl();

#if defined(USE_ROS2_REAL_CONTROL)
    auto ros_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
    this->cmd_vel_subscriber = this->create_subscription<geometry_msgs::msg::Twist>(
        "/rl_real/cmd_vel", ros_qos, std::bind(&RL_Real::CmdvelCallback, this, std::placeholders::_1));
    // 网页独占速度源(standalone web_server 转发浏览器 WS cmd_vel 到此)。
    this->web_cmd_vel_subscriber = this->create_subscription<geometry_msgs::msg::Twist>(
        "/rl_real/web_cmd_vel", ros_qos, std::bind(&RL_Real::WebCmdVelRosCallback, this, std::placeholders::_1));
    this->command_subscriber = this->create_subscription<std_msgs::msg::String>(
        "/rl_real/command", ros_qos, std::bind(&RL_Real::CommandCallback, this, std::placeholders::_1));
    this->imu_stream_subscriber = this->create_subscription<std_msgs::msg::Bool>(
        "/rl_real/imu_stream_enable", ros_qos, std::bind(&RL_Real::ImuStreamCallback, this, std::placeholders::_1));
    this->feedback_publisher = this->create_publisher<std_msgs::msg::String>(
        "/rl_real/feedback", ros_qos);
    this->motor_state_publisher = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        "/rl_real/motor_state", ros_qos);
    this->imu_state_publisher = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        "/rl_real/imu_state", ros_qos);
    this->pose2d_publisher = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        "/rl_real/pose2d", ros_qos);
    this->notify_publisher = this->create_publisher<std_msgs::msg::String>(
        "/rl_real/notify", ros_qos);
    this->notify_ack_subscriber = this->create_subscription<std_msgs::msg::String>(
        "/rl_real/notify_ack", ros_qos, std::bind(&RL_Real::NotifyAckCallback, this, std::placeholders::_1));
    this->check_stand_subscriber = this->create_subscription<std_msgs::msg::String>(
        "/rl_real/check_stand", ros_qos, std::bind(&RL_Real::CheckStandCallback, this, std::placeholders::_1));
    this->check_stand_result_publisher = this->create_publisher<std_msgs::msg::String>(
        "/rl_real/check_stand_result", ros_qos);
    // 姿态接口: 8Hz 周期广播 + posture 命令即时应答
    this->posture_publisher = this->create_publisher<std_msgs::msg::String>(
        "/rl_real/posture", ros_qos);
    // 站立期望值校验(与趴姿检查对称): 请求 /rl_real/check_standing → 结果 /rl_real/check_standing_result
    this->check_standing_subscriber = this->create_subscription<std_msgs::msg::String>(
        "/rl_real/check_standing", ros_qos, std::bind(&RL_Real::CheckStandingCallback, this, std::placeholders::_1));
    this->check_standing_result_publisher = this->create_publisher<std_msgs::msg::String>(
        "/rl_real/check_standing_result", ros_qos);
    // BMS 电池状态:web_server 1Hz 发布,用于低电量持续报警
    this->bms_state_subscriber = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/rl_real/bms_state", ros_qos, std::bind(&RL_Real::BmsStateCallback, this, std::placeholders::_1));
    // 命令生命周期合成层 + 2Hz 心跳(与 WebSocket 同帧双发, 大脑 HAL/桥硬依赖)
    this->cmd_state_publisher = this->create_publisher<std_msgs::msg::String>(
        "/rl_real/cmd_state", ros_qos);
    this->heartbeat_publisher = this->create_publisher<std_msgs::msg::String>(
        "/rl_real/heartbeat", ros_qos);
    // safe_getup 编排推进:50ms 周期。SingleThreadedExecutor 下与命令回调同线程,
    // 阶段切换/SetKeyboard 与手动命令完全串行,无需互斥锁(探针线程仅写 atomic)。
    this->safe_getup_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50), [this]() { this->TickSafeGetup(); });
    // v4 共存: 初始零位 Notify 由 Web 块统一双发; 仅纯 ROS 构建走这里。
#if !JXG_WEB_LAYER
    std::cout << LOGGER::INFO << "ROS2 real control enabled: /rl_real/cmd_vel, /rl_real/command, /rl_real/feedback, /rl_real/motor_state, /rl_real/imu_state, /rl_real/pose2d, /rl_real/posture, /rl_real/check_standing" << std::endl;
    // 初始零位检查结果上报（发布器就绪后补发一次）
    if (this->motors_ready_for_stand) {
        this->Notify("info", "motor position check passed");
    } else {
        this->Notify("error", this->motors_not_ready_reason);
    }
#endif
#endif

#if JXG_WEB_LAYER
    // ===== 内置 Web 调试台桥（HTTP 静态页面 + WebSocket 控制/遥测，替代 ROS2 + rosbridge） =====
    // 独立 web_server 进程(jxg-web.service)已常驻时, 通过 JXG_WEB_STANDALONE=1 让位:
    // 本进程不再内嵌 Web, 全部 Web 流量经 ROS2 话题由独立进程桥接,
    // 这样重启本控制程序时浏览器的 WS/WebSSH/WebFTP 连接完全不受影响。
    bool embedded_web_disabled = false;
    if (const char *e = std::getenv("JXG_WEB_STANDALONE"))
        embedded_web_disabled = (e[0] == '1' || e[0] == 't' || e[0] == 'T' || e[0] == 'y' || e[0] == 'Y');
    if (embedded_web_disabled)
    {
        std::cout << LOGGER::INFO
                  << "Embedded web bridge disabled (JXG_WEB_STANDALONE=1); "
                     "web console served by standalone web_server process" << std::endl;
    }
    else
    {
    this->web_bridge_ = std::make_unique<WebBridge>();
    this->last_cmd_vel_time_ = std::chrono::steady_clock::now();
    if (!this->web_bridge_->start(
            web_cfg.web_port, web_cfg.web_root,
            [this](const WebBridge::CmdVel &v) { this->WebCmdvelCallback(v); },
            [this](const std::string &s, int seq) { this->ApplyRosCommand(s, seq); },
            [this](bool b) {
                SetJXGIMUStreamEnabled(b);
                this->PublishRosFeedback(std::string("imu_stream=") + (b ? "on" : "off"));
            },
            [this](const std::string &s) { this->HandleNotifyAck(s); },
            [this]() { this->HandleCheckStand(); }))
    {
        std::cout << LOGGER::WARNING << "Web bridge failed to start (port "
                  << web_cfg.web_port << " in use?), web console disabled" << std::endl;
        this->web_bridge_.reset();
    }
    std::cout << LOGGER::INFO << "Web control enabled: cmd_vel, command, feedback, motor_state, imu_state, pose2d" << std::endl;
    // 初始零位检查结果上报（Web 桥就绪后补发一次）
    if (this->motors_ready_for_stand) {
        this->Notify("info", "motor position check passed");
    } else {
        this->Notify("error", this->motors_not_ready_reason);
    }
    }  // end else: 内嵌 Web 未被 JXG_WEB_STANDALONE 禁用
#endif

    // this->loop_keyboard = std::make_shared<LoopFunc>("loop_keyboard", 0.05, std::bind(&RL_Real::KeyboardInterface, this));
    this->loop_control  = std::make_shared<LoopFunc>("loop_control",  this->params.Get<float>("dt"), std::bind(&RL_Real::RobotControl, this),
                                                     6 /*CPU:独占大核,200Hz 控制环无推理/IMU 干扰*/, 80 /*FIFO:200Hz 控制环*/);
    this->loop_rl       = std::make_shared<LoopFunc>("loop_rl",       this->params.Get<float>("dt") * this->params.Get<int>("decimation"), std::bind(&RL_Real::RunModel, this),
                                                     4 /*CPU:与 loop_imu 共存,IMU 负载~10% 不影响推理;control 独占 CPU6*/, 70 /*FIFO:50Hz ONNX 推理*/);
    // this->loop_keyboard->start();
    this->loop_control->start();
    this->loop_rl->start();

    // ── 周期性电机重使能线程(普通调度,不抢占 RT 线程) ──
    // 使能命令 0xFC 仅在启动时发一次;若电机上电晚于使能(电源顺序)或使能帧丢失,
    // 电机不进入 MIT 模式,后续所有命令被忽略。本线程每 5 秒重发一轮 0xFC,
    // 覆盖迟到上电的电机。对已使能电机幂等无害。
    std::thread([](){
        while (rclcpp::ok()) {
            std::this_thread::sleep_for(std::chrono::seconds(5));
            try { motor_manager.periodicReEnable(); } catch (...) {}
        }
    }).detach();

#ifdef ENABLE_HR_LOGGER
    this->HrLogInit();
#endif

#ifdef PLOT
    this->plot_t = std::vector<int>(this->plot_size, 0);
    this->plot_real_joint_pos.resize(this->params.Get<int>("num_of_dofs"));
    this->plot_target_joint_pos.resize(this->params.Get<int>("num_of_dofs"));
    for (auto &vector : this->plot_real_joint_pos)  vector = std::vector<double>(this->plot_size, 0);
    for (auto &vector : this->plot_target_joint_pos) vector = std::vector<double>(this->plot_size, 0);
    this->loop_plot = std::make_shared<LoopFunc>("loop_plot", 0.002, std::bind(&RL_Real::Plot, this));
    this->loop_plot->start();
#endif

#ifdef CSV_LOGGER
    this->CSVInit(this->robot_name);
#endif

#ifdef ENABLE_ASYNC_CSV_LOGGER
    // 初始化异步CSV记录器
    csv_logger = std::make_unique<AsyncCSVLogger>();
    if (csv_logger->init(this->robot_name, this->params.Get<int>("num_of_dofs")))
    {
        // 设置采集频率为10Hz (100ms)，不影响500Hz主循环
        csv_logger->setWriteInterval(100);
        csv_logger->start();

        // 记录开始时间
        csv_logger_start_time = std::chrono::duration<double>(
            std::chrono::steady_clock::now().time_since_epoch()).count();

        std::cout << "[AsyncCSVLogger] Data collection started at 10Hz" << std::endl;
    }
    else
    {
        csv_logger.reset();
        std::cerr << "[AsyncCSVLogger] Initialization failed" << std::endl;
    }
#endif
}

RL_Real::~RL_Real()
{

#ifdef ENABLE_HR_LOGGER
    this->HrLogClose();
#endif

    this->loop_control->shutdown();
    this->loop_rl->shutdown();
    // this->loop_keyboard->shutdown();  // 键盘线程已禁用
    motor_manager.stopThreads();
    motor_manager.disableAllMotors();
    this->loop_imu->shutdown();
#ifdef PLOT
    this->loop_plot->shutdown();
#endif

#ifdef ENABLE_ASYNC_CSV_LOGGER
    // 停止并清理CSV记录器
    if (csv_logger)
    {
        csv_logger->stop();
        csv_logger.reset();
    }
#endif

    std::cout << LOGGER::INFO << "RL_Real JXG exit safely" << std::endl;
}


void RL_Real::GetState(RobotState<float> *state)
{
    // 跨线程保护:updateJXGLowState 写 JXG_state 期间,遥测线程不得并发读。
    // 持锁范围覆盖"写+读"全程,保证 state 拿到的是一帧完整一致的快照。
    std::lock_guard<std::mutex> lk(this->jxg_state_mutex_);

    updateJXGLowState(this->JXG_state);
    state->imu.quaternion[0] = this->JXG_state.imu_state.quaternion[0]; // w
    state->imu.quaternion[1] = this->JXG_state.imu_state.quaternion[1]; // x
    state->imu.quaternion[2] = this->JXG_state.imu_state.quaternion[2]; // y
    state->imu.quaternion[3] = this->JXG_state.imu_state.quaternion[3]; // z
    for (int i = 0; i < 3; ++i) {
        state->imu.gyroscope[i] = this->JXG_state.imu_state.gyroscope[i];
    }


    // Debug: 打印 lying_down_offset 值（仅第一次）
    // static bool debug_offset_printed = false;
    // if (!debug_offset_printed) {
    //     std::cout << "[DEBUG] lying_down_offset values:" << std::endl;
    //     for (int i = 0; i < this->params.num_of_dofs; ++i) {
    //         std::cout << "  [" << i << "] = " << this->params.lying_down_offset[0][i].item<double>() << std::endl;
    //     }
    //     std::cout << "[DEBUG] Expected standing pos: [0.0, 0.8, -1.6, 0.0, 0.8, -1.6, 0.0, 0.8, -1.6, 0.0, 0.8, -1.6]" << std::endl;
    //     debug_offset_printed = true;
    // }

    // 实时环:直接使用构造期缓存的静态参数,零 YAML 解析、零堆分配
    const int num_dofs = this->cfg_num_dofs;
    const auto &jm = this->cfg_joint_mapping;
    const auto &off = this->cfg_lying_offset;
    for (int i = 0; i < num_dofs; ++i) {
        const int idx = (i < (int)jm.size()) ? jm[i] : i;
        const float offset_i = (i < (int)off.size()) ? off[i] : 0.0f;
        state->motor_state.q[i]       = this->JXG_state.motor_state[idx].q + offset_i;
        state->motor_state.dq[i]      = this->JXG_state.motor_state[idx].dq;
        state->motor_state.tau_est[i] = this->JXG_state.motor_state[idx].tau_est;
    }
}

void RL_Real::SetCommand(const RobotCommand<float> *command)
{
    // 复用构造期分配的成员缓冲区:零初始化未映射槽(与原每次新建零 vector 语义一致),
    // 避免 200Hz 下每周期 5 次 vector<float>(16) 堆分配。
    std::fill(cmd_tau_buf.begin(), cmd_tau_buf.end(), 0.0f);
    std::fill(cmd_vel_buf.begin(), cmd_vel_buf.end(), 0.0f);
    std::fill(cmd_pos_buf.begin(), cmd_pos_buf.end(), 0.0f);
    std::fill(cmd_kp_buf.begin(),  cmd_kp_buf.end(),  0.0f);
    std::fill(cmd_kd_buf.begin(),  cmd_kd_buf.end(),  0.0f);

    const int num_dofs = this->cfg_num_dofs;
    const auto &jm = this->cfg_joint_mapping;
    const auto &off = this->cfg_lying_offset;
    for (int i = 0; i < num_dofs; ++i) {
        const int idx = (i < (int)jm.size()) ? jm[i] : i;
        const float offset_i = (i < (int)off.size()) ? off[i] : 0.0f;
        cmd_pos_buf[idx] = command->motor_command.q[i] - offset_i;
        cmd_vel_buf[idx] = command->motor_command.dq[i];
        cmd_kp_buf[idx]  = command->motor_command.kp[i];
        cmd_kd_buf[idx]  = command->motor_command.kd[i];
        cmd_tau_buf[idx] = command->motor_command.tau[i];
    }

    motor_manager.sendAllMotorsCommand(cmd_tau_buf, cmd_vel_buf, cmd_pos_buf, cmd_kp_buf, cmd_kd_buf);
}

// 常驻轻量诊断:loop_control 分段 CPU 归因(用户/内核),每秒覆盖写 /tmp/rl_prof.log。
// 每周期 6 次 getrusage(RUSAGE_THREAD)(vDSO 级,实测 <0.1% 核),用于定位实时环偶发 sys 飙升。
namespace {
struct LoopProf {
    static constexpr int N = 6;
    long u[N] = {0}, s[N] = {0}, calls[N] = {0};
    int cyc = 0;
    static inline std::pair<long,long> rus() {
        struct rusage r{};
        ::getrusage(RUSAGE_THREAD, &r);
        return { r.ru_utime.tv_sec*1000000L + r.ru_utime.tv_usec,
                 r.ru_stime.tv_sec*1000000L + r.ru_stime.tv_usec };
    }
};
static LoopProf g_lprof;
struct ProfGuard {
    int id; long u0, s0;
    ProfGuard(int i) : id(i) { auto p = LoopProf::rus(); u0 = p.first; s0 = p.second; }
    ~ProfGuard() {
        auto p = LoopProf::rus();
        g_lprof.u[id] += p.first  - u0;
        g_lprof.s[id] += p.second - s0;
        ++g_lprof.calls[id];
    }
};
}

void RL_Real::RobotControl()
{
    // 常驻轻量诊断:正常 RobotControl ~0.1ms,若某周期超过 4ms 说明卡在某段
    // (曾观测到 loop_control 偶发 95% sys spin)。把 FSM 状态名 + backtrace 追加写 /tmp/rl_stall.log。
    // 正常路径仅两次读时钟,不做任何文件/栈操作。
    struct StallDiag
    {
        RL_Real* self;
        std::chrono::steady_clock::time_point t0;
        explicit StallDiag(RL_Real* s) : self(s), t0(std::chrono::steady_clock::now()) {}
        ~StallDiag()
        {
            double ms = std::chrono::duration<double, std::milli>(
                            std::chrono::steady_clock::now() - t0).count();
            if (ms < 4.0) return;
            static std::atomic<int> n{0};
            int k = n.fetch_add(1);
            if (k >= 300) return;  // 最多抓 300 次(阈值4ms,避开RL态~3.2ms无害抖动)
            std::string st = self->fsm.GetCurrentStateName();
            if (st.empty()) st = "?";
            int fd = ::open("/tmp/rl_stall.log", O_WRONLY | O_CREAT | O_APPEND, 0666);
            if (fd < 0) return;
            char hb[256];
            int m = snprintf(hb, sizeof(hb), "\n===== STALL %.2fms state=%s =====\n", ms, st.c_str());
            ::write(fd, hb, m);
            void* fr[48];
            int c = backtrace(fr, 48);
            backtrace_symbols_fd(fr, c, fd);
            ::close(fd); ::chmod("/tmp/rl_stall.log", 0666);
        }
    };
    StallDiag _stall(this);

    this->motiontime++;

     if (this->control.current_keyboard == Input::Keyboard::W)
    {
        this->control.x += 0.1;
        this->control.current_keyboard = this->control.last_keyboard;
    }
    if (this->control.current_keyboard == Input::Keyboard::S)
    {
        this->control.x -= 0.1;
        this->control.current_keyboard = this->control.last_keyboard;
    }
    if (this->control.current_keyboard == Input::Keyboard::A)
    {
        this->control.y += 0.1;
        this->control.current_keyboard = this->control.last_keyboard;
    }
    if (this->control.current_keyboard == Input::Keyboard::D)
    {
        this->control.y -= 0.1;
        this->control.current_keyboard = this->control.last_keyboard;
    }
    if (this->control.current_keyboard == Input::Keyboard::Q)
    {
        this->control.yaw += 0.1;
        this->control.current_keyboard = this->control.last_keyboard;
    }
    if (this->control.current_keyboard == Input::Keyboard::E)
    {
        this->control.yaw -= 0.1;
        this->control.current_keyboard = this->control.last_keyboard;
    }
    if (this->control.current_keyboard == Input::Keyboard::Space)
    {
        this->control.x = 0;
        this->control.y = 0;
        this->control.yaw = 0;
        this->control.current_keyboard = this->control.last_keyboard;
    }

    // updateJXGLowState(this->JXG_state);
    {
        ProfGuard _pg(0);
        this->GetState(&this->robot_state);
    }

    // IMU 连续检查：陀螺仪全零说明IMU无数据，连续 100ms 才切入落下模式
    // 避免瞬时 SPI 读零(单帧抖动)误触发趴下。
    { ProfGuard _pg(1);
    static int imu_zero_count = 0;
    constexpr int kImuZeroLimit = 20;  // 20×5ms=100ms
    if (this->JXG_state.imu_state.gyroscope[0] == 0.0 &&
        this->JXG_state.imu_state.gyroscope[1] == 0.0 &&
        this->JXG_state.imu_state.gyroscope[2] == 0.0)
    {
        if (++imu_zero_count == kImuZeroLimit)  // 仅达标时触发一次
        {
            std::string cur_state = this->fsm.GetCurrentStateName();
            if (cur_state != "RLFSMStatePassive")
            {
                // write(2) 替代 std::cout:避免 iostream 锁开销;异常路径用 fprintf 即可
                fprintf(stderr, "[WARN] IMU data lost (gyro all-zero for 100ms)! Forcing GetDown...\n");
                this->Notify("warning", "IMU data lost! Forcing GetDown...");
                this->fsm.RequestStateChange("RLFSMStateGetDown");
            }
        }
    }
    else
    {
        imu_zero_count = 0;  // 恢复正常:清零计数
    }

    // 电机反馈连续检查：
    if (this->rl_init_done)
    {
        // 防御:确保每电机计数器容量(历史上 {16,0} 花括号误初始化成 2 元素导致越界 abort)
        if ((int)motor_dead_count.size() != 16)
        {
            std::cout << LOGGER::WARNING << "motor_dead_count size=" << motor_dead_count.size()
                      << " != 16, auto-resizing" << std::endl;
            motor_dead_count.assign(16, 0);
        }
        constexpr float dead_eps = 0.01f;
        constexpr int dead_cycles_limit = 20;  // 20×5ms=100ms，约2次RL推理窗口
        // 轮电机索引位掩码(3/7/11/15),替代 std::find 线性搜索:200Hz 下省 12×4 次比较/周期
        static const uint16_t wheel_mask = (1u << 3) | (1u << 7) | (1u << 11) | (1u << 15);

        for (int i = 0; i < 16; ++i)
        {
            if (wheel_mask & (1u << i))
                continue;

            float dq  = this->JXG_state.motor_state[i].dq;
            float tau = this->JXG_state.motor_state[i].tau_est;

            if (std::abs(dq) < dead_eps && std::abs(tau) < dead_eps)
            {
                motor_dead_count[i]++;
                if (motor_dead_count[i] == dead_cycles_limit)  // 仅触发时打印一次
                {
                    std::string cur_state = this->fsm.GetCurrentStateName();
                    // SafeHold 下机器人可能被外力完全承托(架起/人扶),vel&tau 长期≈0 是正常的
                    if (cur_state != "RLFSMStatePassive" && cur_state != "RLFSMStateGetDown"
                        && cur_state != "RLFSMStateSafeHold")
                    {
                        fprintf(stderr, "[WARN] Motor[%d] feedback abnormal (vel&tau≈0 for 100ms)! Forcing GetDown...\n", i);
                        this->Notify("warning", "Motor[" + std::to_string(i) + "] feedback abnormal (vel&tau≈0 for 100ms)! Forcing GetDown...");
                        this->fsm.RequestStateChange("RLFSMStateGetDown");
                    }
                }
            }
            else
            {
                motor_dead_count[i] = 0;
            }
        }
    }

    } // end S1(安全检查)

    // ── SafetyHold: 任一电机 CAN 反馈超时(>100ms)时,冻结所有电机为当前位置 ──
    // ── P1 倾覆保护 + IMU 看门狗 + NaN 告警消费(仅 RL 态判定,并入 SafeHold 拦截) ──
    // 倾覆(P1-A):|roll|>0.9rad 或 |pitch|>0.8rad 连续 2 帧(10ms@200Hz)确认,防瞬时
    //   误判;GetUp/GetDown 过程姿态本就大,只在 RLLocomotion 态检查,SafeHold 驻留的
    //   任何姿态(含已倾覆)都不再切换,避免震荡。
    // IMU(P1-B):数据凝固检测——quat4+gyro3+acc3 共 10 个 double 与上次采样完全相等
    //   即未更新,凝固 >200ms 判为断供(DM-IMU double 噪声下静止也持续微变,正常不会
    //   误判;含从未上电的默认恒值情形);另拦四元数 NaN 与模长平方偏离 1 超 ±10%。
    // NaN(P1-B):推理线程连续 3 帧坏帧置 nan_alert_,此处 exchange 消费并复位。
    // 触发后与 CAN 断线/手动阻断同构:RL 态当帧切 SafeHold,无一帧策略输出,需人工重进。
    const std::string pre_hold_state = this->fsm.GetCurrentStateName();
    bool tilt_trigger = false, imu_trigger = false, nan_trigger = false;
    if (pre_hold_state == "RLFSMStateRLLocomotion" || pre_hold_state == "RLFSMStateDiffDrive")
    {
        const auto now_guard = std::chrono::steady_clock::now();
        const auto* q = this->JXG_state.imu_state.quaternion;
        const auto* g = this->JXG_state.imu_state.gyroscope;
        const auto* ac = this->JXG_state.imu_state.accelerometer;

        // P1-A 倾覆(ZYX 欧拉,rad)
        const double roll = std::atan2(2.0 * (q[0] * q[1] + q[2] * q[3]),
                                       1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2]));
        double sinp = 2.0 * (q[0] * q[2] - q[3] * q[1]);
        if (sinp > 1.0) sinp = 1.0; else if (sinp < -1.0) sinp = -1.0;
        const double pitch = std::asin(sinp);
        static int tilt_streak = 0;
        if (std::fabs(roll) > 0.9 || std::fabs(pitch) > 0.8) { ++tilt_streak; }
        else { tilt_streak = 0; }
        tilt_trigger = (tilt_streak >= 2);

        // P1-B IMU 凝固 / NaN / 模长(qn2 为 NaN 时两比较恒 false,由 quat_nan 单独拦)
        static double last_sig[10] = {0};
        static auto last_imu_change = now_guard;
        const double sig[10] = {q[0], q[1], q[2], q[3], g[0], g[1], g[2], ac[0], ac[1], ac[2]};
        bool same = true;
        for (int k = 0; k < 10; ++k) { if (sig[k] != last_sig[k]) { same = false; break; } }
        if (!same)
        {
            for (int k = 0; k < 10; ++k) last_sig[k] = sig[k];
            last_imu_change = now_guard;
        }
        bool quat_nan = false;
        for (int k = 0; k < 4; ++k) { if (!std::isfinite(q[k])) { quat_nan = true; break; } }
        const double qn2 = q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3];
        imu_trigger = (now_guard - last_imu_change > std::chrono::milliseconds(200)) ||
                      quat_nan || (qn2 < 0.81 || qn2 > 1.21);

        // P1-B 推理输出 NaN 连续坏帧告警消费
        if (pre_hold_state == "RLFSMStateRLLocomotion")
            nan_trigger = this->nan_alert_.exchange(false, std::memory_order_acq_rel);
    }

    // v7.2 关键安全机制(对齐 v7.1):CAN 断线时驱动器保持执行最后一帧指令,而软件
    // 仍使用冻结反馈——若继续放行 RL 策略,会基于错配观测输出导致姿态突变。
    const bool can_safety = this->SafetyHoldActive();
    // 手动"阻断强化学习"急停:与 CAN 保护走同一套当帧 SafeHold 逻辑。
    const bool manual_block = this->manual_rl_block_.load(std::memory_order_relaxed);
    // BMS 低电量持续报警:每帧检查,SOC/电压超阈值时每 10s 推送一次 notify。
    // 仅报警不强制 SafeHold(留给人工决策;如需强制停机可在此扩展)。
    this->CheckLowBattery();
    this->CheckMotorTempAlert();
    const bool safety_hold_active = can_safety || manual_block || tilt_trigger || imu_trigger || nan_trigger;
    // 每帧(200Hz)刷新 CAN 健康标志并供 posture 广播:必须放在拦截 return 之前,
    // 否则断线后非 RL 分支 return 跳过遥测段,can_ready 会冻结在断线前的 true,
    // 前端永远显示"CAN 正常"(v7.1 同类缺陷)。100ms 阈值,正常 age<3ms 不会误闪。
    // 注意:CAN 胶囊只反映真实 CAN 状态,手动阻断不应让前端误报 CAN 故障。
    this->can_feedback_live_.store(!can_safety, std::memory_order_relaxed);
    if (safety_hold_active)
    {
        const std::string cur_hold_state = this->fsm.GetCurrentStateName();
        const bool in_rl = (cur_hold_state == "RLFSMStateRLLocomotion" ||
                            cur_hold_state == "RLFSMStateDiffDrive");

        // 限频告警(每 2s 一次,避免日志刷屏;首次触发立即报)
        static auto last_safety_warn = std::chrono::steady_clock::time_point{};
        const auto now_warn = std::chrono::steady_clock::now();
        if (now_warn - last_safety_warn >= std::chrono::seconds(2))
        {
            last_safety_warn = now_warn;
            if (tilt_trigger)
            {
                fprintf(stderr, "[SAFETY] TILT GUARD: roll/pitch over limit in RL -> SafeHold NOW\n");
                this->Notify("error", "倾覆保护:姿态超限,已立即退出强化学习并原地保持(SafeHold),恢复后需人工重新进入");
            }
            else if (imu_trigger)
            {
                fprintf(stderr, "[SAFETY] IMU WATCHDOG: data stale/invalid in RL -> SafeHold NOW\n");
                this->Notify("error", "IMU 数据断供或异常,已立即退出强化学习并原地保持(SafeHold),恢复后需人工重新进入");
            }
            else if (nan_trigger)
            {
                fprintf(stderr, "[SAFETY] RL ACTION NaN streak -> SafeHold NOW\n");
                this->Notify("error", "策略输出连续异常(NaN),已立即退出强化学习并原地保持(SafeHold),恢复后需人工重新进入");
            }
            else if (manual_block)
            {
                // 手动阻断只在命令入口弹一次 toast(rl_block 处理处已 Notify
                // "强化学习输出已阻断")。此处若重复 Notify,阻断状态需人工 getup
                // 才解除,期间前端每 2s 弹一次"手动阻断已触发"弹窗,纯骚扰——
                // 降级为只写 journald 日志;持续状态由姿态胶囊(SafeHold)可见。
                fprintf(stderr, "[SAFETY] MANUAL RL BLOCK: force exit -> SafeHold NOW\n");
            }
            else if (in_rl)
            {
                fprintf(stderr, "[SAFETY] CAN feedback stale in RL: exit locomotion -> SafeHold NOW\n");
                this->Notify("error", "CAN 反馈超时，已立即退出强化学习并原地保持(SafeHold)，恢复后需人工重新进入");
            }
            else
            {
                fprintf(stderr, "[SAFETY] CAN feedback stale, holding current position (SafetyHold)\n");
            }
        }

        // 标记进行中的 Web 行为命令为 FAILED(原因: safety hold active)
        {
            std::lock_guard<std::mutex> lk(this->pending_cmd_mutex_);
            if (this->pending_cmd_)
            {
                this->EmitCommandState(this->pending_cmd_->seq, this->pending_cmd_->cmd,
                                       kFailed, this->pending_cmd_->target_state, 0);
                this->pending_cmd_.reset();
            }
        }

        if (in_rl)
        {
            // RL 态:断线当帧立即退出,不等反馈恢复。
            // RequestStateChange 置 CHANGE 标志后不 return,放行到 StateController:
            // fsm.Run() 走 CHANGE 分支——RLLocomotion.Exit()(清 rl_init_done,loop_rl
            // 推理线程下一帧起跳过 Forward/ComputeOutput/PublishAction,不产出任何动作)
            // → SafeHold.Enter()(抓取当前关节角) → SafeHold.Run()
            // (写 q=当前位/dq=0/fixed PD 保持帧)。本帧 SetCommand 发出的就是 SafeHold
            // 保持帧,全程无一帧 RL 策略输出。反馈恢复后停留在 SafeHold,需人工重进 RL。
            this->fsm.RequestStateChange("RLFSMStateSafeHold");
            // fall through:继续走下方 StateController/SetCommand 完成当帧切换
        }
        else
        {
            // 非 RL 态(Passive/GetUp/GetDown/SafeHold):维持拦截——
            // 发当前位 PD 保持帧,跳过 StateController,直到反馈恢复。
            // 覆盖指令: q=当前实际位置, dq=0, kp/kd=fixed, tau=0
            auto all_pos = motor_manager.getAllMotorPos();  // Eigen::VectorXf(16)
            auto fkp = this->params.Get<std::vector<float>>("fixed_kp");
            auto fkd = this->params.Get<std::vector<float>>("fixed_kd");
            if (fkp.size() != 16) fkp.assign(16, 0.0f);
            if (fkd.size() != 16) fkd.assign(16, 1.0f);

            // 零分配静态缓冲区(200Hz 热路径禁止堆分配)
            static std::vector<float> tau_z(16, 0.0f), vel_z(16, 0.0f);
            static std::vector<float> pos_buf(16);
            for (int i = 0; i < 16; ++i) pos_buf[i] = all_pos[i];

            motor_manager.sendAllMotorsCommand(tau_z, vel_z, pos_buf, fkp, fkd);

#if defined(USE_ROS2_REAL_CONTROL) || JXG_WEB_LAYER
            // 拦截期间对外广播不能停摆:posture 停发会使前端冻结在断线前画面;
            // 更关键的是 motor_state/imu_state 也必须照常——只断一路 CAN 时,
            // 其余 3 路 12 台电机与 IMU 仍在实时刷新,整页冻结会误导用户以为全死。
            // 断线 4 台冻结在最后真实值,feedback_age_ms 字段透传其数据年龄供看板标红。
            static int sh_telemetry_cnt = 0;
            if (++sh_telemetry_cnt >= 25) {
                sh_telemetry_cnt = 0;
                this->PublishTelemetryBundle();
            }
#endif
            return;  // 跳过 StateController/SetCommand/HR log(遥测已在上方补发)
        }
    }

    {
        ProfGuard _pg(2);
        this->StateController(&this->robot_state, &this->robot_command);
        this->control.ClearInput();
        this->UpdatePendingCommand();  // Web 行为命令生命周期: 到位发 DONE / 超时发 FAILED
    }
    {
        ProfGuard _pg(3);
        this->SetCommand(&this->robot_command);
    }

#if defined(USE_ROS2_REAL_CONTROL) || JXG_WEB_LAYER
    { ProfGuard _pg(4);
    const double dt = static_cast<double>(this->params.Get<float>("dt"));
    this->odometry_yaw += static_cast<double>(this->control.yaw) * dt;
    const double cos_yaw = std::cos(this->odometry_yaw);
    const double sin_yaw = std::sin(this->odometry_yaw);
    this->velocity_x = cos_yaw * this->control.x - sin_yaw * this->control.y;
    this->velocity_y = sin_yaw * this->control.x + cos_yaw * this->control.y;
    this->velocity_z = 0.0;
    this->odometry_x += this->velocity_x * dt;
    this->odometry_y += this->velocity_y * dt;
    // Telemetry: publish motor and IMU state at ~8Hz when the control loop is 200Hz.
    // (ROS 模式走话题,裸机模式经 web_bridge WebSocket 推送)
    // 与 SafetyHold 拦截分支共用 PublishTelemetryBundle,保证断线拦截期间
    // 其余在线电机/IMU 数据不冻结。
    static int ros_telemetry_cnt = 0;
    if (++ros_telemetry_cnt >= 25) {
        ros_telemetry_cnt = 0;
        // 注: can_feedback_live_ 已移至 SafetyHold 拦截前每帧刷新(100ms),
        // 保证断线拦截期间前端仍能收到 can_ready=false。
        this->PublishTelemetryBundle();
    }
    } // end S4(遥测)
#endif

#ifdef ENABLE_HR_LOGGER
    { ProfGuard _pg(5);
    this->HrLogPush();
    }
#endif

    // 常驻诊断:每秒(200周期)覆盖写一次分段归因
    if (++g_lprof.cyc >= 200) {
        g_lprof.cyc = 0;
        std::string st = this->fsm.GetCurrentStateName();
        if (st.empty()) st = "?";
        char b[1024];
        int n = snprintf(b, sizeof(b),
            "state=%s\n"
            "S0_GetState      usr=%7ldus sys=%7ldus calls=%ld\n"
            "S1_safety_checks usr=%7ldus sys=%7ldus calls=%ld\n"
            "S2_StateCtrl/FSM usr=%7ldus sys=%7ldus calls=%ld\n"
            "S3_SetCommand    usr=%7ldus sys=%7ldus calls=%ld\n"
            "S4_telemetry     usr=%7ldus sys=%7ldus calls=%ld\n"
            "S5_HrLog         usr=%7ldus sys=%7ldus calls=%ld\n",
            st.c_str(),
            g_lprof.u[0],g_lprof.s[0],g_lprof.calls[0],
            g_lprof.u[1],g_lprof.s[1],g_lprof.calls[1],
            g_lprof.u[2],g_lprof.s[2],g_lprof.calls[2],
            g_lprof.u[3],g_lprof.s[3],g_lprof.calls[3],
            g_lprof.u[4],g_lprof.s[4],g_lprof.calls[4],
            g_lprof.u[5],g_lprof.s[5],g_lprof.calls[5]);
        int fd = ::open("/tmp/rl_prof.log", O_WRONLY|O_CREAT|O_TRUNC, 0666);
        if (fd >= 0) { ::write(fd, b, n); ::close(fd); ::chmod("/tmp/rl_prof.log",0666); }
        for (int i = 0; i < LoopProf::N; ++i) { g_lprof.u[i]=0; g_lprof.s[i]=0; g_lprof.calls[i]=0; }
    }
}


void RL_Real::RunModel()
{
    if (this->rl_init_done)
    {
        this->episode_length_buf += 1;
        this->obs.ang_vel = this->robot_state.imu.gyroscope;
        if (this->control.navigation_mode)
        {
#if JXG_WEB_LAYER
            // v4: Web 层开启(含与 ROS 共存)时的统一速度路径。
            // ROS /rl_real/cmd_vel 回调与 Web cmd_vel 回调都会写 control.x/y/yaw,
            // 故此处只认 control,超时归零也只执行一份,避免共存时双重超时/源覆盖。
            // 命令超时保护：上位机断开后，若超过阈值未收到新 cmd_vel，
            // 立即归零速度，避免机器狗保持最后一条指令持续前冲。
            const double dt_cmd_web = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - this->last_cmd_vel_time_).count();
            if (dt_cmd_web > kCmdVelTimeoutSec)
            {
                if (this->control.x != 0.0f || this->control.y != 0.0f || this->control.yaw != 0.0f)
                {
                    this->control.x = 0.0f;
                    this->control.y = 0.0f;
                    this->control.yaw = 0.0f;
                    std::cout << LOGGER::WARNING << "cmd_vel timeout (" << kCmdVelTimeoutSec
                              << "s), velocity zeroed" << std::endl;
                    PublishRosFeedback("cmd_vel timeout: velocity zeroed");
                }
            }
            this->obs.commands = {this->control.x, this->control.y, this->control.yaw};
#elif defined(USE_ROS2_REAL_CONTROL) || (!defined(USE_CMAKE) && defined(USE_ROS))
            // 纯 ROS 构建(无 Web 层): 命令超时保护。上位机(rosbridge)断开后，
            // 若超过阈值未收到新 cmd_vel，立即归零速度，避免保持最后速度指令持续前冲。
            const double dt_cmd = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - this->last_cmd_vel_time_).count();
            if (dt_cmd > kCmdVelTimeoutSec)
            {
                if (this->cmd_vel.linear.x != 0.0f || this->cmd_vel.linear.y != 0.0f || this->cmd_vel.angular.z != 0.0f)
                {
                    this->cmd_vel = geometry_msgs::msg::Twist();
                    std::cout << LOGGER::WARNING << "cmd_vel timeout (" << kCmdVelTimeoutSec
                              << "s), velocity zeroed" << std::endl;
                    PublishRosFeedback("cmd_vel timeout: velocity zeroed");
                }
            }
            this->obs.commands = {(float)this->cmd_vel.linear.x, (float)this->cmd_vel.linear.y, (float)this->cmd_vel.angular.z};
#else
            this->obs.commands = {this->control.x, this->control.y, this->control.yaw};
#endif
        }
        else
        {
            this->obs.commands = {this->control.x, this->control.y, this->control.yaw};
        }
        this->obs.base_quat = this->robot_state.imu.quaternion;
        this->obs.dof_pos = this->robot_state.motor_state.q;
        this->obs.dof_vel = this->robot_state.motor_state.dq;

        this->obs.actions = this->Forward();
        this->ComputeOutput(this->obs.actions, this->output_dof_pos, this->output_dof_vel, this->output_dof_tau);

        // 热修改 2(2026-10-02): 轮速目标一阶低通滤波,抑制策略输出的高频轮速抖动。
        // 只滤轮子(wheel_indices, wheel 模式 JXGw 下为 {3,7,11,15};非 wheel 模式为空,自然跳过)。
        // 首次运行/向量尺寸变化/推理中断超过 0.1s(策略退出后重入)时,状态作废并首拍直跟踪,
        // 不用陈旧值做插值,避免起步瞬间轮速突变。
        //
        // 热修改 3(2026-10-11): cmd=0 门限——三轴指令全部低于死区(输入端
        // ApplyDeadzone 已归零,此处按 kCmdVelDeadzone 复核,兜住绕过输入死区的
        // 指令链路)时,把策略输出的轮速目标强制清零,防止模型在零指令下仍输出
        // 爬行轮速。清零在低通之前(raw=0),轮速经 LPF 平滑收敛到 0,无阶跃。
        {
            const std::vector<int> &wheel_idx = this->params.Get<std::vector<int>>("wheel_indices");
            const auto now_t = std::chrono::steady_clock::now();
            bool reinit = (wheel_vel_lpf_.size() != output_dof_vel.size());
            if (!reinit && wheel_vel_lpf_.size() == output_dof_vel.size() &&
                !wheel_idx.empty())
            {
                const double gap = std::chrono::duration<double>(now_t - last_runmodel_time_).count();
                if (gap > kWheelLpfResetGapSec) reinit = true;
            }
            bool cmd_all_zero = false;
            if (!wheel_idx.empty())
            {
                const std::vector<float> &c = this->obs.commands;
                cmd_all_zero = c.size() >= 3 &&
                               std::abs(c[0]) < kCmdVelDeadzone &&
                               std::abs(c[1]) < kCmdVelDeadzone &&
                               std::abs(c[2]) < kCmdVelDeadzone;
            }
            if (reinit)
            {
                wheel_vel_lpf_ = output_dof_vel;  // 首拍直接跟踪原始目标
                if (cmd_all_zero)
                {
                    for (int i : wheel_idx)
                        if (i >= 0 && (size_t)i < wheel_vel_lpf_.size()) wheel_vel_lpf_[i] = 0.0f;
                }
            }
            else
            {
                for (int i : wheel_idx)
                {
                    if (i < 0 || (size_t)i >= output_dof_vel.size()) continue;
                    const float raw = cmd_all_zero ? 0.0f : output_dof_vel[i];
                    wheel_vel_lpf_[i] = kWheelVelLpfAlpha * raw +
                                        (1.0f - kWheelVelLpfAlpha) * wheel_vel_lpf_[i];
                    output_dof_vel[i] = wheel_vel_lpf_[i];
                }
            }
            last_runmodel_time_ = now_t;
        }

        // 发布最新一帧动作到无锁三缓冲(替代原 3 个 TBB 无界队列 push)。
        this->PublishAction();

        // this->TorqueProtect(this->output_dof_tau);
        // this->AttitudeProtect(this->robot_state.imu.quaternion, 75.0f, 75.0f);

#ifdef CSV_LOGGER
        std::vector<float> tau_est = this->robot_state.motor_state.tau_est;
        this->CSVLogger(this->output_dof_tau, tau_est, this->obs.dof_pos, this->output_dof_pos, this->obs.dof_vel);
#endif

#ifdef ENABLE_ASYNC_CSV_LOGGER
        static int csv_collect_count = 0;
        if (++csv_collect_count >= 10)
        {
            csv_collect_count = 0;

            if (csv_logger && this->output_dof_pos.size() >= (size_t)this->params.Get<int>("num_of_dofs"))
            {
                double current_time = std::chrono::duration<double>(
                    std::chrono::steady_clock::now().time_since_epoch()).count();
                double timestamp = current_time - csv_logger_start_time;

                int num_dofs = this->params.Get<int>("num_of_dofs");
                std::vector<double> target_tau(num_dofs);
                std::vector<double> actual_tau(num_dofs);
                std::vector<double> target_pos(num_dofs);
                std::vector<double> actual_pos(num_dofs);
                std::vector<double> target_vel(num_dofs);
                std::vector<double> actual_vel(num_dofs);

                for (int i = 0; i < num_dofs; ++i)
                {
                    target_tau[i] = (this->output_dof_tau.size() >= (size_t)num_dofs)
                                  ? this->output_dof_tau[i] : 0.0;
                    actual_tau[i] = this->robot_state.motor_state.tau_est[i];
                    target_pos[i] = this->output_dof_pos[i];
                    actual_pos[i] = this->robot_state.motor_state.q[i];
                    target_vel[i] = (this->output_dof_vel.size() >= (size_t)num_dofs)
                                  ? this->output_dof_vel[i] : 0.0;
                    actual_vel[i] = this->robot_state.motor_state.dq[i];
                }

                csv_logger->addData(timestamp, target_tau, actual_tau, target_pos, actual_pos, target_vel, actual_vel);
            }
        }
#endif
    }
}


// C3-fix: HR 日志模块已拆分至 rl_real_hr_log.cpp


std::vector<float> RL_Real::Forward()
{
    std::unique_lock<std::mutex> lock(this->model_mutex, std::try_to_lock);

    if (!lock.owns_lock())
    {
        std::cout << LOGGER::WARNING << "Model is being reinitialized, using previous actions" << std::endl;
        return this->obs.actions;
    }

    std::vector<float> clamped_obs = this->ComputeObservation();

    std::vector<float> actions;
    if (!this->params.Get<std::vector<int>>("observations_history").empty())
    {
        this->history_obs_buf.insert(clamped_obs);
        this->history_obs = this->history_obs_buf.get_obs_vec(this->params.Get<std::vector<int>>("observations_history"));
        actions = this->model->forward({this->history_obs});
    }
    else
    {
        actions = this->model->forward({clamped_obs});
    }

    // P1-B 输出 NaN 拦截:clamp 对 NaN 无效(比较恒 false), 坏帧直接进 PD 会产生
    // NaN 力矩目标。单帧坏帧 → 用上帧有效动作替换(首帧无历史则归零=default 姿态);
    // 连续 3 帧 → 置 nan_alert_, 由控制环并入 SafeHold 拦截组合强制退出 RL。
    // last_valid_actions_/计数仅本推理线程访问, 无竞争。
    {
        bool has_nan = false;
        for (float v : actions) { if (!std::isfinite(v)) { has_nan = true; break; } }
        if (has_nan)
        {
            const int streak = this->nan_bad_streak_.fetch_add(1, std::memory_order_relaxed) + 1;
            if (this->last_valid_actions_.size() == actions.size())
            {
                actions = this->last_valid_actions_;
            }
            else
            {
                actions.assign(actions.size(), 0.0f);
            }
            if (streak >= 3)
            {
                this->nan_alert_.store(true, std::memory_order_release);
                std::cout << LOGGER::ERROR << "RL action NaN streak=" << streak
                          << ", request SafeHold via control loop" << std::endl;
            }
        }
        else
        {
            this->nan_bad_streak_.store(0, std::memory_order_relaxed);
            this->last_valid_actions_ = actions;
        }
    }

    if (!this->params.Get<std::vector<float>>("clip_actions_upper").empty() && !this->params.Get<std::vector<float>>("clip_actions_lower").empty())
    {
        return clamp(actions, this->params.Get<std::vector<float>>("clip_actions_lower"), this->params.Get<std::vector<float>>("clip_actions_upper"));
    }
    else
    {
        return actions;
    }
}




// C3-fix: Web/ROS 命令模块已拆分至 rl_real_command.cpp


void RL_Real::PublishRosFeedback(const std::string &message)
{
#if JXG_ROS_LAYER
    if (this->feedback_publisher)
    {
        std_msgs::msg::String feedback;
        feedback.data = message;
        this->feedback_publisher->publish(feedback);
    }
#endif
#if JXG_WEB_LAYER
    // v4 共存: 同一条反馈同时发 ROS topic(/rl_real/feedback)与 WebSocket
    if (this->web_bridge_) this->web_bridge_->publishFeedback(message);
#endif
}

// C3-fix: 通知系统模块已拆分至 rl_real_notify.cpp

// C3-fix: 姿态检测模块已拆分至 rl_real_posture.cpp

// C3-fix: 安全起立模块已拆分至 rl_real_safe_getup.cpp

// C3-fix: 遥测发布模块已拆分至 rl_real_telemetry.cpp

void RL_Real::Plot()
{
    this->plot_t.erase(this->plot_t.begin());
    this->plot_t.push_back(this->motiontime);
    plt::cla();
    plt::clf();
    for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
    {
        this->plot_real_joint_pos[i].erase(this->plot_real_joint_pos[i].begin());
        this->plot_target_joint_pos[i].erase(this->plot_target_joint_pos[i].begin());
        {
            // 跨线程保护:plot 线程读 JXG_state 需与控制环写互斥
            std::lock_guard<std::mutex> lk(this->jxg_state_mutex_);
            this->plot_real_joint_pos[i].push_back(this->JXG_state.motor_state[i].q);
        }
        this->plot_target_joint_pos[i].push_back(this->JXG_cmd.motor_cmd[i].q);
        plt::subplot(this->params.Get<int>("num_of_dofs"), 1, i + 1);
        plt::named_plot("_real_joint_pos", this->plot_t, this->plot_real_joint_pos[i], "r");
        plt::named_plot("_target_joint_pos", this->plot_t, this->plot_target_joint_pos[i], "b");
        plt::xlim(this->plot_t.front(), this->plot_t.back());
    }
    // plt::legend();
    plt::pause(0.0001);
}

void RL_Real::InitLowCmd()
{
    std::vector<float> default_q = {
        0.0f,  0.8f, -1.6f,
        0.0f,  0.8f, -1.6f,
        0.0f,  0.8f, -1.6f,
        0.0f,  0.8f, -1.6f
    };

    for (int i = 0; i < 16; ++i) {
        this->JXG_cmd.motor_cmd[i].q    = (i < 12) ? default_q[i] : 0.0f;
        this->JXG_cmd.motor_cmd[i].dq   = 0.0f;
        this->JXG_cmd.motor_cmd[i].kp   = 0.0f;
        this->JXG_cmd.motor_cmd[i].kd   = 20.0f;
        this->JXG_cmd.motor_cmd[i].tau  = 0.0f;
    }
}

// C3-fix: 安全检查模块已拆分至 rl_real_safety.cpp

#ifdef __linux__
// ── CPU affinity 巡检线程 ──
// 兜底机制:定期扫描 /proc/self/task,把非实时(SCHED_OTHER)且 affinity
// 不在小核 0-3 的线程拉回小核。防止第三方库(如 DDS/ONNX/TBB)在运行时
// 动态创建线程并自行设置 affinity 飘到大核,与实时控制环争用。
//
// 设计约束:
//   - 普通调度 SCHED_OTHER + nice +10,绝不抢占实时线程
//   - 自身绑小核 0-3
//   - 每 2s 扫描一次,30 个线程的遍历开销 < 50us
//   - 实时线程(FIFO/RR)跳过,由 loop.hpp/usb2can.cpp 自行绑大核
static void affinityWatchdog()
{
    // 自身降权 + 绑小核
    nice(10);
    cpu_set_t little_cpus;
    CPU_ZERO(&little_cpus);
    for (int c = 0; c < 4; ++c) CPU_SET(c, &little_cpus);
    sched_setaffinity(0, sizeof(little_cpus), &little_cpus);

    const pid_t self_tid = static_cast<pid_t>(syscall(SYS_gettid));
    int total_fixed = 0;
    std::cout << "[affinity-watchdog] 启动, self_tid=" << self_tid
              << " nice=" << nice(0) << std::endl;

    while (!g_affinity_watchdog_exit.load()) {
        DIR *task_dir = opendir("/proc/self/task");
        if (!task_dir) {
            std::this_thread::sleep_for(std::chrono::seconds(2));
            continue;
        }

        int round_fixed = 0;
        struct dirent *ent;
        while ((ent = readdir(task_dir)) != nullptr) {
            if (ent->d_name[0] == '.') continue;
            const long tid = std::atol(ent->d_name);
            if (tid <= 0 || tid == self_tid) continue;

            // 实时线程(FIFO=1, RR=2)跳过:由 loop.hpp/usb2can.cpp 自行绑大核
            const int policy = sched_getscheduler(static_cast<pid_t>(tid));
            if (policy == SCHED_FIFO || policy == SCHED_RR) continue;

            // 检查当前 affinity,非 0-3 则纠正
            cpu_set_t cur;
            CPU_ZERO(&cur);
            if (sched_getaffinity(static_cast<pid_t>(tid), sizeof(cur), &cur) != 0) continue;

            bool need_fix = false;
            for (int c = 4; c < 8; ++c) {
                if (CPU_ISSET(c, &cur)) { need_fix = true; break; }
            }
            if (need_fix) {
                if (sched_setaffinity(static_cast<pid_t>(tid), sizeof(little_cpus), &little_cpus) == 0) {
                    ++round_fixed;
                }
            }
        }
        closedir(task_dir);

        if (round_fixed > 0) {
            total_fixed += round_fixed;
            std::cout << "[affinity-watchdog] 本轮纠正 " << round_fixed
                      << " 个线程回小核(累计 " << total_fixed << ")" << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::seconds(2));
    }
}
#endif

int main(int argc, char **argv)
{
    InstallCrashDumper();  // 必须最先安装:崩溃原因/backtrace 写 /tmp/rl_crash.log
    // 版本与构建时间(CMake 生成,编译进二进制)
    std::cout << LOGGER::INFO << "[version] rl_real_JXG v" << APP_VERSION
              << " build " << BUILD_NUMBER
              << " built " << BUILD_TIMESTAMP << std::endl;
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    // 锁定全部当前及未来内存页,避免实时线程运行期间发生缺页(含 swap/page fault)抖动。
    // 服务以 root 运行,默认具备 CAP_IPC_LOCK;非 root 无权限时仅告警,不影响启动。
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
        std::cerr << "[main] WARN: mlockall 失败: " << std::strerror(errno)
                  << "(实时线程仍可能遇到缺页抖动;以 root 运行可消除)" << std::endl;
    }

#ifdef __linux__
    // ── RK3576 大小核 CPU 布局(与 systemd 单元 CPUAffinity=0-7 配合) ──
    // 主线程在此把自身亲和掩码设为 A53 小核簇 0-3。之后所有未显式绑核的普通
    // SCHED_OTHER 线程(rclcpp/FastDDS、WebBridge HTTP+WS、ONNX Runtime worker、
    // main 的 spin)在创建时继承该掩码,全部落在小核。
    // 硬实时线程不受影响:LoopFunc 对 cpu>=0 的环(imu=4、control/rl=6)和
    // USB2CAN 的 8 个 CAN 线程(5/7)在 start 后自行 pthread_setaffinity_np 绑大核。
    // 收益:DDS/Web 突发流量不再与 200Hz 控制环/50Hz 推理争用 CPU6 大核,
    // 也不污染其 L2;4 个 A53 小核从此参与负载(此前被 systemd CPUAffinity=4-7 闲置)。
    {
        cpu_set_t little_cpus;
        CPU_ZERO(&little_cpus);
        for (int c = 0; c < 4; ++c) CPU_SET(c, &little_cpus);
        if (sched_setaffinity(0, sizeof(little_cpus), &little_cpus) != 0) {
            std::cerr << "[main] WARN: 主线程绑小核(0-3)失败: " << std::strerror(errno)
                      << "(普通线程可能仍散落到大核)" << std::endl;
        } else {
            std::cout << "[main] 主线程(及未显式绑核的普通线程)亲和掩码=CPU0-3 (A53 小核簇);"
                         " RT 环/CAN 线程自行绑 CPU4-7 (A72 大核簇)" << std::endl;
        }

        // 纠正在此之前已创建的线程(如库的全局初始化线程)的 affinity。
        // sched_setaffinity(0,...) 仅影响调用线程,已存在的线程仍保留旧掩码(0-7),
        // 会飘到大核与 RT 线程争用。遍历 /proc/self/task 把所有现存线程统一拉回小核。
        // 实时线程(imu/CAN/control/rl)此时尚未创建,不受影响。
        DIR *task_dir = opendir("/proc/self/task");
        if (task_dir) {
            pid_t self = getpid();
            struct dirent *ent;
            int fixed = 0;
            while ((ent = readdir(task_dir)) != nullptr) {
                if (ent->d_name[0] == '.') continue;
                long tid = std::atol(ent->d_name);
                if (tid <= 0 || tid == self) continue;
                if (sched_setaffinity(tid, sizeof(little_cpus), &little_cpus) == 0) {
                    ++fixed;
                }
            }
            closedir(task_dir);
            if (fixed > 0) {
                std::cout << "[main] 已将 " << fixed << " 个预存线程亲和掩码纠正为 CPU0-3"
                          << std::endl;
            }
        }
    }
#endif

#ifdef __linux__
    // 启动 affinity 巡检线程:兜底纠正运行时被第三方库改到大核的非实时线程
    std::thread affinity_watchdog_thread(affinityWatchdog);
#endif

    // v5: 无启动参数,固定执行 JXGw;Web 层为独立 web_server 进程,
    // 主程序不再接受 wheel/--web-port/--web-root 等参数
    WebConfig web_cfg;

#if defined(USE_ROS1) && defined(USE_ROS)
    ros::init(argc, argv, "rl_JXG");
      RL_Real rl_real(web_cfg);

      // 使用 ros::ok() 检查退出
      while (ros::ok() && !g_shutdown_flag.load()) {
          ros::spinOnce();
          std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }

      // 手动关闭 ROS
      ros::shutdown();

#elif defined(USE_ROS2_REAL_CONTROL) || (defined(USE_ROS2) && defined(USE_ROS))
    rclcpp::init(argc, argv);
      // v4: ROS2 分支同样传入 web_cfg; JXG_WEB_LAYER=1 时构造函数内会启动
      // WebBridge(:8088), 与 rclcpp 同进程共存 —— spin_some 驱动 ROS 话题,
      // WebBridge 内部自有线程驱动 HTTP/WebSocket。
      auto rl_node = std::make_shared<RL_Real>(web_cfg);
      std::cout << "RL_Real JXGw started (ROS2"
#if JXG_WEB_LAYER
                << " + WebSocket"
#endif
                << ")" << std::endl;

      // 使用 while 循环替代 spin;rclcpp::spin_some(node) 已弃用,
      // 改用 SingleThreadedExecutor::spin_some()
      rclcpp::executors::SingleThreadedExecutor executor;
      executor.add_node(rl_node);
      while (rclcpp::ok() && !g_shutdown_flag.load()) {
          executor.spin_some();
          std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }

      rclcpp::shutdown();

#else
    RL_Real rl_real(web_cfg);
    std::cout << "RL_Real JXGw started" << std::endl;
    // while (1) std::this_thread::sleep_for(std::chrono::seconds(10));
    while (!g_shutdown_flag.load()) {
      std::this_thread::sleep_for(std::chrono::seconds(10));
  }

#endif
std::cout << "Program exiting..." << std::endl;

#ifdef __linux__
    // 停止 affinity 巡检线程
    g_affinity_watchdog_exit.store(true);
    if (affinity_watchdog_thread.joinable()) {
        affinity_watchdog_thread.join();
    }
#endif

return 0;
}