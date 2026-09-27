#include "rl_real_JXG.hpp"
#include "motor_manager.h"
#include <csignal>
#include <atomic>
#include <functional>
#include <string>
#include <sstream>
#include <cctype>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <iomanip>  // For std::setw, std::fixed, std::setprecision
#include <queue>
#include <mutex>
#include <fstream>
#include <ctime>
#include <sys/mman.h>
#include <cstring>

RobotMotorManager motor_manager;

std::atomic<bool> g_shutdown_flag(false);

// 上位机 cmd_vel 命令超时保护：超过该时长未收到新的 cmd_vel，自动归零速度，
// 防止上位机(Web 调试台 websocket)断开后机器狗保持最后一条速度指令持续前冲。
// 上位机若以低于 2Hz 发送 cmd_vel，请适当调大该值。
static constexpr double kCmdVelTimeoutSec = 0.5;

// steady_clock 秒（用于 cmd_vel 超时与通知重试节拍）
static double SteadyNowSec()
{
    return std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

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


RL_Real::RL_Real(bool wheel_mode, const WebConfig &web_cfg)
{
    this->ang_vel_axis = "body";
    this->robot_name = wheel_mode ? "JXGw" : "JXG";
    this->ReadYaml(this->robot_name, "base.yaml");
    this->InitJointNum(this->params.Get<int>("num_of_dofs"));
    this->CacheHotParams();

    // FSM
    if (FSMManager::GetInstance().IsTypeSupported(this->robot_name))
    {
        auto fsm_ptr = FSMManager::GetInstance().CreateFSM(this->robot_name, this);
        if (fsm_ptr) this->fsm = *fsm_ptr;
    }

    // 硬件初始化
    motor_manager.init();
   
    motor_manager.startThreads(); 

    motor_manager.initAllMotors();

    // 一次性电机零位检查：等待反馈稳定后检测
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    this->motors_ready_for_stand = this->CheckMotorInitPosition(1.0f);  // 阈值1.0rad
    if (this->motors_ready_for_stand) {
        std::cout << "[INIT] Motor position check PASSED" << std::endl;
    } else {
        std::cout << "[INIT] Motor position check FAILED: " << this->motors_not_ready_reason << std::endl;
    }

    open_imu();

    this->loop_imu = std::make_shared<LoopFunc>("loop_imu", 0.001, []() {
        read_imu();  // 1kHz
    }, 4 /*CPU*/, 90 /*FIFO 最高:IMU 采样*/);
    this->loop_imu->start();
    this->InitLowCmd();
    std::vector<float> pos(16, 0.0f);
    std::vector<float> tau(16, 0.0f), vel(16, 0.0f), kp(16, 0.0f), kd(16, 20.0f);
    for (int i = 0; i < 16; ++i) {
        pos[i] = this->JXG_cmd.motor_cmd[i].q;
    }
    for (int i = 0; i < 100; ++i) {
        motor_manager.sendAllMotorsCommand(tau, vel, pos, kp, kd);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }


    this->InitOutputs();
    this->InitControl();

    // ===== 内置 Web 调试台桥（HTTP 静态页面 + WebSocket 控制/遥测，替代 ROS2 + rosbridge） =====
    // 场景音频播放（预设 wav + Web 触发，输出 USB 声卡 card3；音频目录不存在时优雅降级）
    {
        struct stat st;
        if (::stat(web_cfg.briefing_audio_dir.c_str(), &st) == 0 && S_ISDIR(st.st_mode)) {
            this->briefing_audio_ = std::make_unique<rl_briefing::BriefingAudioPlayer>(
                web_cfg.briefing_audio_dir, "plughw:3,0", 48000, 2);
        } else {
            std::cout << LOGGER::WARNING << "Briefing audio dir not found: "
                      << web_cfg.briefing_audio_dir << " (briefing playback disabled)" << std::endl;
        }
    }
    this->web_bridge_ = std::make_unique<WebBridge>();
    this->last_cmd_vel_time_ = std::chrono::steady_clock::now();
    if (!this->web_bridge_->start(
            web_cfg.web_port, web_cfg.web_root,
            // cmd_vel
            [this](const CmdVel &v) { this->CmdvelCallback(v); },
            // command（原 /rl_real/command 字符串）
            [this](const std::string &s) { this->ApplyCommand(s); },
            // imu_stream 开关
            [this](bool b) { this->ImuStreamCallback(b); },
            // notify_ack（原 JSON 字符串）
            [this](const std::string &s) { this->NotifyAckCallback(s); },
            // check_stand
            [this]() { this->CheckStandCallback(); },
            // briefing 播报
            [this](const std::string &s) { this->BriefingCallback(s); }))
    {
        std::cout << LOGGER::WARNING << "Web bridge failed to start (port "
                  << web_cfg.web_port << " in use?), web console disabled" << std::endl;
        this->web_bridge_.reset();
    }
    std::cout << LOGGER::INFO << "Web control enabled: cmd_vel, command, feedback, motor_state, imu_state, pose2d" << std::endl;
    // 初始零位检查结果上报（桥就绪后补发一次）
    if (this->motors_ready_for_stand) {
        this->Notify("info", "motor position check passed");
    } else {
        this->Notify("error", this->motors_not_ready_reason);
    }

    xbox_gamepad = std::make_unique<XboxGamepad>(0);
    loop_gamepad = std::make_shared<LoopFunc>("loop_gamepad", 0.005, [this]() {
        if (xbox_gamepad && xbox_gamepad->connected)
        {
            xbox_gamepad->update();
            last_gamepad_state = gamepad_state;
            gamepad_state = xbox_gamepad->state;


        //  static int cnt = 0;
        // if (++cnt % 40 == 0)
        //     {  // 每 200ms打印一次
        //         printf("Xbox connected | "
        //             "A:%d B:%d X:%d Y:%d "
        //             "LB:%d RB:%d L3:%d R3:%d Start:%d Back:%d "
        //             "LStick:%.2f,%.2f RStick:%.2f,%.2f "
        //             "DPad U:%d D:%d L:%d R:%d\n",
        //             gamepad_state.A, gamepad_state.B, gamepad_state.X, gamepad_state.Y,
        //             gamepad_state.LB, gamepad_state.RB, gamepad_state.L3, gamepad_state.R3,
        //             gamepad_state.start, gamepad_state.back,
        //             gamepad_state.left_x, gamepad_state.left_y,
        //             gamepad_state.right_x, gamepad_state.right_y,
        //             gamepad_state.up, gamepad_state.down, gamepad_state.left, gamepad_state.right
        //         );
        //     }
        }
        else {
            static int cnt2 = 0;
            if (++cnt2 % 200 == 0) printf("Xbox NOT connected!\n");
        }
    }, -1 /*不绑核*/, -1 /*手柄输入非硬实时:SCHED_OTHER(nice 继承服务 -10),可被全部 FIFO 线程抢占*/);
    loop_gamepad->start();

    // 键盘线程已禁用，使用Xbox手柄控制
    // this->loop_keyboard = std::make_shared<LoopFunc>("loop_keyboard", 0.05, std::bind(&RL_Real::KeyboardInterface, this));
    this->loop_control  = std::make_shared<LoopFunc>("loop_control",  this->params.Get<float>("dt"), std::bind(&RL_Real::RobotControl, this),
                                                     4 /*CPU:与 loop_imu 同核,共享状态 cache*/, 80 /*FIFO:200Hz 控制环*/);
    this->loop_rl       = std::make_shared<LoopFunc>("loop_rl",       this->params.Get<float>("dt") * this->params.Get<int>("decimation"), std::bind(&RL_Real::RunModel, this),
                                                     6 /*CPU:推理独占核*/, 70 /*FIFO:50Hz ONNX 推理*/);
    // this->loop_keyboard->start();
    this->loop_control->start();
    this->loop_rl->start();

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
    this->loop_gamepad->shutdown();
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
    updateJXGLowState(this->JXG_state);
    const auto& gp = gamepad_state;
    const bool gamepad_active =
        gp.A || gp.B || gp.X || gp.Y || gp.LB || gp.RB || gp.L3 || gp.R3 ||
        gp.back || gp.start || gp.up || gp.down || gp.left || gp.right ||
        std::abs(gp.left_x) > 0.05f || std::abs(gp.left_y) > 0.05f ||
        std::abs(gp.right_x) > 0.05f || std::abs(gp.right_y) > 0.05f;

    this->gamepad_override_active = gamepad_active;
    if (gamepad_active)
    {
        if (gp.A)      this->control.SetGamepad(Input::Gamepad::A);
        if (gp.B)      this->control.SetGamepad(Input::Gamepad::B);
        if (gp.X)      this->control.SetGamepad(Input::Gamepad::X);
        if (gp.Y)      this->control.SetGamepad(Input::Gamepad::Y);
        if (gp.LB)     this->control.SetGamepad(Input::Gamepad::LB);
        if (gp.RB)     this->control.SetGamepad(Input::Gamepad::RB);
        if (gp.L3)     this->control.SetGamepad(Input::Gamepad::LStick);
        if (gp.R3)     this->control.SetGamepad(Input::Gamepad::RStick);
        if (gp.up)     this->control.SetGamepad(Input::Gamepad::DPadUp);
        if (gp.down)   this->control.SetGamepad(Input::Gamepad::DPadDown);
        if (gp.left)   this->control.SetGamepad(Input::Gamepad::DPadLeft);
        if (gp.right)  this->control.SetGamepad(Input::Gamepad::DPadRight);

        if (gp.LB && gp.A) this->control.SetGamepad(Input::Gamepad::LB_A);
        if (gp.LB && gp.B) this->control.SetGamepad(Input::Gamepad::LB_B);
        if (gp.LB && gp.X) this->control.SetGamepad(Input::Gamepad::LB_X);
        if (gp.LB && gp.Y) this->control.SetGamepad(Input::Gamepad::LB_Y);
        if (gp.LB && gp.L3) this->control.SetGamepad(Input::Gamepad::LB_LStick);
        if (gp.LB && gp.R3) this->control.SetGamepad(Input::Gamepad::LB_RStick);
        if (gp.LB && gp.down) this->control.SetGamepad(Input::Gamepad::LB_DPadDown);
        if (gp.LB && gp.left) this->control.SetGamepad(Input::Gamepad::LB_DPadLeft);
        if (gp.LB && gp.right) this->control.SetGamepad(Input::Gamepad::LB_DPadRight);
        if (gp.RB && gp.A) this->control.SetGamepad(Input::Gamepad::RB_A);
        if (gp.RB && gp.B) this->control.SetGamepad(Input::Gamepad::RB_B);
        if (gp.RB && gp.X) this->control.SetGamepad(Input::Gamepad::RB_X);
        if (gp.RB && gp.Y) this->control.SetGamepad(Input::Gamepad::RB_Y);
        if (gp.RB && gp.L3) this->control.SetGamepad(Input::Gamepad::RB_LStick);
        if (gp.RB && gp.R3) this->control.SetGamepad(Input::Gamepad::RB_RStick);
        if (gp.RB && gp.up) this->control.SetGamepad(Input::Gamepad::RB_DPadUp);
        if (gp.RB && gp.down) this->control.SetGamepad(Input::Gamepad::RB_DPadDown);
        if (gp.RB && gp.left) this->control.SetGamepad(Input::Gamepad::RB_DPadLeft);
        if (gp.RB && gp.right) this->control.SetGamepad(Input::Gamepad::RB_DPadRight);
        if (gp.LB && gp.RB) this->control.SetGamepad(Input::Gamepad::LB_RB);

        this->control.x   = -gp.left_y;
        this->control.y   = gp.left_x;
        this->control.yaw = -gp.right_x;
    }

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

    for (int i = 0; i < cfg_num_dofs_; ++i) {
        int idx = cfg_joint_mapping_[i];
        state->motor_state.q[i]       = this->JXG_state.motor_state[idx].q + cfg_lying_down_offset_[i];
        state->motor_state.dq[i]      = this->JXG_state.motor_state[idx].dq;
        state->motor_state.tau_est[i] = this->JXG_state.motor_state[idx].tau_est;
    }
}

void RL_Real::CacheHotParams()
{
    cfg_num_dofs_          = params.Get<int>("num_of_dofs");
    cfg_dt_                = params.Get<float>("dt");
    cfg_decimation_        = params.Get<int>("decimation");
    cfg_joint_mapping_     = params.Get<std::vector<int>>("joint_mapping");
    cfg_lying_down_offset_ = params.Get<std::vector<float>>("lying_down_offset");
    cfg_observations_history_ = params.Get<std::vector<int>>("observations_history");
    cfg_clip_actions_upper_   = params.Get<std::vector<float>>("clip_actions_upper");
    cfg_clip_actions_lower_   = params.Get<std::vector<float>>("clip_actions_lower");
    cfg_has_clip_ = !cfg_clip_actions_upper_.empty() && !cfg_clip_actions_lower_.empty();

    cmd_tau_.resize(16); cmd_vel_.resize(16); cmd_pos_.resize(16);
    cmd_kp_.resize(16);  cmd_kd_.resize(16);
}

void RL_Real::SetCommand(const RobotCommand<float> *command)
{
    for (int i = 0; i < cfg_num_dofs_; ++i) {
        int idx = cfg_joint_mapping_[i];
        cmd_pos_[idx] = command->motor_command.q[i] - cfg_lying_down_offset_[i];
        cmd_vel_[idx] = command->motor_command.dq[i];
        cmd_kp_[idx]  = command->motor_command.kp[i];
        cmd_kd_[idx]  = command->motor_command.kd[i];
        cmd_tau_[idx] = command->motor_command.tau[i];
    }
    motor_manager.sendAllMotorsCommand(cmd_tau_, cmd_vel_, cmd_pos_, cmd_kp_, cmd_kd_);
}

void RL_Real::RobotControl()
{
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
    if (this->control.current_keyboard == Input::Keyboard::N || this->control.current_gamepad == Input::Gamepad::X)
    {
        this->control.navigation_mode = !this->control.navigation_mode;
        std::cout << std::endl << LOGGER::INFO << "Navigation mode: " << (this->control.navigation_mode ? "ON" : "OFF") << std::endl;
        this->control.current_keyboard = this->control.last_keyboard;
        this->control.current_gamepad = this->control.last_gamepad;
    }

    // updateJXGLowState(this->JXG_state);
    this->GetState(&this->robot_state);

    // IMU 连续检查：陀螺仪全零说明IMU无数据，自动切入落下模式
    if (this->JXG_state.imu_state.gyroscope[0] == 0.0 &&
        this->JXG_state.imu_state.gyroscope[1] == 0.0 &&
        this->JXG_state.imu_state.gyroscope[2] == 0.0)
    {
        std::string cur_state = this->fsm.current_state_->GetStateName();
        if (cur_state != "RLFSMStatePassive")
        {
            static int imu_warn_cnt = 0;
            if (++imu_warn_cnt % 100 == 1)  // ~0.2s打印一次，避免刷屏
            {
                std::cout << LOGGER::WARNING << "IMU data lost! Forcing GetDown..." << std::endl;
                this->Notify("warning", "IMU data lost! Forcing GetDown...");
            }
            this->fsm.RequestStateChange("RLFSMStateGetDown");
        }
    }

    // 手柄断连检查：
    static bool gamepad_was_connected = xbox_gamepad && xbox_gamepad->connected;
    if (xbox_gamepad)
    {
        if (xbox_gamepad->connected)
            gamepad_was_connected = true;
        else if (gamepad_was_connected)
        {
            std::string cur_state = this->fsm.current_state_->GetStateName();
            if (cur_state != "RLFSMStatePassive")
            {
                static int gamepad_warn_cnt = 0;
                if (++gamepad_warn_cnt % 100 == 1)
                {
                    std::cout << LOGGER::WARNING << "Gamepad disconnected! Forcing GetDown..." << std::endl;
                    this->Notify("warning", "Gamepad disconnected! Forcing GetDown...");
                }
                this->fsm.RequestStateChange("RLFSMStateGetDown");
            }
        }
    }

    // 电机反馈连续检查：
    if (this->rl_init_done)
    {
        constexpr float dead_eps = 0.01f;
        constexpr int dead_cycles_limit = 10;  // 10×5ms=50ms，约1次RL推理窗口
        static const std::vector<int> wheel_indices = {3, 7, 11, 15};

        for (int i = 0; i < 16; ++i)
        {
            if (std::find(wheel_indices.begin(), wheel_indices.end(), i) != wheel_indices.end())
                continue;

            float dq  = this->JXG_state.motor_state[i].dq;
            float tau = this->JXG_state.motor_state[i].tau_est;

            if (std::abs(dq) < dead_eps && std::abs(tau) < dead_eps)
            {
                motor_dead_count[i]++;
                if (motor_dead_count[i] == dead_cycles_limit)  // 仅触发时打印一次
                {
                    std::string cur_state = this->fsm.current_state_->GetStateName();
                    if (cur_state != "RLFSMStatePassive" && cur_state != "RLFSMStateGetDown")
                    {
                        std::cout << LOGGER::WARNING << "Motor[" << i << "] feedback abnormal (vel&tau≈0 for 50ms)! Forcing GetDown..." << std::endl;
                        this->Notify("warning", "Motor[" + std::to_string(i) + "] feedback abnormal (vel&tau≈0 for 50ms)! Forcing GetDown...");
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

    // 检测手柄 LB+RB 组合键退出
    if (this->control.current_gamepad == Input::Gamepad::LB_RB) {
        std::cout << "\nGamepad LB+RB pressed. Exiting program..." << std::endl;
        g_shutdown_flag.store(true);
        return;
    }

    this->StateController(&this->robot_state, &this->robot_command);
    this->control.ClearInput();
    this->SetCommand(&this->robot_command);

    const double dt = static_cast<double>(cfg_dt_);
    this->odometry_yaw += static_cast<double>(this->control.yaw) * dt;
    const double cos_yaw = std::cos(this->odometry_yaw);
    const double sin_yaw = std::sin(this->odometry_yaw);
    this->velocity_x = cos_yaw * this->control.x - sin_yaw * this->control.y;
    this->velocity_y = sin_yaw * this->control.x + cos_yaw * this->control.y;
    this->velocity_z = 0.0;
    this->odometry_x += this->velocity_x * dt;
    this->odometry_y += this->velocity_y * dt;
    // Web telemetry: publish motor and IMU state at ~8Hz (200Hz/25).
    static int web_telemetry_cnt = 0;
    if (++web_telemetry_cnt >= 25) {
        web_telemetry_cnt = 0;
        auto all_pos = motor_manager.getAllMotorPos();
        auto all_vel = motor_manager.getAllMotorVel();
        auto all_tau = motor_manager.getAllMotorTorque();
        auto all_temp = motor_manager.getAllMotorTemp();

        this->PublishMotorState(cfg_num_dofs_, cfg_joint_mapping_, cfg_lying_down_offset_,
                                all_pos, all_vel, all_tau, all_temp);
        this->PublishImuState();
        this->PublishPose2D();
        this->ProcessNotifyRetries(SteadyNowSec());
    }

    // DEBUG: 每 100 次控制循环打印一次电机参数（~0.5s @ 200Hz）
    static int debug_cnt = 0;
    if (++debug_cnt >= 100) {
        debug_cnt = 0;
        auto all_pos = motor_manager.getAllMotorPos();
        auto all_vel = motor_manager.getAllMotorVel();
        auto all_tau = motor_manager.getAllMotorTorque();
        auto all_temp = motor_manager.getAllMotorTemp();

        std::cout << std::fixed << std::setprecision(3);
        std::cout << "\n[MOTOR] idx | target_pos  real_pos  | target_vel  real_vel  | target_tau  real_tau | kp     kd    temp" << std::endl;
        for (int i = 0; i < cfg_num_dofs_; ++i) {
            int idx = cfg_joint_mapping_[i];
            float tgt_pos = this->robot_command.motor_command.q[i];
            float tgt_vel = this->robot_command.motor_command.dq[i];
            float tgt_tau = this->robot_command.motor_command.tau[i];
            float kp = this->robot_command.motor_command.kp[i];
            float kd = this->robot_command.motor_command.kd[i];
            float real_pos = all_pos[idx] + cfg_lying_down_offset_[i];
            float real_vel = all_vel[idx];
            float real_tau = all_tau[idx];
            float real_temp = (idx < (int)all_temp.size()) ? all_temp[idx] : 0.0f;
            std::cout << "       " << std::setw(2) << idx
                      << " | " << std::setw(10) << tgt_pos << " " << std::setw(9) << real_pos
                      << " | " << std::setw(10) << tgt_vel << " " << std::setw(9) << real_vel
                      << " | " << std::setw(10) << tgt_tau << " " << std::setw(9) << real_tau
                      << " | " << std::setw(6) << kp << " " << std::setw(6) << kd
                      << " " << std::setw(6) << std::setprecision(1) << real_temp << "C" << std::setprecision(3)
                      << std::endl;
        }
    }

#ifdef ENABLE_HR_LOGGER
    this->HrLogPush();
#endif
}


void RL_Real::RunModel()
{
    if (this->rl_init_done)
    {
        this->episode_length_buf += 1;
        this->obs.ang_vel = this->robot_state.imu.gyroscope;
        if (this->control.navigation_mode)
        {
            // 命令超时保护：上位机(Web 调试台)断开后，若超过阈值未收到新 cmd_vel，
            // 立即归零速度，避免机器狗保持最后一条指令持续前冲。
            const double dt_cmd = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - this->last_cmd_vel_time_).count();
            if (dt_cmd > kCmdVelTimeoutSec)
            {
                if (this->cmd_vel.x != 0.0 || this->cmd_vel.y != 0.0 || this->cmd_vel.yaw != 0.0)
                {
                    this->cmd_vel = CmdVel();
                    std::cout << LOGGER::WARNING << "cmd_vel timeout (" << kCmdVelTimeoutSec
                              << "s), velocity zeroed" << std::endl;
                    PublishRosFeedback("cmd_vel timeout: velocity zeroed");
                }
            }
            this->obs.commands = {(float)this->cmd_vel.x, (float)this->cmd_vel.y, (float)this->cmd_vel.yaw};
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

        if (!this->output_dof_pos.empty())
        {
            output_dof_pos_queue.push(this->output_dof_pos);
        }
        if (!this->output_dof_vel.empty())
        {
            output_dof_vel_queue.push(this->output_dof_vel);
        }
        if (!this->output_dof_tau.empty())
        {
            output_dof_tau_queue.push(this->output_dof_tau);
        }

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


#ifdef ENABLE_HR_LOGGER
// ===== 高频(200Hz)记录器实现 =====
// 在 loop_control(200Hz) 末尾调用; 同步写入 1MB 用户缓冲, 每 ~10s flush 一次。
void RL_Real::HrLogInit()
{
    auto now = std::chrono::system_clock::now();
    std::time_t tt = std::chrono::system_clock::to_time_t(now);
    std::tm tmv = *std::localtime(&tt);
    char ts[32];
    std::strftime(ts, sizeof(ts), "%Y%m%d_%H%M%S", &tmv);

    char path[256];
    std::snprintf(path, sizeof(path), "logs/%s_hr_%s.csv", this->robot_name.c_str(), ts);

    static std::vector<char> hr_buf(1 << 20);  // 1MB 用户缓冲
    this->hr_file.rdbuf()->pubsetbuf(hr_buf.data(), static_cast<std::streamsize>(hr_buf.size()));
    this->hr_file.open(path, std::ios::out | std::ios::trunc);
    if (!this->hr_file.is_open())
    {
        std::cerr << LOGGER::WARNING << "HR logger: 无法打开 " << path << std::endl;
        return;
    }

    this->hr_num_dofs = this->params.Get<int>("num_of_dofs");
    this->hr_jm = this->params.Get<std::vector<int>>("joint_mapping");
    this->hr_offset = this->params.Get<std::vector<float>>("lying_down_offset");

    this->hr_file << "t,gyro_x,gyro_y,gyro_z,acc_x,acc_y,acc_z,"
                     "quat_w,quat_x,quat_y,quat_z,cmd_x,cmd_y,cmd_z";
    for (int i = 0; i < this->hr_num_dofs; ++i)
        this->hr_file << ",m" << i << "_tgt_pos,m" << i << "_real_pos,m" << i
                      << "_tgt_vel,m" << i << "_real_vel,m" << i << "_real_tau,m" << i << "_temp";
    this->hr_file << "\n";
    this->hr_file.flush();

    this->hr_t0 = std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    this->hr_rows = 0;
    std::cout << LOGGER::INFO << "HR logger (200Hz) 已启动: " << path << std::endl;
}

void RL_Real::HrLogPush()
{
    if (!this->hr_file.is_open() || this->hr_num_dofs <= 0) return;
    if (!this->rl_init_done) return;   // 仅在策略加载后记录

    const auto &imu = this->robot_state.imu;
    if (imu.gyroscope.size() < 3 || imu.accelerometer.size() < 3 || imu.quaternion.size() < 4)
        return;
    auto fb = motor_manager.getAllMotorFeedback();  // 一次加锁, 同帧一致
    const auto &mc = this->robot_command.motor_command;

    double now = std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();

    this->hr_file << std::fixed << std::setprecision(5)
                  << (now - this->hr_t0) << ','
                  << imu.gyroscope[0] << ',' << imu.gyroscope[1] << ',' << imu.gyroscope[2] << ','
                  << imu.accelerometer[0] << ',' << imu.accelerometer[1] << ',' << imu.accelerometer[2] << ','
                  << imu.quaternion[0] << ',' << imu.quaternion[1] << ','
                  << imu.quaternion[2] << ',' << imu.quaternion[3] << ','
                  << this->control.x << ',' << this->control.y << ',' << this->control.yaw;

    for (int i = 0; i < this->hr_num_dofs; ++i)
    {
        if ((size_t)i >= mc.q.size() || (size_t)i >= mc.dq.size()) break;
        int idx = (i < (int)this->hr_jm.size()) ? this->hr_jm[i] : i;
        float off = (i < (int)this->hr_offset.size()) ? this->hr_offset[i] : 0.0f;
        float rp = (fb.pos.size() > idx) ? (fb.pos[idx] + off) : 0.0f;
        float rv = (fb.vel.size() > idx) ? fb.vel[idx] : 0.0f;
        float rt = (fb.tau.size() > idx) ? fb.tau[idx] : 0.0f;
        float tp = (fb.temp.size() > (size_t)idx) ? fb.temp[idx] : 0.0f;
        this->hr_file << ',' << mc.q[i] << ',' << rp
                      << ',' << mc.dq[i] << ',' << rv
                      << ',' << rt << ',' << tp;
    }
    this->hr_file << '\n';

    if (++this->hr_rows % 2000 == 0)
        this->hr_file.flush();   // ~10s 落盘一次
}

void RL_Real::HrLogClose()
{
    if (this->hr_file.is_open())
    {
        this->hr_file.flush();
        this->hr_file.close();
        std::cout << LOGGER::INFO << "HR logger 已关闭, 共 " << this->hr_rows << " 行" << std::endl;
    }
}
#endif


std::vector<float> RL_Real::Forward()
{
    std::unique_lock<std::mutex> lock(this->model_mutex, std::try_to_lock);

    if (!lock.owns_lock())
    {
        std::cout << LOGGER::WARNING << "Model is being reinitialized, using previous actions" << std::endl;
        return this->obs.actions;
    }

    std::vector<float> clamped_obs = this->ComputeObservation();

    // policy yaml 在模型加载阶段读入,首次推理时把相关键补进缓存
    if (!cfg_policy_cached_)
    {
        CacheHotParams();
        cfg_policy_cached_ = true;
    }

    std::vector<float> actions;
    if (!cfg_observations_history_.empty())
    {
        this->history_obs_buf.insert(clamped_obs);
        this->history_obs = this->history_obs_buf.get_obs_vec(cfg_observations_history_);
        actions = this->model->forward({this->history_obs});
    }
    else
    {
        actions = this->model->forward({clamped_obs});
    }

    if (cfg_has_clip_)
    {
        return clamp(actions, cfg_clip_actions_lower_, cfg_clip_actions_upper_);
    }
    else
    {
        return actions;
    }
}




namespace {
std::string normalizeCommand(std::string value)
{
    auto not_space = [](unsigned char c) { return !std::isspace(c); };
    value.erase(value.begin(), std::find_if(value.begin(), value.end(), not_space));
    value.erase(std::find_if(value.rbegin(), value.rend(), not_space).base(), value.end());
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return value;
}
}

void RL_Real::CmdvelCallback(const CmdVel &msg)
{
    this->last_cmd_vel_time_ = std::chrono::steady_clock::now();
    this->cmd_vel = msg;
    if (this->gamepad_override_active)
    {
        PublishRosFeedback("cmd_vel ignored: gamepad override active");
        return;
    }

    this->control.x = static_cast<float>(msg.x);
    this->control.y = static_cast<float>(msg.y);
    this->control.yaw = static_cast<float>(msg.yaw);
    std::cout << LOGGER::INFO << "cmd_vel: linear("
              << msg.x << ", " << msg.y << ", 0"
              << ") angular(0, 0, " << msg.yaw
              << ")" << std::endl;
    PublishRosFeedback("cmd_vel accepted");
}

void RL_Real::BriefingCallback(const std::string &msg)
{
    if (!briefing_audio_) return;

    std::string cmd = msg;
    while (!cmd.empty() && (cmd.back() == ' ' || cmd.back() == '\n' ||
                            cmd.back() == '\r' || cmd.back() == '\t')) {
        cmd.pop_back();
    }
    while (!cmd.empty() && (cmd.front() == ' ' || cmd.front() == '\t')) {
        cmd.erase(cmd.begin());
    }
    if (cmd.empty()) return;

    auto publish = [this](const std::string& s) {
        if (!this->web_bridge_) return;
        this->web_bridge_->publishBriefingStatus(s);
    };

    if (cmd == "stop") {
        briefing_audio_->stop();
        publish("stopped");
        return;
    }
    if (cmd == "list") {
        std::string out = "audio list:";
        for (const auto& n : briefing_audio_->list()) {
            out += " " + n;
        }
        publish(out);
        return;
    }

    bool ok = briefing_audio_->play(cmd);
    publish(ok ? ("playing: " + cmd) : ("not found: " + cmd));
}

void RL_Real::ImuStreamCallback(bool enabled)
{
    SetJXGIMUStreamEnabled(enabled);
    PublishRosFeedback(std::string("imu_stream=") + (enabled ? "on" : "off"));
}

void RL_Real::ApplyCommand(const std::string &command)
{
    const std::string cmd = normalizeCommand(command);
    if (cmd.empty()) return;

    if (this->gamepad_override_active && cmd != "feedback" && cmd != "status")
    {
        PublishRosFeedback("command ignored: gamepad override active");
        return;
    }

    if (cmd == "getup" || cmd == "stand" || cmd == "up")
    {
        this->control.SetKeyboard(Input::Keyboard::Num0);
        PublishRosFeedback("command accepted: getup");
    }
    else if (cmd == "getdown" || cmd == "down" || cmd == "lie" || cmd == "sit")
    {
        this->control.SetKeyboard(Input::Keyboard::Num9);
        this->control.x = 0.0f;
        this->control.y = 0.0f;
        this->control.yaw = 0.0f;
        PublishRosFeedback("command accepted: getdown");
    }
    else if (cmd == "locomotion" || cmd == "rl" || cmd == "walk" || cmd == "move")
    {
        this->control.SetKeyboard(Input::Keyboard::Num1);
        PublishRosFeedback("command accepted: locomotion");
    }
    else if (cmd == "passive" || cmd == "stop" || cmd == "safe_stop")
    {
        this->control.SetKeyboard(Input::Keyboard::P);
        this->control.x = 0.0f;
        this->control.y = 0.0f;
        this->control.yaw = 0.0f;
        this->cmd_vel = CmdVel();
        PublishRosFeedback("command accepted: passive stop");
    }
    else if (cmd == "vel_stop" || cmd == "hold")
    {
        this->control.x = 0.0f;
        this->control.y = 0.0f;
        this->control.yaw = 0.0f;
        this->cmd_vel = CmdVel();
        PublishRosFeedback("command accepted: velocity stop");
    }
    else if (cmd == "nav_on")
    {
        this->control.navigation_mode = true;
        PublishRosFeedback("command accepted: nav_on");
    }
    else if (cmd == "nav_off")
    {
        this->control.navigation_mode = false;
        PublishRosFeedback("command accepted: nav_off");
    }
    else if (cmd == "imu_on")
    {
        SetJXGIMUStreamEnabled(true);
        PublishRosFeedback("command accepted: imu_on");
    }
    else if (cmd == "imu_off")
    {
        SetJXGIMUStreamEnabled(false);
        PublishRosFeedback("command accepted: imu_off");
    }
    else if (cmd == "zero" || cmd == "setzero" || cmd == "set_zero" || cmd == "set_zero_all" || cmd == "zero_all")
    {
        motor_manager.setZeroAllMotors();
        // 等待电机反馈稳定后重新评估起立条件，避免置零后必须重启程序
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        this->motors_ready_for_stand = this->CheckMotorInitPosition(1.0f);
        if (this->motors_ready_for_stand) {
            PublishRosFeedback("command accepted: set_zero_all (motors ready for stand)");
            this->Notify("info", "motors re-zeroed, ready for stand");
        } else {
            PublishRosFeedback("command accepted: set_zero_all (motors not ready)");
            this->Notify("error", this->motors_not_ready_reason);
        }
    }
    else if (cmd.rfind("zero_motor", 0) == 0)
    {
        // 单电机置零：zero_motor <motor_id 0-15>
        // motor_id = 腿*4 + 关节（腿 0~3 = FL/FR/RL/RR，关节 0=髋/1=大腿/2=小腿/3=足端轮）
        std::istringstream iss(cmd);
        std::string token;
        int motor_id = -1;
        iss >> token >> motor_id;
        if (motor_id < 0 || motor_id > 15)
        {
            PublishRosFeedback("zero_motor rejected: invalid args (usage: zero_motor <motor_id 0-15>)");
        }
        else
        {
            motor_manager.setZero(motor_id);
            // 等待反馈稳定后重新评估起立条件
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            this->motors_ready_for_stand = this->CheckMotorInitPosition(1.0f);
            std::ostringstream oss;
            oss << "command accepted: zero_motor motor_id=" << motor_id;
            PublishRosFeedback(oss.str());
        }
    }
    else if (cmd == "quit" || cmd == "exit")
    {
        g_shutdown_flag.store(true);
        PublishRosFeedback("command accepted: exit");
    }
    else if (cmd == "feedback" || cmd == "status")
    {
        std::string state_name = this->fsm.current_state_ ? this->fsm.current_state_->GetStateName() : "unknown";
        PublishRosFeedback("status state=" + state_name + " gamepad_override=" + (this->gamepad_override_active ? "true" : "false"));
    }
    else
    {
        PublishRosFeedback("unknown command: " + cmd);
    }
}

void RL_Real::PublishRosFeedback(const std::string &message)
{
    if (!this->web_bridge_) return;
    this->web_bridge_->publishFeedback(message);
}

void RL_Real::Notify(const std::string &type, const std::string &content)
{
    this->PublishNotify(type, content);
}

void RL_Real::PublishNotify(const std::string &type, const std::string &content)
{
    if (!this->web_bridge_) return;

    uint32_t id = ++this->notify_seq_;
    double now_sec = std::chrono::duration<double>(
        std::chrono::system_clock::now().time_since_epoch()).count();

    // 手动构建 JSON（无第三方 JSON 库）：内容做最小转义（反斜杠、双引号）
    std::string esc = content;
    for (size_t p = 0; (p = esc.find('\\', p)) != std::string::npos; p += 2)
        esc.replace(p, 1, "\\\\");
    for (size_t p = 0; (p = esc.find('"', p)) != std::string::npos; p += 2)
        esc.replace(p, 1, "\\\"");

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3);
    oss << "{\"id\":" << id
        << ",\"type\":\"" << type << "\""
        << ",\"content\":\"" << esc << "\""
        << ",\"timestamp\":" << now_sec << "}";

    this->web_bridge_->publishNotify(oss.str());

    NotifyRecord rec;
    rec.json = oss.str();
    rec.timestamp = now_sec;
    rec.retry_count = 0;
    std::lock_guard<std::mutex> lock(this->notify_mutex_);
    this->notify_pending_[id] = rec;
}

void RL_Real::NotifyAckCallback(const std::string &json)
{
    // 解析 {"id":N,"ok":true}
    const std::string &s = json;
    uint32_t id = 0;
    bool ok = false;

    size_t id_pos = s.find("\"id\"");
    if (id_pos != std::string::npos) {
        size_t colon = s.find(':', id_pos);
        if (colon != std::string::npos) {
            try {
                id = static_cast<uint32_t>(std::stoul(s.substr(colon + 1)));
            } catch (...) {
                return;
            }
        }
    }
    size_t ok_pos = s.find("\"ok\"");
    if (ok_pos != std::string::npos)
        ok = (s.find("true", ok_pos) != std::string::npos);

    std::lock_guard<std::mutex> lock(this->notify_mutex_);
    auto it = this->notify_pending_.find(id);
    if (it != this->notify_pending_.end() && ok)
        this->notify_pending_.erase(it);
}

void RL_Real::ProcessNotifyRetries(double now_sec)
{
    // 每秒最多处理一次，避免高频循环里频繁加锁
    if (now_sec - this->notify_last_retry_ts_ < 1.0)
        return;
    this->notify_last_retry_ts_ = now_sec;

    std::lock_guard<std::mutex> lock(this->notify_mutex_);
    for (auto it = this->notify_pending_.begin(); it != this->notify_pending_.end(); )
    {
        if (now_sec - it->second.timestamp > 1.0 && it->second.retry_count < 3)
        {
            if (this->web_bridge_) this->web_bridge_->publishNotify(it->second.json);
            it->second.retry_count++;
            ++it;
        }
        else if (it->second.retry_count >= 3)
        {
            it = this->notify_pending_.erase(it);  // 超过重试次数丢弃
        }
        else
        {
            ++it;
        }
    }
}

void RL_Real::CheckStandCallback()
{
    if (!this->web_bridge_) return;

    // 站立前检测：仅检测，不执行站立动作
    bool motors_ok = this->CheckMotorInitPosition(1.0f);
    bool imu_ok = !(this->robot_state.imu.gyroscope[0] == 0.0f &&
                    this->robot_state.imu.gyroscope[1] == 0.0f &&
                    this->robot_state.imu.gyroscope[2] == 0.0f);

    bool ready = motors_ok && imu_ok;
    std::string reason;
    if (!motors_ok)
        reason = this->motors_not_ready_reason;
    else if (!imu_ok)
        reason = "IMU no data";

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3);
    oss << "{\"ready\":" << (ready ? "true" : "false")
        << ",\"motors_ready\":" << (motors_ok ? "true" : "false")
        << ",\"imu_ready\":" << (imu_ok ? "true" : "false")
        << ",\"reason\":\"" << JsonEscape(reason) << "\"}";

    this->web_bridge_->publishCheckStandResult(oss.str());
}

void RL_Real::PublishMotorState(int num_dofs,
                                const std::vector<int> &joint_mapping,
                                const std::vector<float> &lying_down_offset,
                                const Eigen::VectorXf &all_pos,
                                const Eigen::VectorXf &all_vel,
                                const Eigen::VectorXf &all_tau,
                                const std::vector<float> &all_temp)
{
    if (!this->web_bridge_) return;

    // 字段块序：idx,target_pos,real_pos,target_vel,real_vel,target_tau,real_tau,kp,kd,temp
    // 前端 ros-bridge.js 按 data 长度自适应 stride=10/9，温度为第 10 字段(°C)。
    constexpr int fields_per_motor = 10;
    std::vector<float> data;
    data.reserve(static_cast<size_t>(num_dofs * fields_per_motor));

    for (int i = 0; i < num_dofs; ++i)
    {
        int idx = joint_mapping[i];
        data.push_back(static_cast<float>(idx));
        data.push_back(this->robot_command.motor_command.q[i]);
        data.push_back(all_pos[idx] + lying_down_offset[i]);
        data.push_back(this->robot_command.motor_command.dq[i]);
        data.push_back(all_vel[idx]);
        data.push_back(this->robot_command.motor_command.tau[i]);
        data.push_back(all_tau[idx]);
        data.push_back(this->robot_command.motor_command.kp[i]);
        data.push_back(this->robot_command.motor_command.kd[i]);
        data.push_back(idx < static_cast<int>(all_temp.size()) ? all_temp[idx] : 0.0f);
    }

    this->web_bridge_->publishMotorState(data);
}

void RL_Real::PublishImuState()
{
    if (!this->web_bridge_) return;

    const auto &imu = this->JXG_state.imu_state;
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

    // 字段序：quat_w,quat_x,quat_y,quat_z,roll,pitch,yaw,acc_x,acc_y,acc_z,
    //         gyro_x,gyro_y,gyro_z,pos_north,pos_east,pos_down,vel_body_x,vel_body_y,vel_body_z
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

    this->web_bridge_->publishImuState(data);
}

void RL_Real::PublishPose2D()
{
    if (!this->web_bridge_) return;

    // 字段序：x,y,z,yaw,vel_x,vel_y,yaw_rate
    std::vector<float> data = {
        static_cast<float>(this->odometry_x),
        static_cast<float>(this->odometry_y),
        static_cast<float>(this->odometry_z),
        static_cast<float>(this->odometry_yaw),
        static_cast<float>(this->velocity_x),
        static_cast<float>(this->velocity_y),
        static_cast<float>(this->control.yaw)
    };

    this->web_bridge_->publishPose2D(data);
}

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
        this->plot_real_joint_pos[i].push_back(this->JXG_state.motor_state[i].q);
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

bool RL_Real::CheckMotorInitPosition(float threshold_rad)
{
    // 轮毂电机索引，跳过检查
    static const std::vector<int> wheel_indices = {3, 7, 11, 15};

    auto all_pos = motor_manager.getAllMotorPos();

    bool all_ok = true;
    std::string bad_motors;

    for (int i = 0; i < 16; ++i)
    {
        // 跳过轮毂电机
        if (std::find(wheel_indices.begin(), wheel_indices.end(), i) != wheel_indices.end())
            continue;

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
        this->motors_not_ready_reason = "Motor(s) too far from zero:" + bad_motors
            + " (threshold=" + std::to_string(threshold_rad) + "rad). Please re-zero motors.";
    }

    return all_ok;
}

int main(int argc, char **argv)
{
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    // 锁定全部当前及未来内存页,避免实时线程运行期间发生缺页(含 swap/page fault)抖动。
    // 服务以 root 运行,默认具备 CAP_IPC_LOCK;非 root 无权限时仅告警,不影响启动。
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
        std::cerr << "[main] WARN: mlockall 失败: " << std::strerror(errno)
                  << "(实时线程仍可能遇到缺页抖动;以 root 运行可消除)" << std::endl;
    }

    // 解析命令行参数
    bool wheel_mode = false;
    WebConfig web_cfg;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "wheel") {
            wheel_mode = true;
        } else if (arg == "--web-port" && i + 1 < argc) {
            web_cfg.web_port = std::atoi(argv[++i]);
        } else if (arg == "--web-root" && i + 1 < argc) {
            web_cfg.web_root = argv[++i];
        } else if (arg == "--briefing-audio-dir" && i + 1 < argc) {
            web_cfg.briefing_audio_dir = argv[++i];
        } else if (arg == "-h" || arg == "--help") {
            std::cout << "Usage: rl_real_JXG [wheel] [--web-port N] [--web-root DIR]"
                         " [--briefing-audio-dir DIR]" << std::endl;
            return 0;
        }
    }

    RL_Real rl_real(wheel_mode, web_cfg);
    std::cout << "RL_Real JXG started in " << (wheel_mode ? "wheel" : "normal") << " mode" << std::endl;
    // 控制环/推理环由 LoopFunc 线程驱动；主线程保持与原 ROS spin 相同的 20ms 节奏
    while (!g_shutdown_flag.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    std::cout << "Program exiting..." << std::endl;

    return 0;
}