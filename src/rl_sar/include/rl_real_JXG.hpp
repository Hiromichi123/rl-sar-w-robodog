#pragma once

#include "rl_sdk.hpp"
#include "observation_buffer.hpp"
#include "loop.hpp"
#include "fsm.hpp"
#include "fsm_JXG.hpp"
#include "fsm_JXGw.hpp"
#include "JXG_low_cmd.h"
#include "JXG_low_state.h"
#include <SDL2/SDL.h>
#include <memory>
#include <string>
#include <vector>
#include <fstream>
#include <atomic>
#include <mutex>
#include <map>
#include <cstdint>
#include <chrono>
#include "matplotlibcpp.h"
#include "briefing_audio_player.hpp"

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

#include "web_bridge.hpp"

// Web 调试台默认静态文件根目录（由 CMake 注入，可用 --web-root 覆盖）
#ifndef WEB_ROOT
#define WEB_ROOT "./web"
#endif

// 主程序 Web/音频相关配置（由 main() 从命令行解析后传入）
struct WebConfig
{
    int web_port = 8088;
    std::string web_root = WEB_ROOT;
    std::string briefing_audio_dir = "/home/kickpi/briefing_audio";
};


struct GamepadState {
    bool A = false, B = false, X = false, Y = false;
    bool LB = false, RB = false;
    bool L3 = false, R3 = false;        // 左/右摇杆按下
    bool back = false, start = false;
    bool up = false, down = false, left = false, right = false;

    float left_x = 0.0f, left_y = 0.0f;
    float right_x = 0.0f, right_y = 0.0f;
};

class RL_Real : public RL
{
public:
    explicit RL_Real(bool wheel_mode = false, const WebConfig &web_cfg = WebConfig());
    ~RL_Real();

private:
  
    std::vector<float> Forward() override;
    void GetState(RobotState<float> *state) override;
    void SetCommand(const RobotCommand<float> *command) override;
    void RunModel();
    void RobotControl();

    // 循环
    std::shared_ptr<LoopFunc> loop_keyboard;
    std::shared_ptr<LoopFunc> loop_control;
    std::shared_ptr<LoopFunc> loop_rl;
    std::shared_ptr<LoopFunc> loop_gamepad;      // 200Hz 手柄专用线程
    std::shared_ptr<LoopFunc> loop_imu;          // 1kHz IMU读取线程
    std::shared_ptr<LoopFunc> loop_plot;


    
    JXG_low_cmd    JXG_cmd;
    JXGLowState    JXG_state{16};

    struct XboxGamepad {
        SDL_Joystick* joy = nullptr;
        GamepadState state{};
        bool connected = false;

        XboxGamepad(int idx = 0) {
            if (SDL_InitSubSystem(SDL_INIT_JOYSTICK) != 0) return;
            joy = SDL_JoystickOpen(idx);
            connected = (joy != nullptr);
            if (connected) printf("[Xbox] 手柄已连接成功: %s\n", SDL_JoystickName(joy));
        }
        ~XboxGamepad() {
            if (joy) SDL_JoystickClose(joy);
            SDL_QuitSubSystem(SDL_INIT_JOYSTICK);
        }

        void update() {
            SDL_JoystickUpdate();  // 必须每帧调用，SDL靠它检测热插拔

            if (!connected) {
                // 未连接时每~1秒尝试重连一次（200Hz / 200 = 1s）
                static int reconnect_cnt = 0;
                if (++reconnect_cnt >= 200 && SDL_NumJoysticks() > 0) {
                    reconnect_cnt = 0;
                    joy = SDL_JoystickOpen(0);
                    if (joy) {
                        connected = true;
                        printf("[Xbox] 手柄重新连接成功: %s\n", SDL_JoystickName(joy));
                    }
                }
                return;
            }

            // 检测是否断连
            if (!SDL_JoystickGetAttached(joy)) {
                connected = false;
                printf("[Xbox] 手柄断开连接!\n");
                SDL_JoystickClose(joy);
                joy = nullptr;
                state = {};  // 清零状态，防止保持断开前的按键
                return;
            }

            // 按钮映射 (gaishixiaoji专用)
            state.A     = SDL_JoystickGetButton(joy, 1);
            state.B     = SDL_JoystickGetButton(joy, 2);
            state.X     = SDL_JoystickGetButton(joy, 0);
            state.Y     = SDL_JoystickGetButton(joy, 3);
            state.LB    = SDL_JoystickGetButton(joy, 4);
            state.RB    = SDL_JoystickGetButton(joy, 5);
            state.back  = SDL_JoystickGetButton(joy, 13);
            state.start = SDL_JoystickGetButton(joy, 12);
            state.L3    = SDL_JoystickGetButton(joy, 6);
            state.R3    = SDL_JoystickGetButton(joy, 7);
            int hat = SDL_JoystickGetHat(joy, 0);
            state.up    = hat & SDL_HAT_UP;
            state.down  = hat & SDL_HAT_DOWN;
            state.left  = hat & SDL_HAT_LEFT;
            state.right = hat & SDL_HAT_RIGHT;

            auto norm = [](Sint16 v) -> float {
                float val = v / 32767.0f;
                return (fabs(val) < 0.08f) ? 0.0f : val;
            };
            state.left_x  =   -norm(SDL_JoystickGetAxis(joy, 0));   // 左右取反，和宇树一致
            state.left_y  =   norm(SDL_JoystickGetAxis(joy, 1));
            state.right_x =   norm(SDL_JoystickGetAxis(joy, 2));
            state.right_y =   -norm(SDL_JoystickGetAxis(joy, 5));   // 右摇杆
            //  调试：打印所有轴的值，找到右摇杆的正确索引
            // static int debug_count = 0;

            // if (debug_count % 100 == 0) {  
            // //每100次循环打印一次，避免刷屏
            //     printf("[DEBUG] Axes: 0=%.2f, 1=%.2f, 2=%.2f, 3=%.2f, 4=%.2f, 5=%.2f\n",
            //         norm(SDL_JoystickGetAxis(joy, 0)),
            //         norm(SDL_JoystickGetAxis(joy, 1)),
            //         norm(SDL_JoystickGetAxis(joy, 2)),
            //         norm(SDL_JoystickGetAxis(joy, 3)),
            //         norm(SDL_JoystickGetAxis(joy, 4)),
            //         norm(SDL_JoystickGetAxis(joy, 5)));
            //         }
            //       debug_count++;
                


            // for (int i = 0; i < SDL_JoystickNumButtons(joy); i++)
            // {
            //     int val = SDL_JoystickGetButton(joy, i);
            //     if (val)
            //      {
            //         printf("Pressed button index = %d\n", i);
            //      }
            // }

            // // 调试：打印hat状态
            // int num_hats = SDL_JoystickNumHats(joy);
            // for (int h = 0; h < num_hats; h++) {
            //     int hat = SDL_JoystickGetHat(joy, h);
            //     if (hat != SDL_HAT_CENTERED) {
            //         printf("[DEBUG] Hat %d value = 0x%X (", h, hat);
            //         if (hat & SDL_HAT_UP) printf("UP ");
            //         if (hat & SDL_HAT_DOWN) printf("DOWN ");
            //         if (hat & SDL_HAT_LEFT) printf("LEFT ");
            //         if (hat & SDL_HAT_RIGHT) printf("RIGHT ");
            //         printf(")\n");
            //     }
            // }
        }
    };

    std::unique_ptr<XboxGamepad> xbox_gamepad;
    GamepadState gamepad_state{};
    GamepadState last_gamepad_state{};
    bool gamepad_override_active = false;

    void InitLowCmd();
    bool CheckMotorInitPosition(float threshold_rad);

    // 电机反馈异常连续计数器（16个电机，每帧更新）
    // 注意: 不能写 {16, 0} —— 花括号会走 initializer_list 构造, 得到只有 2 个元素 {16,0},
    // 访问下标 2 即越界(C++17 类成员也不允许圆括号直接初始化, 故用 = 形式)。
    std::vector<int> motor_dead_count = std::vector<int>(16, 0);


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
    // 高频(200Hz)记录器
    void HrLogInit();
    void HrLogPush();
    void HrLogClose();
    std::ofstream hr_file;
    double hr_t0 = 0.0;                 // 记录起点(steady_clock)
    long hr_rows = 0;                   // 已写行数(用于定期 flush)
    int hr_num_dofs = 0;                // 缓存的 num_of_dofs
    std::vector<int> hr_jm;             // 缓存的 joint_mapping
    std::vector<float> hr_offset;       // 缓存的 lying_down_offset
#endif

    // ===== 热路径配置缓存 =====
    // 200Hz 控制环直接访问,避免每周期数十次 YAML 树查找 + vector 重分配
    //(实测这部分曾占满单个控制周期 5.2ms/5.7ms, 导致 loop_control 99% CPU)。
    // 在构造函数 params 加载完成后由 CacheHotParams() 初始化一次。
    int cfg_num_dofs_ = 0;
    float cfg_dt_ = 0.005f;
    int cfg_decimation_ = 1;
    std::vector<int>   cfg_joint_mapping_;
    std::vector<float> cfg_lying_down_offset_;
    std::vector<int>   cfg_observations_history_;
    std::vector<float> cfg_clip_actions_upper_;
    std::vector<float> cfg_clip_actions_lower_;
    bool cfg_has_clip_ = false;
    bool cfg_policy_cached_ = false;   // policy yaml 参数在模型加载后再缓存一次
    // SetCommand 复用的 16 元素缓冲(避免 200Hz 堆分配)
    std::vector<float> cmd_tau_, cmd_vel_, cmd_pos_, cmd_kp_, cmd_kd_;
    void CacheHotParams();

    // ===== 内置 Web 调试台桥（替代原 ROS2 话题层 + rosbridge） =====
    std::unique_ptr<WebBridge> web_bridge_;
    CmdVel cmd_vel;                                            // 最近一次上位机速度指令
    std::chrono::steady_clock::time_point last_cmd_vel_time_;  // 最后收到 cmd_vel 的时间（命令超时保护）
    void CmdvelCallback(const CmdVel &msg);
    void ImuStreamCallback(bool enabled);
    void ApplyCommand(const std::string &command);
    void PublishRosFeedback(const std::string &message);
    void PublishMotorState(int num_dofs,
                           const std::vector<int> &joint_mapping,
                           const std::vector<float> &lying_down_offset,
                           const Eigen::VectorXf &all_pos,
                           const Eigen::VectorXf &all_vel,
                           const Eigen::VectorXf &all_tau,
                           const std::vector<float> &all_temp);
    void PublishImuState();
    void PublishPose2D();
    void Notify(const std::string &type, const std::string &content) override;
    void PublishNotify(const std::string &type, const std::string &content);
    void NotifyAckCallback(const std::string &json);
    void ProcessNotifyRetries(double now_sec);
    void CheckStandCallback();

    // 场景音频播放（预设 wav + Web 触发）
    void BriefingCallback(const std::string &msg);
    std::unique_ptr<rl_briefing::BriefingAudioPlayer> briefing_audio_;

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
};