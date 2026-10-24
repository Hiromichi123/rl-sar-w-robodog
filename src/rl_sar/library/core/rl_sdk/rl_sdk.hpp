/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef RL_SDK_HPP
#define RL_SDK_HPP

#include <iostream>
#include <string>
#include <exception>
#include <unistd.h>
#include <algorithm>
#include <vector>
#include <memory>
#include <fstream>
#include <mutex>
#include <atomic>
#include <chrono>

#include <yaml-cpp/yaml.h>
#include "fsm.hpp"
#include "observation_buffer.hpp"
#include "vector_math.hpp"
#include "inference_runtime.hpp"
#include "logger.hpp"
#include "motion_loader.hpp"

template <typename T>
struct RobotCommand
{
    struct MotorCommand
    {
        std::vector<int> mode;
        std::vector<T> q;
        std::vector<T> dq;
        std::vector<T> tau;
        std::vector<T> kp;
        std::vector<T> kd;

        void resize(size_t num_joints)
        {
            mode.resize(num_joints, 0);
            q.resize(num_joints, 0.0f);
            dq.resize(num_joints, 0.0f);
            tau.resize(num_joints, 0.0f);
            kp.resize(num_joints, 0.0f);
            kd.resize(num_joints, 0.0f);
        }
    } motor_command;
};

template <typename T>
struct RobotState
{
    struct IMU
    {
        std::vector<T> quaternion = {1.0f, 0.0f, 0.0f, 0.0f}; // w, x, y, z
        std::vector<T> gyroscope = {0.0f, 0.0f, 0.0f};
        std::vector<T> accelerometer = {0.0f, 0.0f, 0.0f};
    } imu;

    struct MotorState
    {
        std::vector<T> q;
        std::vector<T> dq;
        std::vector<T> ddq;
        std::vector<T> tau_est;
        std::vector<T> cur;

        void resize(size_t num_joints)
        {
            q.resize(num_joints, 0.0f);
            dq.resize(num_joints, 0.0f);
            ddq.resize(num_joints, 0.0f);
            tau_est.resize(num_joints, 0.0f);
            cur.resize(num_joints, 0.0f);
        }
    } motor_state;
};

namespace Input
{
    // Recommend: Num0-GetUp Num9-GetDown N-ToggleNavMode
    //            R-SimReset Enter-SimToggle
    //            M-MotorEnable K-MotorDisable P-MotorPassive
    //            Num1-BaseLocomotion Num2-Num8-Skills(7)
    //            WS-AxisX AD-AxisY QE-AxisYaw Space-AxisClear
    enum class Keyboard
    {
        None = 0,
        A, B, C, D, E, F, G, H, I, J, K, L, M,
        N, O, P, Q, R, S, T, U, V, W, X, Y, Z,
        Num0, Num1, Num2, Num3, Num4, Num5, Num6, Num7, Num8, Num9,
        Space, Enter, Escape,
        Up, Down, Left, Right
    };

    // Recommend: A-GetUp B-GetDown X-ToggleNavMode Y-None
    //            RB_Y-SimReset RB_X-SimToggle
    //            LB_A-MotorEnable LB_B-MotorDisable LB_X-MotorPassive
    //            RB_DPadUp-BaseLocomotion RB_DPadOthers/LB_DPadOthers-Skills(7)
    //            LY-AxisX LX-AxisY RX-AxisYaw
    enum class Gamepad
    {
        None = 0,
        A, B, X, Y, LB, RB, LStick, RStick, DPadUp, DPadDown, DPadLeft, DPadRight,
        LB_A, LB_B, LB_X, LB_Y, LB_LStick, LB_RStick, LB_DPadUp, LB_DPadDown, LB_DPadLeft, LB_DPadRight,
        RB_A, RB_B, RB_X, RB_Y, RB_LStick, RB_RStick, RB_DPadUp, RB_DPadDown, RB_DPadLeft, RB_DPadRight,
        LB_RB
    };
}

struct Control
{
    Input::Keyboard current_keyboard = Input::Keyboard::None, last_keyboard = Input::Keyboard::None;
    Input::Gamepad current_gamepad = Input::Gamepad::None, last_gamepad = Input::Gamepad::None;

    float x = 0.0f;
    float y = 0.0f;
    float yaw = 0.0f;
    bool navigation_mode = false;

    void SetKeyboard(Input::Keyboard keyboad)
    {
        if (current_keyboard != keyboad)
        {
            last_keyboard = current_keyboard;
            current_keyboard = keyboad;
        }
    }

    void SetGamepad(Input::Gamepad gamepad)
    {
        if (current_gamepad != gamepad)
        {
            last_gamepad = current_gamepad;
            current_gamepad = gamepad;
        }
    }

    void ClearInput()
    {
        current_keyboard = last_keyboard;
        current_gamepad = Input::Gamepad::None;
    }
};

struct YamlParams
{
    YAML::Node config_node;

    // Get config value by key
    // WARNING: For vectors/containers, store result in a variable before using iterators/references:
    //   ✓ auto vec = params.Get<std::vector<int>>("key"); vec.begin()
    //   ✗ params.Get<std::vector<int>>("key").begin()  // dangling reference!
    template<typename T>
    T Get(const std::string& key, const T& default_value = T()) const
    {
        if (config_node[key])
        {
            return config_node[key].as<T>();
        }
        return default_value;
    }

    bool Has(const std::string& key) const
    {
        return config_node[key].IsDefined();
    }
};

template <typename T>
struct Observations
{
    std::vector<T> lin_vel;
    std::vector<T> ang_vel;
    std::vector<T> gravity_vec;
    std::vector<T> commands;
    std::vector<T> base_quat;
    std::vector<T> dof_pos;
    std::vector<T> dof_vel;
    std::vector<T> actions;
};

// ============================================================================
// SPSCTripleBuffer: 单生产者/单消费者无锁三缓冲(latest-value 邮箱)
//
// 用于替代原 tbb::concurrent_queue 连接推理线程(loop_rl ~50Hz, 生产者)与
// 控制线程(loop_control ~200Hz, 消费者)。原方案在 ARM 上实测出现活锁:
// 消费者 try_pop 遇到生产者 push 正在申请新内存页时会 sched_yield 自旋,
// 两线程在同一 page 计数器上互相让空转(CPU 100% 但均不前进), 且无界
// 队列还会持续内存分配。
//
// 本实现:
//   * 启动后零内存分配、零自旋、零系统调用(仅 1 字节原子 exchange);
//   * 消费者永远取生产者最新发布的一帧(RL 策略本就要最新动作, 无需 FIFO);
//   * 三个槽保证生产者与消费者任何时刻操作不同内存, 无数据撕裂;
//   * 无新帧时 Refresh() 返回 false, 消费者保持上一帧(不阻塞)。
//
// flags_ 编码(1 字节): bit7=dirty(有新帧), bit0-1=READY 槽序号。
// 生产者拥有 FREE 槽(write_idx_), 消费者拥有 BUSY 槽(read_idx_)。
// ============================================================================
template <typename T>
class SPSCTripleBuffer
{
public:
    SPSCTripleBuffer() = default;
    explicit SPSCTripleBuffer(const T& init)
    {
        for (auto& b : buf_) b = init;
    }

    // 生产者: 获取可写槽引用, 填充数据后调用 Commit() 发布。
    T& Write() { return buf_[write_idx_]; }

    void Commit()
    {
        // 原子发布当前写槽; 取回的旧 flags 低 2 位即下一可用 FREE 槽
        // (旧 READY 未被消费则覆盖之; 已被消费则是消费者刚释放的旧 BUSY 槽)。
        uint8_t old = flags_.exchange(kDirty | write_idx_, std::memory_order_acq_rel);
        write_idx_ = old & kIdxMask;
    }

    // 消费者: 有新帧时原子切换到最新帧, 返回 true; 无新帧返回 false。
    bool Refresh()
    {
        uint8_t f = flags_.load(std::memory_order_acquire);
        if (!(f & kDirty)) return false;
        // 把自己当前占有的 BUSY 槽登记为新的 READY 占位(clean),
        // 并取得生产者刚发布的槽作为新的 BUSY。
        uint8_t old = flags_.exchange(read_idx_ & kIdxMask, std::memory_order_acq_rel);
        read_idx_ = old & kIdxMask;
        return true;
    }

    const T& Read() const { return buf_[read_idx_]; }

private:
    static constexpr uint8_t kDirty = 0x80;
    static constexpr uint8_t kIdxMask = 0x03;
    // 初始状态机: 消费者 BUSY=槽0, 生产者 FREE=槽2, 槽1 为 clean 占位 READY。
    // 保证首次握手前生产者不会写到消费者持有的槽(构造时三槽均已初始化为同值)。
    std::atomic<uint8_t> flags_{1};
    T buf_[3];
    uint8_t write_idx_ = 2;          // 生产者首个写槽
    uint8_t read_idx_ = 0;           // 消费者初始占有槽
};

// 一帧 RL 输出: 位置/速度/前馈力矩同帧发布, 保证消费者不错配。
struct RLActionFrame
{
    std::vector<float> pos;
    std::vector<float> vel;
    std::vector<float> tau;
};

class RL
{
public:
    RL() {};
    ~RL() {};

    YamlParams params;
    Observations<float> obs;
    std::vector<int> obs_dims;

    RobotState<float> robot_state;
    RobotCommand<float> robot_command;

    // 推理线程 -> 控制线程的唯一动作通道(无锁三缓冲, 替代原 3 个 TBB 队列)。
    // InitOutputs() 时按 num_of_dofs 预分配, 运行期不再有任何堆分配。
    std::unique_ptr<SPSCTripleBuffer<RLActionFrame>> action_buf;
    // 推理线程发布最新一帧动作(pos/vel/tau 同帧)。
    void PublishAction();

    // 动作看门狗(仅控制线程访问这些字段): locomotion 态下若推理线程卡死,
    // 超过阈值未产生新帧, RLControl() 会零力矩并请求切 passive, 防止
    // 冻结在 locomotion 持续执行陈旧动作(曾导致轮子疯转)。
    static constexpr double kActionStaleTimeoutSec = 0.1;  // 50Hz 推理, 正常 20ms/帧
    std::chrono::steady_clock::time_point last_action_time_;
    bool action_watchdog_armed = false;
    bool action_watchdog_tripped = false;
    void ArmActionWatchdog();

    // P1-B 策略输出 NaN 拦截(RunModel 推理线程写, 控制环读):
    // clamp 对 NaN 无效(比较恒 false), 坏帧直接进 PD 会产生 NaN 力矩目标。
    // 单帧坏帧用上帧有效动作替换(首帧无历史则归零=default 姿态), 连续 3 帧
    // 置 nan_alert_, 控制环消费后并入 SafeHold 拦截组合。
    std::vector<float> last_valid_actions_;   // 仅推理线程访问, 无需锁
    std::atomic<int> nan_bad_streak_{0};
    std::atomic<bool> nan_alert_{false};

    FSM fsm;
    // StateController 缓存: fsm_state/fsm_command 为固定地址透传指针,
    // 入参不变时跳过对全部 FSM 状态的 dynamic_pointer_cast 遍历(仅控制线程访问)
    const RobotState<float>* last_ctrl_state_ = nullptr;
    RobotCommand<float>* last_ctrl_cmd_ = nullptr;
    RobotState<float> start_state;
    RobotState<float> now_state;
    bool rl_init_done = false;

    // 进入强化学习后前三秒输出姿态站立校验:防止模型暴走导致结构损伤。
    // 以进 RL 瞬间的实测关节角为基准(而非 default_dof_pos,避免 v27 模型零速站姿
    // 固有偏差 0.16~0.30rad 造成假阳性)。仅在零速/近零速时检查,用户主动给速度时放行。
    std::chrono::steady_clock::time_point rl_enter_time_;
    std::vector<float> rl_enter_q_;                          // 进 RL 时的 16 关节实测角
    int rl_stand_violation_count_ = 0;                         // 连续违规帧数
    std::atomic<bool> rl_stand_monitor_active_{false};         // 校验窗口是否激活

    // init
    void InitObservations();
    void InitOutputs();
    void InitControl();
    void InitRL(std::string robot_config_path);
    void InitJointNum(size_t num_joints);

    // rl functions
    virtual std::vector<float> Forward() = 0;
    std::vector<float> ComputeObservation();
    virtual void GetState(RobotState<float> *state) = 0;
    virtual void SetCommand(const RobotCommand<float> *command) = 0;
    void StateController(const RobotState<float> *state, RobotCommand<float> *command);

    // 通知/告警上报（真机 ROS2 控制中重写为 ROS 发布；仿真/无 ROS 环境为空实现）
    // type: info | warning | error | note
    virtual void Notify(const std::string &type, const std::string &content) { (void)type; (void)content; }
    void ComputeOutput(const std::vector<float> &actions, std::vector<float> &output_dof_pos, std::vector<float> &output_dof_vel, std::vector<float> &output_dof_tau);

    // yaml params
    void ReadYaml(const std::string& file_path, const std::string& file_name);

    // csv logger
    std::string csv_filename;
    void CSVInit(std::string robot_name);
    void CSVLogger(const std::vector<float> &torque, const std::vector<float> &tau_est, const std::vector<float> &joint_pos, const std::vector<float> &joint_pos_target, const std::vector<float> &joint_vel);

    // control
    Control control;
    void KeyboardInterface();

    // history buffer
    ObservationBuffer history_obs_buf;
    std::vector<float> history_obs;

    // others
    int motiontime = 0;
    std::string robot_name, config_name;
    bool simulation_running = true;
    std::string ang_vel_axis = "body";  // "world" or "body"
    unsigned long long episode_length_buf = 0;
    float motion_length = 0.0;
    int InverseJointMapping(int idx) const;

    // Motion tracking (for mimic/dance tasks)
    std::unique_ptr<MotionLoader> motion_loader;

    // protect func
    void TorqueProtect(const std::vector<float> &origin_output_dof_tau);
    void AttitudeProtect(const std::vector<float> &quaternion, float pitch_threshold, float roll_threshold);

    // rl module
    std::unique_ptr<InferenceRuntime::Model> model;
    // output buffer
    std::vector<float> output_dof_tau;
    std::vector<float> output_dof_pos;
    std::vector<float> output_dof_vel;

    // 电机初始位置校验（一次性，启动时检查）
    bool motors_ready_for_stand = true;
    std::string motors_not_ready_reason;

    // 起立置零校验(「卧姿校验」check_stand)手动结果,与探针同语义:
    //   * 仅 check_stand 命令(HandleCheckStand)写入,趴下(Passive::Enter)作废;
    //   * 结果不跨趴下周期,getup 门禁/胶囊 lie_done/lie_ready/前端预检三方同源;
    //   * done=false 表示本次趴下周期尚未手动校验(胶囊显灰,拒绝起立)。
    std::atomic<bool> lie_check_done_{false};
    std::atomic<bool> lie_check_ok_{false};

    // 置零校验:12 个腿关节当前角绝对值均 <= threshold_rad 才通过(4 个轮毂跳过),
    // 失败时 motors_not_ready_reason 填入超限电机明细。FSM 仅持有 RL&,故声明为虚函数;
    // 基类默认放行(仿真等非硬件平台不阻断起立),真实硬件 RL_Real override 为严格校验。
    virtual bool CheckMotorInitPosition(float /*threshold_rad*/) { return true; }

    // Hardware differential drive is opt-in; simulation/other robots reject it.
    virtual bool CanEnterDiffDrive() { return false; }
    virtual void EnterDiffDrive() {}
    virtual void RunDiffDrive(RobotCommand<float>*) {}
    virtual void ExitDiffDrive() {}
    virtual int DiffDriveRequest() { return 0; }

    // 当前行为动作是否已到位/进入稳态（由各 FSM 状态在控制环上报）。
    // 用于 Web 命令生命周期: getup 需等站立插值完成、getdown 需等回到 passive,
    // 命令到达目标态且本标志为真时才回 cmd_state=DONE。初始 passive 即稳态。
    std::atomic<bool> behavior_settled{true};

    // thread safety
    std::mutex model_mutex;
};

class RLFSMState : public FSMState
{
public:
    RLFSMState(RL& rl, const std::string& name)
        : FSMState(name), rl(rl), fsm_state(nullptr), fsm_command(nullptr) {}

    RL& rl;
    const RobotState<float> *fsm_state;
    RobotCommand<float> *fsm_command;

    bool Interpolate(
        float& percent,
        const std::vector<float>& start_pos,
        const std::vector<float>& target_pos,
        float duration_seconds,
        const std::string& description = "",
        bool use_fixed_gains = true
    );

    void RLControl();
};

#endif // RL_SDK_HPP
