#ifndef MOTOR_MANAGER_H
#define MOTOR_MANAGER_H

#include "usb2can.h"
#include "SerialPort.h"
#include "SocketCANPort.h"
#include "CanSetup.h"
#include <Eigen/Dense>
#include <thread>
#include <mutex>
#include <vector>
#include <memory>
#include <atomic>
#include <array>

// 电机映射表结构
struct MotorMap {
    int can_idx;      // CAN设备索引（0-3）
    uint32_t can_id;  // 电机CAN ID
    int leg_motor_idx;// 腿内索引（0-2）
    float dir;        // 方向系数（±1）
    MotorType motor_type; // 电机型号
};

// 腿部控制指令（使用可原子操作的结构）
// 安全:Eigen 定长向量默认不初始化!腿线程 startThreads() 后立即以 500Hz 发送槽内
// 内容,而首帧上层指令要等电机使能流程(~2s)后才写入——使能瞬间电机可能收到随机
// kp/kd(新堆内存常为全 0 → kp=kd=0 自由落体,站立时即砸地)。必须类内零初始化。
struct LegCommand {
    Eigen::RowVector4f tau = Eigen::RowVector4f::Zero();
    Eigen::RowVector4f vel = Eigen::RowVector4f::Zero();
    Eigen::RowVector4f pos = Eigen::RowVector4f::Zero();
    Eigen::RowVector4f kp  = Eigen::RowVector4f::Zero();
    Eigen::RowVector4f kd  = Eigen::RowVector4f::Zero();
};

// 腿部反馈数据
struct LegFeedback {
    Eigen::RowVector4f pos = Eigen::RowVector4f::Zero();
    Eigen::RowVector4f vel = Eigen::RowVector4f::Zero();
    Eigen::RowVector4f tau = Eigen::RowVector4f::Zero();
    Eigen::RowVector4f error = Eigen::RowVector4f::Zero(); // 协议无错误字段,恒 0
    Eigen::RowVector4f temp = Eigen::RowVector4f::Constant(-1.0f); // 电机温度℃,-1=无反馈
};

class RobotMotorManager {
public:
    RobotMotorManager();
    ~RobotMotorManager();

    // can_transport: "usb2can"(串口 /dev/ttycan*) 或 "socketcan"(内核 PF_CAN can0~can3)
    void init(const std::string& can_transport = "usb2can");
    void initAllMotors();
    void startThreads();
    void stopThreads();
    void sendLegCommand(int leg_index,
                       const Eigen::RowVector4f& tau,
                       const Eigen::RowVector4f& vel,
                       const Eigen::RowVector4f& pos,
                       const Eigen::RowVector4f& kp,
                       const Eigen::RowVector4f& kd);
    void sendAllMotorsCommand(const std::vector<float>& tau,
                          const std::vector<float>& vel,
                          const std::vector<float>& pos,
                          const std::vector<float>& kp,
                          const std::vector<float>& kd);                         
    void sendMotorCommand(int motor_global_idx,
                          float tau, 
                          float vel, 
                          float pos,            
                          float kp, 
                          float kd);

    Eigen::RowVector4f getMotorPos_leg(int leg_index);
    Eigen::RowVector4f getMotorVel_leg(int leg_index);
    Eigen::RowVector4f getMotorTorque_leg(int leg_index);
    Eigen::VectorXf getAllMotorPos();
    Eigen::VectorXf getAllMotorVel();
    Eigen::VectorXf getAllMotorTorque();
    Eigen::VectorXf getAllMotorTemp();    // 16 个电机温度(℃),无反馈为 -1
    std::vector<float> getAllMotorError();  // 获取所有电机错误码(协议无此字段,恒 0)

    // 新增：一次性读取所有电机反馈（原子操作，保证数据一致性）
    struct AllMotorFeedback {
        Eigen::VectorXf pos;   // 16个电机位置
        Eigen::VectorXf vel;   // 16个电机速度
        Eigen::VectorXf tau;   // 16个电机力矩
        std::vector<float> error; // 16个电机错误码(恒 0)
        Eigen::VectorXf temp;  // 16个电机温度℃,无反馈为 -1
    };
    AllMotorFeedback getAllMotorFeedback();
    // 零分配版本:调用方预分配 out 的 pos/vel/tau(16) 和 error(16),本函数仅拷贝数据。
    // 200Hz 控制环必须用此版本,避免每周期 Eigen::VectorXf resize 触发堆分配。
    void getAllMotorFeedback(AllMotorFeedback& out);

    // ── L1/L2 CAN 链路与反馈新鲜度(只读,供遥测使用,不参与控制环)──
    // L1: 某物理电机(0..15)最近一次有效反馈是否在 timeout_ms 内
    bool isMotorFeedbackFresh(int global_idx, int timeout_ms = 500);
    // L1: 某物理电机(0..15)距上次有效反馈的毫秒数;从未收到返回 -1(供看板显示时间戳)
    int64_t motorFeedbackAgeMs(int global_idx);
    // L2: 某物理电机(0..15)所属 CAN 链路号(can_idx 0..3);映射无效返回 -1
    int motorCanIdx(int global_idx);
    // L2: CAN 链路数量(=4)
    int canLinkCount();
    // L2: 某 CAN 链路(can_idx 0..3)距上次收到任意合法帧的毫秒数;从未收到返回 -1
    int64_t canLinkLastRxAgeMs(int can_idx);




    void enableMotor(int motor_id);
    void disableMotor(int motor_id);
    /** 单电机可控性探针(只读参考):发小角度偏移后检测反馈是否跟踪。
     *  腿关节:kp=80/kd=3/pos=当前+0.05rad,持续 500ms,判据 |Δq|>0.005rad;
     *  小腿关节:在腿关节基础上明确上抬 0.10rad(前负后正,轮端可见抬升);
     *  轮关节:kp=0/kd=4/vel=0.5rad/s,持续 800ms,判据 |Δq|>0.03rad(净位移);
     *  轮探针先激活同腿小腿保持探针抬腿离地再转轮,结束后释放并等 200ms 回落。
     *  探针结束后腿关节目标回位 150ms,避免累积爬移。
     *  仅返回是否可控,不触发任何恢复动作(结果仅供参考展示)。
     *  @param keep_calf_lifted 小腿探完不回落、保持抬起,由随后同腿轮探针复用
     *                          并释放(批量探针用,避免"抬两次")
     *  @param calf_pre_lifted 轮探针前小腿已被抬起,跳过自抬与 400ms 等待
     *  @return true=反馈跟踪(可控);false=无响应(未使能/故障)。阻塞约 650ms。 */
    bool probeMotorCheck(int global_idx,
                         bool keep_calf_lifted = false,
                         bool calf_pre_lifted = false);
    /** 全 16 台探针:4 腿并行(单腿 4 台串行,约 2.6s),失败台冷却 300ms 后
     *  逐台串行自动重测(消除批量并行供电跌落/CAN 拥挤的假阴性),以重测为准。
     *  @return 各电机是否可控的向量(下标=global_idx, true=可控) */
    std::vector<bool> probeAllMotorsCheck();
    /** 单腿 4 台探针,失败台自动串行重测一轮。
     *  @param leg 腿号 0=FL 1=FR 2=RL 3=RR
     *  @return 该腿 4 台电机是否可控的向量(下标=leg_motor_idx 0-3) */
    std::vector<bool> probeLegCheck(int leg);
    /** 是否已执行过探针检测(有历史结果) */
    bool hasProbeResults() const;
    /** 全部 16 台电机探针是否均通过(有结果且全 ok) */
    bool areAllMotorsProbedOk() const;
    /** 作废全部探针结果:趴下后调用,探针记录不允许跨趴下周期沿用(趴着时关节
     *  可能被人工搬动/电机断电重连,旧"可控"结论对新趴姿不再可靠)。
     *  @return 作废前是否存在有效探针记录(供调用方决定是否需要提示) */
    bool invalidateProbeResults();
    /** 单电机置零(线程安全+反馈确认):0xFE 后等待 |pos|<=0.05rad,未确认自动重发 */
    void setZeroMotor(int motor_id);
    void enableAllMotors();
    void disableAllMotors();
    void setZeroAllMotors();
    /**
     * @brief 可靠批量置零:四腿并行 + 反馈确认 + 自动重发
     * @param ids global motor id 列表(0..15,重复自动去重)
     * @return 经全部尝试(首发+2 次重发)后仍未确认的电机 id,升序;空向量=全部成功
     * 按 can_idx 分 4 路并行;每台发 0xFE 后轮询反馈(80ms 窗口,反馈 380~460fps
     * 可采样 30+ 帧),未掉线但位置仍 >0.05rad 判失败重发。掉线/无反馈电机自然进失败列表。
     */
    std::vector<int> setZeroMotorsVerified(const std::vector<int>& ids);
    /** 周期性重使能:每 5 秒由控制环调用。对所有电机重发 0xFC,
     *  覆盖迟到上电的电机(电源顺序问题)或丢失使能帧的情况。
     *  对已使能电机幂等无害。20ms 间距(16×20ms=320ms)避免占用过久。 */
    void periodicReEnable();

private:
    void legThreadFunc(int leg_index);
    void sendControlCommand(int can_idx, uint32_t can_id, uint8_t cmd);
    // 置零验证:读单台最新反馈角(腿反馈锁);轮询直到 |pos|<=0.05rad 且反馈新鲜
    float readMotorPosOnce(int global_idx);
    bool verifyMotorZeroed(int global_idx, int timeout_ms);
    /** 探针回位:把探针目标改回探针前基线,闭环轮询反馈直到 |pos-target|<=tol
     *  或超时。轮关节为速度探针,只做 0 速停转(无位置基线)。
     *  探针槽释放后上层 Passive 是 kp=0/kd=8 纯阻尼,腿无位置保持力,单靠重力
     *  回落会停在偏移位,故必须先闭环拉回再释放。
     *  @return true=已到位(或该关节没有可回位的位置目标) */
    bool restoreProbePosition(int global_idx, float target,
                              int timeout_ms = 600, float tol = 0.01f);

    std::atomic<bool> running_{false};
    std::vector<std::thread> leg_threads_;
    std::vector<std::mutex> fb_mutex_;
    std::mutex global_fb_mutex_;  // 新增：全局反馈锁，用于getAllMotorFeedback
    std::vector<LegFeedback> leg_feedbacks_;
    // C1-fix: 双缓冲消除 sendAllMotorsCommand ↔ legThreadFunc 数据竞争
    //   - leg_cmd_buffers_[leg][0/1]: 每条腿 2 个 LegCommand 缓冲区
    //   - active_cmd_idx_[leg]: 原子索引,0 或 1,指示 CAN 线程当前应读的 front buffer
    //   - 写线程写 back buffer(1-active) 后原子交换;读线程 load(acquire) 后读 front
    //   - 无锁,保证 CAN 线程永远读到完整一致的一帧(不会半旧半新)
    std::array<std::array<LegCommand, 2>, 4> leg_cmd_buffers_;
    std::array<std::atomic<int>, 4> active_cmd_idx_; 

    USB2CANManager can_manager_;
    std::vector<int> can_device_indices_;
    std::vector<std::shared_ptr<SerialPort>> serials_;
    std::vector<std::shared_ptr<SocketCANPort>> can_ports_;  // socketcan 模式使用
    std::string can_transport_ = "usb2can";  // 当前传输层,init() 写入,sendControlCommand 分流用
    std::vector<MotorMap> motor_map_;
    std::mutex api_mutex_;

    // 可控性探针覆写槽:active 时 legThreadFunc 对该电机用探针指令而非上层指令槽。
    // (上层 Passive.Run 每帧覆写指令槽,直接改槽会被冲掉;覆写发生在发送侧才有效)
    struct EnableProbe {
        std::atomic<bool> active{false};
        float kp = 0, kd = 0, pos = 0, vel = 0;
    };
    std::array<EnableProbe, 16> enable_probes_;

    // 探针结果缓存:global_idx → true(可控)/false(无响应);空表示未检测
    std::array<std::atomic<bool>, 16> probe_results_;
    std::array<std::atomic<bool>, 16> probe_results_valid_;

    // 探针前基线(结束后闭环回位用):小腿保持抬起时跨"小腿探针→轮探针"
    // 复用同一基线;探针槽释放时清 valid,下次探针(如失败重测)重取当前位。
    // 仅各腿探针线程按腿写读,无并发冲突,无需原子保护。
    std::array<float, 16> probe_home_{};
    std::array<bool, 16> probe_home_valid_{};

    static constexpr float P_MIN = -12.57f;
    static constexpr float P_MAX = 12.57f;
    static constexpr float V_MIN = -20.0f;
    static constexpr float V_MAX = 20.0f;
    static constexpr float T_MIN = -60.0f;
    static constexpr float T_MAX = 60.0f;
    std::vector<int> connected_motors_ = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};  // 16个电机全部使能
};

#endif // MOTOR_MANAGER_H