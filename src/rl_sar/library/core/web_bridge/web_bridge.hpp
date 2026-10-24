#ifndef WEB_BRIDGE_HPP
#define WEB_BRIDGE_HPP

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

// ============================================================================
// WebBridge:内置 HTTP + WebSocket 桥(无第三方依赖)
//
// 单端口同时提供:
//   1) 静态页面(GET /,3D 仿真 web 控制台)
//   2) 控制/遥测 WebSocket(GET 升级 /ws)
//   3) 文件管理 HTTP API(/api/fs/*,供系统管理页签的 WebFTP 使用)
//
// WS 入站消息(JSON,一行一帧):
//   {"type":"cmd_vel","vx":0.0,"vy":0.0,"yaw":0.0}
//   {"type":"command","data":"<命令文本>","seq":<int>}      # seq 可选
//   {"type":"imu_stream","data":true|false}
//   {"type":"notify_ack","data":"{\"id\":..,\"ok\":true}"}
//   {"type":"check_stand"}
//   ── 实时日志页签 ──
//   {"type":"log_subscribe","data":{"n":300}}   # 订阅程序日志(最近 n 行 + 实时跟随)
//   {"type":"log_unsubscribe"}
//   ── WebSSH(系统管理页签,需通过鉴权) ──
//   {"type":"ssh_start","data":{"cols":100,"rows":30}}
//   {"type":"ssh_data","data":"<按键/粘贴文本>"}
//   {"type":"ssh_resize","data":{"cols":100,"rows":30}}
//   {"type":"ssh_stop"}
//
// WS 出站消息(JSON):
//   motor_state / imu_state : data=[float,...]
//   pose2d                  : data=[x,y,z,yaw,vel_x,vel_y,yaw_rate]
//   feedback                : data=string
//   notify                  : data=<JSON 对象字符串>
//   cmd_state               : data={seq,cmd,code,phase,target,state,elapsed_ms}
//                            # seq=0 为 ROS/无序号客户端的合成生命周期(大脑桥据此转 /rl_real/cmd_state)
//   heartbeat(2Hz)          : data={seq,state,nav,uptime_ms,active_seq}
//   check_stand_result      : data="{...}"
//   posture                 : data={<JSON 对象>}
//   check_standing_result   : data="{...}"
//   log                     : data="一行日志"     # 实时日志页签
//   log_status              : data="<文本>"
//   ssh_ready / ssh_exit    : 终端会话生命周期
//   ssh_data                : data="PTY 输出"
//
// 鉴权(WebSSH / WebFTP / 电源与日志开关等系统管理操作):
//   环境变量 WEB_ADMIN_TOKEN 非空时,WS 握手需带 ?token=<值>(/ws?token=xxx),
//   HTTP API 需带 ?token=<值> 或 X-Admin-Token 头;未设置环境变量时沿用
//   "局域网可信"模型放行(与原页面行为一致,启动时会打印安全告警)。
// ============================================================================

class WebBridge
{
public:
    struct CmdVel
    {
        double x = 0.0;
        double y = 0.0;
        double yaw = 0.0;
    };

    using CmdVelCb  = std::function<void(const CmdVel &)>;
    using CommandCb = std::function<void(const std::string &text, int seq)>;
    using BoolCb    = std::function<void(bool)>;
    using StringCb  = std::function<void(const std::string &)>;
    using VoidCb    = std::function<void()>;

    WebBridge();
    ~WebBridge();

    /// 启动 HTTP + WS 服务(0.0.0.0:port)。web_root 为静态页面根目录。
    bool start(int port, const std::string &web_root,
               CmdVelCb on_cmd_vel, CommandCb on_command, BoolCb on_imu_stream,
               StringCb on_notify_ack, VoidCb on_check_stand);

    void stop();

    // ---- 遥测/事件广播(由 rl_real 控制线程调用,内部加锁) ----
    void publishMotorState(const std::vector<float> &data);
    void publishImuState(const std::vector<float> &data);
    void publishPose2D(const std::vector<float> &data);
    void publishFeedback(const std::string &text);
    void publishNotify(const std::string &json);
    // 逐字广播已构造好的 JSON 文本帧(cmd_state / heartbeat 由 rl_real 统一构造,
    // ROS 话题与 WebSocket 共用同一帧,保证大脑下游解析契约一致)
    void broadcastFrame(const std::string &frame);
    void publishCheckStandResult(const std::string &json);
    void publishPosture(const std::string &json);
    void publishCheckStandingResult(const std::string &json);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

#endif  // WEB_BRIDGE_HPP
