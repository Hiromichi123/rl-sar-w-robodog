#pragma once

// ============================================================================
// web_bridge —— 内置 WebSocket + 静态文件服务器（替代 rosbridge/ROS2 话题层）
//
// 功能：
//   1. 同一端口提供 HTTP 静态文件服务（Web 调试台页面）与 WebSocket 服务
//      （路径 /ws，RFC6455 文本帧），单进程替代原 rosbridge(:9090)+web(:8088)。
//   2. 入站(JSON)经有界队列转交回调（独立分发线程，等价 ROS 单线程 executor）。
//   3. 出站(JSON)广播给所有已连接 WS 客户端；发布端永不阻塞控制循环
//      （非阻塞写，EAGAIN 丢帧；慢客户端超写缓冲上限自动断开）。
//
// 线协议：均为单行 JSON 文本帧。
//   Web→机器人: {"type":"cmd_vel","vx":..,"vy":..,"yaw":..}
//              {"type":"command","data":"getup"}          (data 同原 std_msgs/String)
//              {"type":"imu_stream","data":true}
//              {"type":"notify_ack","data":"{\"id\":1,\"ok\":true}"}
//              {"type":"check_stand","data":"check"}
//              {"type":"briefing","data":"<场景名|stop|list>"}
//   机器人→Web: {"type":"motor_state","data":[..]}        (data 同原 Float32MultiArray.data)
//              {"type":"imu_state","data":[..]}
//              {"type":"pose2d","data":[..]}
//              {"type":"feedback","data":"<text>"}
//              {"type":"notify","data":"{\"id\":..,...}"} (data 同原 JSON 字符串)
//              {"type":"check_stand_result","data":"{..}"}
//              {"type":"briefing_status","data":"<text>"}
// ============================================================================

#include <cstddef>
#include <functional>
#include <string>
#include <vector>

// 底盘速度指令（原 geometry_msgs/Twist 中被使用的三个分量）
struct CmdVel
{
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
};

class WebBridge
{
public:
    using CmdVelCb = std::function<void(const CmdVel &)>;
    using StringCb = std::function<void(const std::string &)>;
    using BoolCb = std::function<void(bool)>;
    using VoidCb = std::function<void()>;

    WebBridge();
    ~WebBridge();

    WebBridge(const WebBridge &) = delete;
    WebBridge &operator=(const WebBridge &) = delete;

    /**
     * 启动服务器（监听 0.0.0.0:port）。
     * @param port        HTTP + WebSocket 同端口（生产默认 8088）
     * @param web_root    静态文件根目录（Web 调试台目录）
     * @param on_*        入站回调（在独立分发线程按序执行，不要做长时间阻塞以外的事）
     * @return 成功返回 true；端口占用等失败返回 false
     */
    bool start(int port, const std::string &web_root,
               CmdVelCb on_cmd_vel, StringCb on_command, BoolCb on_imu_stream,
               StringCb on_notify_ack, VoidCb on_check_stand, StringCb on_briefing);
    void stop();

    // ---- 出站广播（线程安全，可在任意线程调用，绝不阻塞） ----
    void publishMotorState(const std::vector<float> &data);
    void publishImuState(const std::vector<float> &data);
    void publishPose2D(const std::vector<float> &data);
    void publishFeedback(const std::string &text);
    void publishNotify(const std::string &json);          // data = 已拼装的 JSON 字符串
    void publishCheckStandResult(const std::string &json);
    void publishBriefingStatus(const std::string &text);

    size_t clientCount() const;

private:
    struct Impl;
    Impl *impl_;
};
