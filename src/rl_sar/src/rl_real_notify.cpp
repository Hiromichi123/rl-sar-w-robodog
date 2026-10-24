// C3-fix: 通知系统模块(从 rl_real_JXG.cpp 拆分)
// 包含: Notify / FlushNotifyQueue / PublishNotify / NotifyAckCallback /
//       HandleNotifyAck / ProcessNotifyRetries
// 依赖: rl_real_JXG.hpp 中声明的成员变量(notify_seq_, async_notify_queue_,
//       notify_publisher, web_bridge_, notify_mutex_, notify_pending_, notify_last_retry_ts_)

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

static double NotifySteadyNowSec()
{
    return std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

void RL_Real::Notify(const std::string &type, const std::string &content)
{
    // C2-fix: 实时线程只构建 JSON 并推入无锁队列,不阻塞在 ROS publish / WebSocket send / mutex
    uint32_t id = ++this->notify_seq_;
    double now_sec =
#if JXG_ROS_LAYER && !JXG_WEB_LAYER
        this->now().seconds();
#else
        NotifySteadyNowSec();
#endif

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

    this->async_notify_queue_.Push(oss.str());
}

void RL_Real::FlushNotifyQueue()
{
    // C2-fix: 非实时线程(50ms timer)批量取出队列中的 JSON,实际执行 ROS publish / WebSocket send / mutex 更新
    std::string json;
    while (this->async_notify_queue_.Pop(json)) {
        // 从 JSON 中提取 id 和 timestamp(用于 notify_pending_ 跟踪)
        uint32_t id = 0;
        double ts = 0.0;
        size_t id_pos = json.find("\"id\":");
        if (id_pos != std::string::npos) {
            size_t start = id_pos + 5;
            size_t end = json.find(',', start);
            if (end != std::string::npos) {
                id = static_cast<uint32_t>(std::stoul(json.substr(start, end - start)));
            }
        }
        size_t ts_pos = json.find("\"timestamp\":");
        if (ts_pos != std::string::npos) {
            size_t start = ts_pos + 12;
            size_t end = json.find('}', start);
            if (end != std::string::npos) {
                ts = std::stod(json.substr(start, end - start));
            }
        }

#if JXG_ROS_LAYER
        if (this->notify_publisher) {
            std_msgs::msg::String msg;
            msg.data = json;
            this->notify_publisher->publish(msg);
        }
#endif
#if JXG_WEB_LAYER
        if (this->web_bridge_) this->web_bridge_->publishNotify(json);
#endif

        if (id > 0) {
            NotifyRecord rec;
            rec.json = json;
            rec.timestamp = ts;
            rec.retry_count = 0;
            std::lock_guard<std::mutex> lock(this->notify_mutex_);
            this->notify_pending_[id] = rec;
        }
    }
}

void RL_Real::PublishNotify(const std::string &type, const std::string &content)
{
    // 兼容旧接口:直接调用 Notify(异步)
    this->Notify(type, content);
}

#if JXG_ROS_LAYER
void RL_Real::NotifyAckCallback(const std_msgs::msg::String::SharedPtr msg)
{
    this->HandleNotifyAck(msg->data);
}
#endif

void RL_Real::HandleNotifyAck(const std::string &s)
{
    // 解析 {"id":N,"ok":true}
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
#if JXG_ROS_LAYER
            if (this->notify_publisher)
            {
                std_msgs::msg::String m;
                m.data = it->second.json;
                this->notify_publisher->publish(m);
            }
#endif
#if JXG_WEB_LAYER
            // v4 共存: 重试也双侧重发
            if (this->web_bridge_) this->web_bridge_->publishNotify(it->second.json);
#endif
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
