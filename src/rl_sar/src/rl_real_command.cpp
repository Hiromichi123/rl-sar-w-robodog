// C3-fix: Web/ROS 命令处理模块(从 rl_real_JXG.cpp 拆分)
// 包含: IsWebExclusiveActive / ApplyWebVelocity / CmdvelCallback /
//       WebCmdVelRosCallback / WebCmdvelCallback / CommandCallback /
//       ImuStreamCallback / ApplyRosCommand / EmitCommandState /
//       MaybePublishHeartbeat / TrackFsmCommand / UpdatePendingCommand
// 依赖: rl_real_JXG.hpp 中声明的成员变量与函数

#include "rl_real_JXG.hpp"

#include <sstream>
#include <unistd.h>

// 版本宏兜底(由 CMake target_compile_definitions 注入)
#ifndef APP_VERSION
#define APP_VERSION "dev"
#endif
#ifndef BUILD_NUMBER
#define BUILD_NUMBER "-"
#endif
#ifndef BUILD_TIMESTAMP
#define BUILD_TIMESTAMP "-"
#endif

// 全局变量(定义于 rl_real_JXG.cpp)
extern std::atomic<bool> g_shutdown_flag;

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

// ===== 静态辅助函数(与 rl_real_JXG.cpp 中的同名 static 函数语义一致) =====
static std::string CmdExecShell(const std::string &cmd)
{
    std::string out;
    FILE *fp = popen(cmd.c_str(), "r");
    if (!fp) return out;
    char buf[1024];
    while (fgets(buf, sizeof(buf), fp)) out.append(buf);
    pclose(fp);
    return out;
}

static bool CmdIsValidUnitName(const std::string &s)
{
    if (s.empty() || s.size() > 128) return false;
    for (char c : s)
        if (!(isalnum((unsigned char)c) || c == '.' || c == '-' || c == '_' || c == '@'))
            return false;
    return true;
}

static constexpr double kWebExclusiveHoldSec = 1.0;
static constexpr float kCmdVelDeadzone = 0.05f;

static inline float CmdApplyDeadzone(float v)
{
    return std::abs(v) < kCmdVelDeadzone ? 0.0f : v;
}

static std::string CmdJsonEscape(const std::string &s)
{
    std::string out = s;
    for (size_t p = 0; (p = out.find('\\', p)) != std::string::npos; p += 2)
        out.replace(p, 1, "\\\\");
    for (size_t p = 0; (p = out.find('"', p)) != std::string::npos; p += 2)
        out.replace(p, 1, "\\\"");
    return out;
}

static double CmdSteadyNowSec()
{
    return std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

namespace {
std::string CmdNormalizeCommand(std::string value)
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

// ===== 速度与独占仲裁 =====
#if JXG_ROS_LAYER || JXG_WEB_LAYER
bool RL_Real::IsWebExclusiveActive() const
{
    return std::chrono::duration<double>(
               std::chrono::steady_clock::now() - this->last_web_active_time_)
               .count() <= kWebExclusiveHoldSec;
}

void RL_Real::ApplyWebVelocity(float x, float y, float yaw)
{
    const auto now_vel = std::chrono::steady_clock::now();
    this->last_cmd_vel_time_ = now_vel;

    x   = CmdApplyDeadzone(x);
    y   = CmdApplyDeadzone(y);
    yaw = CmdApplyDeadzone(yaw);

    if (x != 0.0f || y != 0.0f || yaw != 0.0f)
    {
        this->last_web_active_time_ = now_vel;
    }

    this->ReceiveDiffVelocity(x, y, yaw);
    this->control.x   = x;
    this->control.y   = y;
    this->control.yaw = yaw;
    std::cout << LOGGER::INFO << "web cmd_vel: linear(" << x << ", " << y
              << ", 0) angular(0, 0, " << yaw << ")" << std::endl;
}
#endif

#if JXG_ROS_LAYER
void RL_Real::CmdvelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    if (this->diff_enabled_.load() || this->IsWebExclusiveActive())
    {
        const auto now_vel = std::chrono::steady_clock::now();
        static std::chrono::steady_clock::time_point last_ignore_log{};
        if (std::chrono::duration<double>(now_vel - last_ignore_log).count() >= 1.0)
        {
            last_ignore_log = now_vel;
            PublishRosFeedback("cmd_vel ignored: web exclusive active");
        }
        return;
    }

    const auto now_vel = std::chrono::steady_clock::now();
    this->last_cmd_vel_time_ = now_vel;
    msg->linear.x  = CmdApplyDeadzone(static_cast<float>(msg->linear.x));
    msg->linear.y  = CmdApplyDeadzone(static_cast<float>(msg->linear.y));
    msg->angular.z = CmdApplyDeadzone(static_cast<float>(msg->angular.z));
    this->ReceiveDiffVelocity(msg->linear.x, msg->linear.y, msg->angular.z);
    this->cmd_vel = *msg;

    this->control.x = static_cast<float>(msg->linear.x);
    this->control.y = static_cast<float>(msg->linear.y);
    this->control.yaw = static_cast<float>(msg->angular.z);
    std::cout << LOGGER::INFO << "cmd_vel: linear("
              << msg->linear.x << ", " << msg->linear.y << ", " << msg->linear.z
              << ") angular("
              << msg->angular.x << ", " << msg->angular.y << ", " << msg->angular.z
              << ")" << std::endl;
    PublishRosFeedback("cmd_vel accepted");
}

void RL_Real::WebCmdVelRosCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    this->ApplyWebVelocity(static_cast<float>(msg->linear.x),
                           static_cast<float>(msg->linear.y),
                           static_cast<float>(msg->angular.z));
}
#endif

#if JXG_WEB_LAYER
void RL_Real::WebCmdvelCallback(const WebBridge::CmdVel &msg)
{
    this->ApplyWebVelocity(static_cast<float>(msg.x),
                           static_cast<float>(msg.y),
                           static_cast<float>(msg.yaw));
    PublishRosFeedback("cmd_vel accepted(web)");
}
#endif

#if JXG_ROS_LAYER
void RL_Real::CommandCallback(const std_msgs::msg::String::SharedPtr msg)
{
    const std::string cmd = CmdNormalizeCommand(msg->data);
    // Local Web owns differential mode; brain commands cannot acquire or replace it.
    if (cmd.rfind("diff_", 0) == 0 ||
        (this->diff_enabled_.load() && cmd != "getdown" && cmd != "vel_stop" &&
         cmd != "status" && cmd != "feedback" && cmd != "rl_block")) {
        PublishRosFeedback("command rejected: local differential control owns command channel");
        EmitCommandState(0, cmd, kRejected, std::string(), 0.0);
        return;
    }
    if (cmd == "passive" || cmd == "stop" || cmd == "safe_stop")
    {
        PublishRosFeedback("command rejected: passive hidden on ROS (use getdown)");
        EmitCommandState(0, "passive", kRejected, std::string(), 0.0);
        return;
    }
    ApplyRosCommand(msg->data);
}

void RL_Real::ImuStreamCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    SetJXGIMUStreamEnabled(msg->data);
    PublishRosFeedback(std::string("imu_stream=") + (msg->data ? "on" : "off"));
}
#endif

// ===== 命令处理核心 =====
void RL_Real::ApplyRosCommand(const std::string &command, int seq)
{
    const std::string cmd = CmdNormalizeCommand(command);
    if (cmd.empty()) return;

    if (seq > 0)
    {
        const double now_sec = CmdSteadyNowSec();
        std::lock_guard<std::mutex> lk(this->pending_cmd_mutex_);
        if (seq == this->last_cmd_seq_ && (now_sec - this->last_cmd_time_) < 1.5)
            return;
        this->last_cmd_seq_ = seq;
        this->last_cmd_time_ = now_sec;
    }

    const double t0 = CmdSteadyNowSec();
    auto emit = [&](int code, const std::string &target = std::string()) {
        this->EmitCommandState(seq, cmd, code, target, (CmdSteadyNowSec() - t0) * 1000.0);
    };
    emit(kReceived);

    if (cmd == "diff_enable" || cmd == "diff_disable" || cmd == "diff_drive" || cmd == "diff_exit")
    {
        const bool active = this->diff_active_.load();
        if (cmd == "diff_enable") {
            if (!this->diff_controller_ || !this->DiffMissionIdle()) {
                PublishRosFeedback("command rejected: diff_enable, controller unavailable or fresh mission is busy");
                emit(kRejected); return;
            }
            this->diff_enabled_.store(true);
            emit(kDone);
        } else if (cmd == "diff_disable" || cmd == "diff_exit") {
            this->ClearDiffVelocity();
            if (cmd == "diff_disable") this->diff_enabled_.store(false);
            if (active) {
                this->diff_request_.store(2);
                TrackFsmCommand(seq, cmd, "RLFSMStateGetUp", 5.0);
            } else { emit(kDone); }
        } else {
            if (!this->CanEnterDiffDrive()) {
                PublishRosFeedback("command rejected: diff_drive, require enabled, no fresh busy mission, settled GetUp and valid standing/CAN checks");
                emit(kRejected, "RLFSMStateDiffDrive");
                return;
            }
            this->ClearDiffVelocity();
            this->diff_request_.store(1);
            TrackFsmCommand(seq, cmd, "RLFSMStateDiffDrive", 5.0);
        }
        PublishRosFeedback("command accepted: " + cmd);
        return;
    }
    if (this->diff_active_.load() &&
        (cmd == "getup" || cmd == "stand" || cmd == "up" ||
         cmd == "locomotion" || cmd == "rl" || cmd == "walk" || cmd == "move" ||
         cmd.rfind("safe_getup", 0) == 0 || cmd.rfind("probe", 0) == 0)) {
        PublishRosFeedback("command rejected: exit differential drive first");
        emit(kRejected);
        return;
    }

    const bool zero_allowed_passive =
        this->fsm.GetCurrentStateName() == "RLFSMStatePassive";
    const bool is_zero_cmd =
        (cmd.size() > 11 && cmd.compare(0, 11, "zero_motor ") == 0) ||
        (cmd.size() > 12 && cmd.compare(0, 12, "zero_motors ") == 0) ||
        cmd == "zero" || cmd == "setzero" || cmd == "set_zero" ||
        cmd == "set_zero_all" || cmd == "zero_all";
    if (is_zero_cmd && !zero_allowed_passive)
    {
        PublishRosFeedback("command rejected: " + cmd +
                           ", zero only allowed in Passive state (send passive/stop first)");
        this->Notify("error", "置零被拒绝：仅趴下/被动态(Passive)允许置零，请先发送 passive/stop");
        emit(kRejected);
        return;
    }

    auto parse_zero_ids = [](const std::string& s, std::vector<int>* out) -> std::string {
        out->clear();
        size_t i = 0;
        bool any = false;
        while (i < s.size())
        {
            while (i < s.size() && (s[i] == ' ' || s[i] == ',')) ++i;
            if (i >= s.size()) break;
            if (s[i] < '0' || s[i] > '9') return "invalid id list, expected 'zero_motors <0..15 comma-separated>'";
            int v = 0, digits = 0;
            while (i < s.size() && s[i] >= '0' && s[i] <= '9')
            {
                v = v * 10 + (s[i] - '0');
                ++i;
                ++digits;
            }
            if (digits > 2 || v >= 16) return "motor id out of range (0..15)";
            out->push_back(v);
            any = true;
            while (i < s.size() && s[i] == ' ') ++i;
            if (i < s.size() && s[i] != ',') return "invalid separator, expected comma";
        }
        return any ? std::string() : std::string("empty id list");
    };

    auto publish_zero_result = [&](const std::vector<int>& requested,
                                   const std::vector<int>& failed) -> bool {
        this->motors_ready_for_stand = this->CheckMotorInitPosition(1.0f);
        std::ostringstream oss;
        oss << "zero_result {\"zeroed\":[";
        bool first = true;
        for (int id : requested)
        {
            if (std::find(failed.begin(), failed.end(), id) != failed.end()) continue;
            if (!first) oss << ',';
            oss << id;
            first = false;
        }
        oss << "],\"failed\":[";
        for (size_t i = 0; i < failed.size(); ++i)
        {
            if (i) oss << ',';
            oss << failed[i];
        }
        oss << "],\"motors_ready\":" << (this->motors_ready_for_stand ? "true" : "false")
            << ",\"reason\":\"" << CmdJsonEscape(this->motors_not_ready_reason) << "\"}";
        PublishRosFeedback(oss.str());
        return failed.empty();
    };

    if (cmd == "can_recover" || cmd == "can_bounce" || cmd == "can_reset")
    {
        emit(kExecuting);
        this->Notify("info", "正在恢复 CAN 接口(运行 can_setup.sh)...");
        PublishRosFeedback("can_recover: executing can_setup.sh");
        std::thread([this]() {
            const char* script = nullptr;
            const char* candidates[] = {
                "/home/jinjiao/rl_sar-w2_recovered_v7.6/scripts/can_setup.sh",
                "/opt/rl_sar/scripts/can_setup.sh",
            };
            for (auto p : candidates) {
                if (std::ifstream(p).good()) { script = p; break; }
            }
            if (!script) {
                this->Notify("error", "can_recover 失败: 找不到 can_setup.sh");
                return;
            }
            std::string cmd_str = std::string("bash ") + script + " 2>&1";
            FILE* pipe = popen(cmd_str.c_str(), "r");
            if (!pipe) {
                this->Notify("error", "can_recover 失败: popen 失败");
                return;
            }
            std::string output;
            char buf[256];
            while (fgets(buf, sizeof(buf), pipe)) output += buf;
            int rc = pclose(pipe);
            if (rc == 0) {
                this->Notify("success", "CAN 接口恢复完成,等待电机反馈自动恢复通信");
                PublishRosFeedback("can_recover: done");
            } else {
                this->Notify("error", "can_recover: can_setup.sh 返回 " + std::to_string(rc) + "\n" + output.substr(0, 500));
                PublishRosFeedback("can_recover: failed (rc=" + std::to_string(rc) + ")");
            }
        }).detach();
        emit(kDone);
        return;
    }
    else if (cmd == "sys_reboot" || cmd == "reboot")
    {
        PublishRosFeedback("system reboot in 1s ...");
        std::cout << LOGGER::WARNING << "Web/ROS 触发系统重启(1s 后)" << std::endl;
        emit(kExecuting);
        std::system("( sleep 1; systemctl reboot ) >/dev/null 2>&1 &");
        return;
    }
    if (cmd == "sys_poweroff" || cmd == "poweroff" || cmd == "shutdown")
    {
        PublishRosFeedback("system poweroff in 1s ...");
        std::cout << LOGGER::WARNING << "Web/ROS 触发系统关机(1s 后)" << std::endl;
        emit(kExecuting);
        std::system("( sleep 1; systemctl poweroff ) >/dev/null 2>&1 &");
        return;
    }

    {
        auto split2 = [](const std::string &s, std::string *a, std::string *b) {
            size_t sp = s.find(' ');
            if (sp == std::string::npos) { *a = s; *b = ""; }
            else { *a = s.substr(0, sp); *b = s.substr(sp + 1); }
        };
        std::string head, arg;
        split2(cmd, &head, &arg);

        auto sendSvcList = [&]() {
            std::string raw = CmdExecShell(
                "systemctl list-units --type=service --all --no-pager --no-legend 2>/dev/null");
            std::ostringstream oss;
            oss << "{\"type\":\"svc_list\",\"data\":{\"units\":[";
            bool first = true;
            std::istringstream iss(raw);
            std::string line;
            while (std::getline(iss, line))
            {
                if (line.empty()) continue;
                std::istringstream ls(line);
                std::string u, load, active, sub2, desc;
                if (!(ls >> u >> load >> active >> sub2)) continue;
                std::getline(ls, desc);
                size_t ds = desc.find_first_not_of(" \t");
                if (ds != std::string::npos) desc = desc.substr(ds);
                size_t de = desc.find_last_not_of(" \t\r\n");
                if (de != std::string::npos) desc = desc.substr(0, de + 1);
                if (!first) oss << ",";
                first = false;
                oss << "{\"unit\":\"" << CmdJsonEscape(u) << "\""
                    << ",\"load\":\"" << CmdJsonEscape(load) << "\""
                    << ",\"active\":\"" << CmdJsonEscape(active) << "\""
                    << ",\"sub\":\"" << CmdJsonEscape(sub2) << "\""
                    << ",\"desc\":\"" << CmdJsonEscape(desc) << "\"}";
            }
            oss << "]}}";
            if (this->web_bridge_) this->web_bridge_->broadcastFrame(oss.str());
        };

        if (head == "svc" || cmd.compare(0, 4, "svc ") == 0)
        {
            std::string sub, rest;
            split2(arg, &sub, &rest);

            if (sub == "list" || sub == "status")
            {
                sendSvcList();
                emit(kDone);
                return;
            }
            else if (sub == "start" || sub == "stop" || sub == "restart" ||
                     sub == "enable" || sub == "disable")
            {
                if (!CmdIsValidUnitName(rest)) { emit(kRejected); return; }
                CmdExecShell("systemctl " + sub + " " + rest + " 2>&1");
                PublishRosFeedback("svc " + sub + " " + rest + ": done");
                sendSvcList();
                emit(kDone);
                return;
            }
            else if (sub == "log")
            {
                if (!CmdIsValidUnitName(rest)) { emit(kRejected); return; }
                std::string raw = CmdExecShell(
                    "journalctl -u " + rest + " -n 200 --no-pager 2>/dev/null");
                std::ostringstream oss;
                oss << "{\"type\":\"svc_log\",\"data\":{\"unit\":\"" << CmdJsonEscape(rest)
                    << "\",\"lines\":[";
                bool first = true;
                std::istringstream iss(raw);
                std::string line;
                while (std::getline(iss, line))
                {
                    if (!first) oss << ",";
                    first = false;
                    oss << "\"" << CmdJsonEscape(line) << "\"";
                }
                oss << "]}}";
                if (this->web_bridge_) this->web_bridge_->broadcastFrame(oss.str());
                emit(kDone);
                return;
            }
            else if (sub == "file")
            {
                if (!CmdIsValidUnitName(rest)) { emit(kRejected); return; }
                std::string path = "/etc/systemd/system/" + rest;
                std::ifstream f(path);
                std::string content((std::istreambuf_iterator<char>(f)),
                                     std::istreambuf_iterator<char>());
                std::ostringstream oss;
                oss << "{\"type\":\"svc_file\",\"data\":{\"unit\":\"" << CmdJsonEscape(rest)
                    << "\",\"content\":\"" << CmdJsonEscape(content) << "\"}}";
                if (this->web_bridge_) this->web_bridge_->broadcastFrame(oss.str());
                emit(kDone);
                return;
            }
            else if (sub == "delete")
            {
                if (!CmdIsValidUnitName(rest)) { emit(kRejected); return; }
                CmdExecShell("systemctl disable " + rest + " 2>&1; rm -f /etc/systemd/system/" + rest);
                CmdExecShell("systemctl daemon-reload 2>&1");
                PublishRosFeedback("svc delete " + rest + ": done");
                sendSvcList();
                emit(kDone);
                return;
            }
        }
        else if (head == "ps" || cmd.compare(0, 3, "ps ") == 0)
        {
            std::string sub, rest;
            split2(arg, &sub, &rest);
            if (sub == "list")
            {
                std::string raw = CmdExecShell("ps aux --no-headers 2>/dev/null");
                std::ostringstream oss;
                oss << "{\"type\":\"ps_list\",\"data\":{\"procs\":[";
                bool first = true;
                std::istringstream iss(raw);
                std::string line;
                while (std::getline(iss, line))
                {
                    if (line.empty()) continue;
                    std::istringstream ls(line);
                    std::string user, pid, cpu, mem, vsz, rss, tty, stat, start, time_, comm;
                    if (!(ls >> user >> pid >> cpu >> mem >> vsz >> rss >> tty >> stat >> start >> time_)) continue;
                    std::getline(ls, comm);
                    size_t cs = comm.find_first_not_of(" \t");
                    if (cs != std::string::npos) comm = comm.substr(cs);
                    if (!first) oss << ",";
                    first = false;
                    oss << "{\"pid\":" << pid
                        << ",\"user\":\"" << CmdJsonEscape(user) << "\""
                        << ",\"cpu\":" << cpu
                        << ",\"mem\":" << mem
                        << ",\"rss\":" << rss
                        << ",\"stat\":\"" << CmdJsonEscape(stat) << "\""
                        << ",\"comm\":\"" << CmdJsonEscape(comm) << "\""
                        << ",\"cmd\":\"" << CmdJsonEscape(comm) << "\"}";
                }
                oss << "]}}";
                if (this->web_bridge_) this->web_bridge_->broadcastFrame(oss.str());
                emit(kDone);
                return;
            }
            else if (sub == "kill" || sub == "kill9")
            {
                if (rest.empty() || rest.find_first_not_of("0123456789") != std::string::npos)
                { emit(kRejected); return; }
                std::string sig = (sub == "kill9") ? "9" : "TERM";
                CmdExecShell("kill -" + sig + " " + rest + " 2>&1");
                PublishRosFeedback("ps " + sub + " " + rest + ": done");
                emit(kDone);
                return;
            }
            else if (sub == "start")
            {
                if (rest.empty()) { emit(kRejected); return; }
                CmdExecShell("nohup " + rest + " >/dev/null 2>&1 &");
                PublishRosFeedback("ps start: done");
                emit(kDone);
                return;
            }
        }
        else if (cmd == "sysinfo")
        {
            std::string uptime_raw = CmdExecShell("cat /proc/uptime 2>/dev/null");
            double uptime_s = 0.0;
            std::istringstream(uptime_raw) >> uptime_s;
            std::string load_raw = CmdExecShell("cat /proc/loadavg 2>/dev/null");
            double load1 = 0, load5 = 0, load15 = 0;
            { std::istringstream ls(load_raw); ls >> load1 >> load5 >> load15; }
            std::string mem_raw = CmdExecShell("cat /proc/meminfo 2>/dev/null");
            long mem_total_kb = 0, mem_avail_kb = 0, swap_total_kb = 0, swap_free_kb = 0;
            {
                std::istringstream ms(mem_raw);
                std::string line;
                while (std::getline(ms, line))
                {
                    std::istringstream ls(line);
                    std::string key; long val;
                    if (ls >> key >> val)
                    {
                        if (key == "MemTotal:") mem_total_kb = val;
                        else if (key == "MemAvailable:") mem_avail_kb = val;
                        else if (key == "SwapTotal:") swap_total_kb = val;
                        else if (key == "SwapFree:") swap_free_kb = val;
                    }
                }
            }
            long mem_used_kb = mem_total_kb - mem_avail_kb;
            long swap_used_kb = swap_total_kb - swap_free_kb;
            std::string top_raw = CmdExecShell("top -bn1 2>/dev/null | grep -i 'Cpu(s)'");
            double cpu_pct = 0.0;
            {
                size_t p = top_raw.find(':');
                if (p != std::string::npos)
                {
                    std::string rest = top_raw.substr(p + 1);
                    double us = 0, sy = 0;
                    std::istringstream cs(rest);
                    std::string tok;
                    if (cs >> us) { }
                    cs >> tok;
                    if (cs >> sy) { }
                    cpu_pct = us + sy;
                }
            }
            std::string ps_raw = CmdExecShell("ps -e --no-headers 2>/dev/null | wc -l");
            int tasks = 0;
            std::istringstream(ps_raw) >> tasks;
            std::string df_raw = CmdExecShell("df -k 2>/dev/null");
            std::ostringstream disks;
            disks << "[";
            {
                std::istringstream ds(df_raw);
                std::string line;
                bool first = true;
                bool header = true;
                while (std::getline(ds, line))
                {
                    if (header) { header = false; continue; }
                    if (line.empty()) continue;
                    std::istringstream ls(line);
                    std::string fs, total, used, avail, pct, mount;
                    if (!(ls >> fs >> total >> used >> avail >> pct >> mount)) continue;
                    if (fs.find("tmpfs") != std::string::npos || fs.find("devtmpfs") != std::string::npos)
                        continue;
                    if (!first) disks << ",";
                    first = false;
                    disks << "{\"fs\":\"" << CmdJsonEscape(fs) << "\""
                          << ",\"mount\":\"" << CmdJsonEscape(mount) << "\""
                          << ",\"total_kb\":" << total
                          << ",\"used_kb\":" << used << "}";
                }
            }
            disks << "]";

            std::ostringstream oss;
            oss << "{\"type\":\"sys_info\",\"data\":{"
                << "\"uptime_s\":" << (long)uptime_s
                << ",\"load1\":" << load1
                << ",\"load5\":" << load5
                << ",\"load15\":" << load15
                << ",\"cpu_pct\":" << cpu_pct
                << ",\"mem_total_kb\":" << mem_total_kb
                << ",\"mem_used_kb\":" << mem_used_kb
                << ",\"swap_total_kb\":" << swap_total_kb
                << ",\"swap_used_kb\":" << swap_used_kb
                << ",\"tasks\":" << tasks
                << ",\"disks\":" << disks.str()
                << "}}";
            if (this->web_bridge_) this->web_bridge_->broadcastFrame(oss.str());
            emit(kDone);
            return;
        }
    }

    if (cmd == "log" || cmd == "log on" || cmd == "log off" || cmd == "log status")
    {
#ifdef ENABLE_HR_LOGGER
        if (cmd == "log on")
        {
            if (!this->hr_file.is_open()) this->HrLogInit();
            this->hr_log_enabled_.store(true);
            PublishRosFeedback("hr_logger: on (200Hz CSV 记录已开启)");
        }
        else if (cmd == "log off")
        {
            this->hr_log_enabled_.store(false);
            PublishRosFeedback("hr_logger: off (200Hz CSV 记录已暂停)");
        }
        else
        {
            PublishRosFeedback(std::string("hr_logger: ") +
                               (this->hr_log_enabled_.load() ? "on" : "off"));
        }
#else
        PublishRosFeedback("hr_logger: not compiled (ENABLE_HR_LOGGER undefined)");
#endif
        emit(kDone);
        return;
    }

    if (cmd == "safe_getup" || cmd.rfind("safe_getup ", 0) == 0)
    {
        bool want_rl = false;
        if (cmd.size() > 11)
        {
            std::string arg = cmd.substr(11);
            while (!arg.empty() && (arg.front() == ' ' || arg.front() == '\t')) arg.erase(arg.begin());
            while (!arg.empty() && (arg.back() == ' ' || arg.back() == '\t')) arg.pop_back();
            for (auto &c : arg) c = (char)std::tolower((unsigned char)c);
            if (arg == "status" || arg == "state")
            {
                // 查询当前 safe_getup 状态并推送 notify(供页面刷新后回显)
                this->SafeGetupStatus();
                PublishRosFeedback("command accepted: safe_getup status");
                emit(kDone);
                return;
            }
            want_rl = (arg == "rl" || arg == "walk" || arg == "locomotion" || arg == "1");
            if (!want_rl && !arg.empty())
            {
                PublishRosFeedback("command rejected: safe_getup, unknown arg '" + arg + "' (use: safe_getup [rl|status])");
                emit(kRejected, "RLFSMStateGetUp");
                return;
            }
        }
        const std::string reject_reason = this->StartSafeGetup(want_rl);
        if (!reject_reason.empty())
        {
            PublishRosFeedback("command rejected: safe_getup, " + reject_reason);
            emit(kRejected, "RLFSMStateGetUp");
            return;
        }
        PublishRosFeedback(want_rl
            ? "command accepted: safe_getup (lie check -> probe -> getup -> stand check -> locomotion)"
            : "command accepted: safe_getup (lie check -> probe -> getup -> stand check)");
        emit(kExecuting);
    }
    else if (cmd == "getup" || cmd == "stand" || cmd == "up")
    {
        const bool imu_live = !(this->robot_state.imu.gyroscope[0] == 0.0f &&
                                this->robot_state.imu.gyroscope[1] == 0.0f &&
                                this->robot_state.imu.gyroscope[2] == 0.0f);
        std::string can_reason;
        const bool can_live = this->CheckCanFeedbackFresh(500, &can_reason);
        if (!can_live)
        {
            PublishRosFeedback("command rejected: getup, " + can_reason);
            this->Notify("error", "已拒绝起立：" + can_reason);
            emit(kRejected, "RLFSMStateGetUp");
            return;
        }
        if (!motor_manager.hasProbeResults())
        {
            PublishRosFeedback("command rejected: getup, motors not probed");
            this->Notify("error", "已拒绝起立：电机未执行可控性探针,请先在「关节管理」执行探针");
            emit(kRejected, "RLFSMStateGetUp");
            return;
        }
        if (!motor_manager.areAllMotorsProbedOk())
        {
            PublishRosFeedback("command rejected: getup, some motors not controllable");
            this->Notify("error", "已拒绝起立：存在不可控电机,请先在「关节管理」执行探针确认");
            emit(kRejected, "RLFSMStateGetUp");
            return;
        }
        const std::string cur_fsm = this->fsm.GetCurrentStateName();
        const bool in_safe_hold = cur_fsm == "RLFSMStateSafeHold";
        const bool in_passive = cur_fsm == "RLFSMStatePassive";
        bool zero_check_passed = this->motors_ready_for_stand;
        bool lie_check_missing = false;
        if (in_passive)
        {
            lie_check_missing = !this->lie_check_done_.load();
            zero_check_passed = !lie_check_missing && this->lie_check_ok_.load();
            this->motors_ready_for_stand = zero_check_passed;
        }
        if ((!zero_check_passed || !imu_live) && !in_safe_hold)
        {
            std::string reason;
            if (!zero_check_passed)
                reason = lie_check_missing
                    ? std::string("未执行卧姿校验,请先执行「卧姿校验」")
                    : this->motors_not_ready_reason;
            else
                reason = std::string("IMU no data");
            PublishRosFeedback("command rejected: getup, " + reason);
            this->Notify("error", std::string("已拒绝起立：") + reason);
            emit(kRejected, "RLFSMStateGetUp");
            return;
        }
        if (this->manual_rl_block_.exchange(false))
        {
            PublishRosFeedback("manual RL block cleared by getup");
        }
        this->control.SetKeyboard(Input::Keyboard::Num0);
        PublishRosFeedback("command accepted: getup");
        TrackFsmCommand(seq, "getup", "RLFSMStateGetUp", 6.0);
    }
    else if (cmd == "getdown" || cmd == "down" || cmd == "lie" || cmd == "sit")
    {
        this->ClearDiffVelocity();
        std::string can_reason;
        if (!this->CheckCanFeedbackFresh(500, &can_reason))
        {
            PublishRosFeedback("command rejected: getdown, " + can_reason);
            this->Notify("error", "已拒绝趴下：" + can_reason);
            emit(kRejected, "RLFSMStateGetDown");
            return;
        }
        this->manual_rl_block_.store(false, std::memory_order_relaxed);
        this->control.SetKeyboard(Input::Keyboard::Num9);
        this->control.x = 0.0f;
        this->control.y = 0.0f;
        this->control.yaw = 0.0f;
        PublishRosFeedback("command accepted: getdown");
        TrackFsmCommand(seq, "getdown", "RLFSMStatePassive", 8.0, 2.5);
    }
    else if (cmd == "locomotion" || cmd == "rl" || cmd == "walk" || cmd == "move")
    {
        std::string can_reason;
        if (!this->CheckCanFeedbackFresh(500, &can_reason))
        {
            PublishRosFeedback("command rejected: locomotion, " + can_reason);
            this->Notify("error", "已拒绝进入行走：" + can_reason);
            emit(kRejected, "RLFSMStateRLLocomotion");
            return;
        }
        const bool in_safe_hold =
            this->fsm.GetCurrentStateName() == "RLFSMStateSafeHold";
        if (!in_safe_hold)
        {
            std::string stand_reason;
            if (!this->CheckStandingPose(&stand_reason))
            {
                PublishRosFeedback("command rejected: locomotion, " + stand_reason);
                this->Notify("error", std::string("已拒绝进入强化学习：") + stand_reason);
                emit(kRejected, "RLFSMStateRLLocomotion");
                return;
            }
        }
        this->control.SetKeyboard(Input::Keyboard::Num1);
        PublishRosFeedback("command accepted: locomotion");
        TrackFsmCommand(seq, "locomotion", "RLFSMStateRLLocomotion", 5.0);
    }
    else if (cmd == "passive" || cmd == "stop" || cmd == "safe_stop")
    {
        this->control.SetKeyboard(Input::Keyboard::P);
        this->control.x = 0.0f;
        this->control.y = 0.0f;
        this->control.yaw = 0.0f;
#if JXG_ROS_LAYER
        this->cmd_vel = geometry_msgs::msg::Twist();
#endif
        PublishRosFeedback("command accepted: passive stop");
        TrackFsmCommand(seq, "passive", "RLFSMStatePassive", 5.0);
    }
    else if (cmd == "rl_block" || cmd == "rl_stop" || cmd == "block")
    {
        this->manual_rl_block_.store(true, std::memory_order_relaxed);
        this->control.x = 0.0f;
        this->control.y = 0.0f;
        this->control.yaw = 0.0f;
#if JXG_ROS_LAYER
        this->cmd_vel = geometry_msgs::msg::Twist();
#endif
        PublishRosFeedback("command accepted: RL OUTPUT BLOCKED -> SafeHold (send getup to resume)");
        this->Notify("error", "强化学习输出已阻断：原地锁定，发 getup 恢复");
        emit(kDone);
    }
    else if (cmd == "probe_motors" || cmd == "probe")
    {
        emit(kExecuting);
        this->Notify("info", "开始电机可控性探针(约 2s),期间请勿操作...");
        this->Notify("probe", "start");
        PublishRosFeedback("probe motors: executing");
        std::thread([this]() {
            static const char* names[16] = {
                "FL_hip","FL_thigh","FL_calf","FL_wheel",
                "FR_hip","FR_thigh","FR_calf","FR_wheel",
                "RL_hip","RL_thigh","RL_calf","RL_wheel",
                "RR_hip","RR_thigh","RR_calf","RR_wheel"
            };
            std::vector<bool> ok = motor_manager.probeAllMotorsCheck();
            int fail_count = 0;
            std::string fail_list;
            for (int i = 0; i < 16; ++i) {
                this->Notify("probe", "result m" + std::to_string(i) + " " + names[i] + " " + (ok[i] ? "ok" : "fail"));
                if (!ok[i]) {
                    ++fail_count;
                    fail_list += std::string(names[i]) + "(m" + std::to_string(i) + ") ";
                }
            }
            this->Notify("probe", "done " + std::to_string(fail_count));
            if (fail_count == 0) {
                this->Notify("success", "探针完成:全部 16 台电机可控 ✅");
                PublishRosFeedback("probe motors: all 16 motors controllable");
            } else {
                this->Notify("warn", "探针完成:" + std::to_string(fail_count) + " 台无响应(仅供参考): " + fail_list);
                PublishRosFeedback("probe motors: " + std::to_string(fail_count) + " motors not responding");
            }
        }).detach();
    }
    else if (cmd.rfind("probe_motor ", 0) == 0 || cmd.rfind("probe ", 0) == 0)
    {
        size_t sp = cmd.find(' ');
        std::string idstr = (sp != std::string::npos) ? cmd.substr(sp + 1) : "";
        if (!idstr.empty() && idstr[0] == 'm') idstr = idstr.substr(1);
        int id = -1;
        try { id = std::stoi(idstr); } catch (...) { id = -1; }
        if (id < 0 || id >= 16) {
            this->Notify("error", "探针失败:电机 ID 无效(0-15),收到: " + idstr);
            emit(kRejected);
            return;
        }
        static const char* names[16] = {
            "FL_hip","FL_thigh","FL_calf","FL_wheel",
            "FR_hip","FR_thigh","FR_calf","FR_wheel",
            "RL_hip","RL_thigh","RL_calf","RL_wheel",
            "RR_hip","RR_thigh","RR_calf","RR_wheel"
        };
        emit(kExecuting);
        this->Notify("probe", "single m" + std::to_string(id) + " " + names[id] + " start");
        std::thread([this, id]() {
            bool ok = motor_manager.probeMotorCheck(id);
            static const char* names[16] = {
                "FL_hip","FL_thigh","FL_calf","FL_wheel",
                "FR_hip","FR_thigh","FR_calf","FR_wheel",
                "RL_hip","RL_thigh","RL_calf","RL_wheel",
                "RR_hip","RR_thigh","RR_calf","RR_wheel"
            };
            this->Notify("probe", "single m" + std::to_string(id) + " " + names[id] + " " + (ok ? "ok" : "fail"));
            if (ok) {
                this->Notify("success", std::string(names[id]) + "(m" + std::to_string(id) + ") 探针:可控 ✅");
            } else {
                this->Notify("warn", std::string(names[id]) + "(m" + std::to_string(id) + ") 探针:无响应(仅供参考)");
            }
        }).detach();
    }
    else if (cmd.rfind("probe_leg ", 0) == 0)
    {
        size_t sp = cmd.find(' ');
        std::string legstr = (sp != std::string::npos) ? cmd.substr(sp + 1) : "";
        int leg = -1;
        if (legstr == "FL" || legstr == "fl" || legstr == "0") leg = 0;
        else if (legstr == "FR" || legstr == "fr" || legstr == "1") leg = 1;
        else if (legstr == "RL" || legstr == "rl" || legstr == "2") leg = 2;
        else if (legstr == "RR" || legstr == "rr" || legstr == "3") leg = 3;
        if (leg < 0 || leg >= 4) {
            this->Notify("error", "探针失败:腿号无效(FL/FR/RL/RR 或 0-3),收到: " + legstr);
            emit(kRejected);
            return;
        }
        static const char* leg_names[4] = {"FL", "FR", "RL", "RR"};
        static const char* names[16] = {
            "FL_hip","FL_thigh","FL_calf","FL_wheel",
            "FR_hip","FR_thigh","FR_calf","FR_wheel",
            "RL_hip","RL_thigh","RL_calf","RL_wheel",
            "RR_hip","RR_thigh","RR_calf","RR_wheel"
        };
        emit(kExecuting);
        this->Notify("probe", "legstart " + std::to_string(leg));
        std::thread([this, leg]() {
            static const char* leg_names[4] = {"FL", "FR", "RL", "RR"};
            static const char* names[16] = {
                "FL_hip","FL_thigh","FL_calf","FL_wheel",
                "FR_hip","FR_thigh","FR_calf","FR_wheel",
                "RL_hip","RL_thigh","RL_calf","RL_wheel",
                "RR_hip","RR_thigh","RR_calf","RR_wheel"
            };
            std::vector<bool> ok = motor_manager.probeLegCheck(leg);
            int fail_count = 0;
            for (int j = 0; j < 4; ++j) {
                int gid = leg * 4 + j;
                this->Notify("probe", "result m" + std::to_string(gid) + " " + names[gid] + " " + (ok[j] ? "ok" : "fail"));
                if (!ok[j]) ++fail_count;
            }
            this->Notify("probe", "legdone " + std::to_string(leg) + " " + std::to_string(fail_count));
            if (fail_count == 0) {
                this->Notify("success", std::string(leg_names[leg]) + " 腿探针:4 台全部可控 ✅");
            } else {
                this->Notify("warn", std::string(leg_names[leg]) + " 腿探针:" + std::to_string(fail_count) + " 台无响应(仅供参考)");
            }
        }).detach();
    }
    else if (cmd == "check_stand" || cmd == "standcheck" || cmd == "stand_check" || cmd == "check")
    {
        emit(kExecuting);
        const bool ready = this->HandleCheckStand();
        emit(ready ? kDone : kRejected, "RLFSMStateGetUp");
    }
    else if (cmd == "check_standing" || cmd == "standingcheck" || cmd == "standing_check")
    {
        emit(kExecuting);
        const bool at_stand = this->HandleCheckStanding();
        emit(at_stand ? kDone : kRejected, "RLFSMStateGetDown");
    }
    else if (cmd == "posture" || cmd == "pose" || cmd == "get_posture")
    {
        const std::string pjson = this->PublishPosture();
        PublishRosFeedback("posture " + pjson);
        emit(kDone);
    }
    else if (cmd == "vel_stop" || cmd == "hold")
    {
        this->ClearDiffVelocity();
        this->control.x = 0.0f;
        this->control.y = 0.0f;
        this->control.yaw = 0.0f;
#if JXG_ROS_LAYER
        this->cmd_vel = geometry_msgs::msg::Twist();
#endif
        PublishRosFeedback("command accepted: velocity stop");
        emit(kDone);
    }
    else if (cmd == "nav_on")
    {
        this->ClearDiffVelocity();
        this->diff_nav_.store(true);
        this->control.navigation_mode = true;
        PublishRosFeedback("command accepted: nav_on");
        emit(kDone);
    }
    else if (cmd == "nav_off")
    {
        this->diff_nav_.store(false);
        this->ClearDiffVelocity();
        this->control.navigation_mode = false;
        PublishRosFeedback("command accepted: nav_off");
        emit(kDone);
    }
    else if (cmd == "imu_on")
    {
        SetJXGIMUStreamEnabled(true);
        PublishRosFeedback("command accepted: imu_on");
        emit(kDone);
    }
    else if (cmd == "imu_off")
    {
        SetJXGIMUStreamEnabled(false);
        PublishRosFeedback("command accepted: imu_off");
        emit(kDone);
    }
    else if (cmd.size() > 12 && cmd.compare(0, 12, "zero_motors ") == 0)
    {
        std::vector<int> ids;
        const std::string perr = parse_zero_ids(cmd.substr(12), &ids);
        if (!perr.empty())
        {
            PublishRosFeedback("command rejected: zero_motors, " + perr);
            emit(kRejected);
            return;
        }
        std::sort(ids.begin(), ids.end());
        ids.erase(std::unique(ids.begin(), ids.end()), ids.end());
        const std::vector<int> failed = motor_manager.setZeroMotorsVerified(ids);
        const bool all_ok = publish_zero_result(ids, failed);
        if (all_ok)
            this->Notify("info", std::to_string(ids.size()) + " 个电机置零成功（已反馈确认）");
        else
            this->Notify("error", std::to_string(failed.size()) + " 个电机置零未确认，请检查 CAN 后仅重试失败电机");
        emit(kDone);
    }
    else if (cmd.size() > 11 && cmd.compare(0, 11, "zero_motor ") == 0)
    {
        int motor_id = -1;
        bool valid = false;
        {
            const std::string num = cmd.substr(11);
            try
            {
                size_t consumed = 0;
                const int v = std::stoi(num, &consumed);
                if (consumed == num.size() && v >= 0 && v < 16)
                {
                    motor_id = v;
                    valid = true;
                }
            }
            catch (...)
            {
                valid = false;
            }
        }
        if (!valid)
        {
            PublishRosFeedback("command rejected: zero_motor, expected 'zero_motor <0..15>'");
            emit(kRejected);
            return;
        }
        const std::vector<int> failed = motor_manager.setZeroMotorsVerified({motor_id});
        const bool all_ok = publish_zero_result({motor_id}, failed);
        if (all_ok && this->motors_ready_for_stand)
            this->Notify("info", "motor " + std::to_string(motor_id) + " re-zeroed, ready for stand");
        else if (!all_ok)
            this->Notify("error", "motor " + std::to_string(motor_id) + " 置零未确认，请检查 CAN");
        else
            this->Notify("error", this->motors_not_ready_reason);
        emit(kDone);
    }
    else if (cmd == "zero" || cmd == "setzero" || cmd == "set_zero" || cmd == "set_zero_all" || cmd == "zero_all")
    {
        std::vector<int> ids(16);
        for (int i = 0; i < 16; ++i) ids[i] = i;
        const std::vector<int> failed = motor_manager.setZeroMotorsVerified(ids);
        const bool all_ok = publish_zero_result(ids, failed);
        if (all_ok)
            this->Notify("info", "全部 16 个电机置零成功（已反馈确认）");
        else
            this->Notify("error", std::to_string(failed.size()) + " 个电机置零未确认，请检查 CAN 后重试");
        emit(kDone);
    }
    else if (cmd == "quit" || cmd == "exit")
    {
        g_shutdown_flag.store(true);
        PublishRosFeedback("command accepted: exit");
        emit(kDone);
    }
    else if (cmd == "feedback" || cmd == "status")
    {
        std::string state_name = this->fsm.GetCurrentStateName();
        if (state_name.empty()) state_name = "unknown";
        PublishRosFeedback("status state=" + state_name + " gamepad_override=false"
                           + " " + this->BuildPostureJson());
        this->PublishPosture();
        emit(kDone);
    }
    else if (cmd == "bms_threshold" || cmd.rfind("bms_threshold ", 0) == 0)
    {
        // 低电量阈值配置:
        //   bms_threshold              → 查询当前阈值与报警周期
        //   bms_threshold soc <N>      → 设置 SOC 阈值(%)
        //   bms_threshold voltage <N>  → 设置电压阈值(V)
        //   bms_threshold interval <N> → 设置报警续报周期(秒)
        if (cmd == "bms_threshold")
        {
            std::ostringstream oss;
            oss << "bms_threshold soc=" << this->low_battery_soc_threshold_
                << "% voltage=" << this->low_battery_voltage_threshold_
                << "V interval=" << this->low_battery_notify_interval_sec_ << "s";
            PublishRosFeedback(oss.str());
            this->Notify("info", "当前低电量阈值: SOC<" +
                std::to_string((int)this->low_battery_soc_threshold_) + "% 或 电压<" +
                std::to_string(this->low_battery_voltage_threshold_) + "V, 报警周期" +
                std::to_string(this->low_battery_notify_interval_sec_) + "秒");
            emit(kDone);
            return;
        }
        std::string arg = cmd.substr(14);  // "bms_threshold " 之后
        // 解析 "<key> <N>"
        size_t sp = arg.find(' ');
        if (sp == std::string::npos)
        {
            PublishRosFeedback("command rejected: bms_threshold, expected 'bms_threshold soc|voltage|interval <N>'");
            emit(kRejected);
            return;
        }
        std::string key = arg.substr(0, sp);
        std::string val_str = arg.substr(sp + 1);
        float val = 0.0f;
        try {
            size_t consumed = 0;
            val = std::stof(val_str, &consumed);
            if (consumed != val_str.size()) throw std::invalid_argument("trailing chars");
        } catch (...) {
            PublishRosFeedback("command rejected: bms_threshold, invalid number '" + val_str + "'");
            emit(kRejected);
            return;
        }
        if (key == "soc")
        {
            if (val < 0.0f || val > 100.0f)
            {
                PublishRosFeedback("command rejected: bms_threshold soc, must be 0~100");
                emit(kRejected);
                return;
            }
            this->low_battery_soc_threshold_ = val;
            std::ostringstream oss;
            oss << "bms_threshold soc=" << val << "%";
            PublishRosFeedback(oss.str());
            this->Notify("info", "低电量 SOC 阈值已设为 " + std::to_string((int)val) + "%");
            emit(kDone);
        }
        else if (key == "voltage")
        {
            if (val <= 0.0f || val > 60.0f)
            {
                PublishRosFeedback("command rejected: bms_threshold voltage, must be 0~60V");
                emit(kRejected);
                return;
            }
            this->low_battery_voltage_threshold_ = val;
            std::ostringstream oss;
            oss << "bms_threshold voltage=" << val << "V";
            PublishRosFeedback(oss.str());
            this->Notify("info", "低电量电压阈值已设为 " + std::to_string(val) + "V");
            emit(kDone);
        }
        else if (key == "interval")
        {
            int iv = static_cast<int>(val);
            if (iv < 1 || iv > 3600)
            {
                PublishRosFeedback("command rejected: bms_threshold interval, must be 1~3600 秒");
                emit(kRejected);
                return;
            }
            this->low_battery_notify_interval_sec_ = iv;
            std::ostringstream oss;
            oss << "bms_threshold interval=" << iv << "s";
            PublishRosFeedback(oss.str());
            this->Notify("info", "低电量报警周期已设为 " + std::to_string(iv) + " 秒");
            emit(kDone);
        }
        else
        {
            PublishRosFeedback("command rejected: bms_threshold, unknown key '" + key + "' (use soc, voltage or interval)");
            emit(kRejected);
        }
    }
    else if (cmd == "version" || cmd == "about")
    {
        // 版本查询:返回版本号/构建号/构建时间/工作目录(供 web「关于」页展示,便于区分 vN 部署)
        char cwd[1024] = {0};
        if (!getcwd(cwd, sizeof(cwd) - 1)) cwd[0] = '\0';
        std::ostringstream oss;
        oss << "version rl_real_JXG v" << APP_VERSION
            << " build " << BUILD_NUMBER
            << " built " << BUILD_TIMESTAMP
            << " cwd " << cwd;
        PublishRosFeedback(oss.str());
        emit(kDone);
        return;
    }
    else if (cmd == "temp_threshold" || cmd.rfind("temp_threshold ", 0) == 0)
    {
        // 电机温度告警阈值配置:
        //   temp_threshold               → 查询当前阈值与周期
        //   temp_threshold <N>           → 设置中值告警阈值(°C)(等价 temp <N>)
        //   temp_threshold temp <N>      → 设置中值告警阈值(°C)
        //   temp_threshold interval <N>  → 设置告警续报周期(秒)
        if (cmd == "temp_threshold")
        {
            std::ostringstream oss;
            oss << "temp_threshold temp=" << this->motor_temp_alert_threshold_
                << "°C interval=" << this->motor_temp_alert_interval_sec_ << "s";
            PublishRosFeedback(oss.str());
            this->Notify("info", "当前电机温度告警: 滑动中值>" +
                std::to_string(this->motor_temp_alert_threshold_) + "°C, 周期" +
                std::to_string(this->motor_temp_alert_interval_sec_) + "秒 (仅提醒,不干预)");
            emit(kDone);
            return;
        }
        std::string arg = cmd.substr(15);  // "temp_threshold " 之后
        // 解析 "[key] <N>":无 key 默认按 temp 处理(兼容旧用法)
        std::string key = "temp";
        std::string val_str = arg;
        size_t sp = arg.find(' ');
        if (sp != std::string::npos)
        {
            key = arg.substr(0, sp);
            val_str = arg.substr(sp + 1);
        }
        float val = 0.0f;
        try {
            size_t consumed = 0;
            val = std::stof(val_str, &consumed);
            if (consumed != val_str.size()) throw std::invalid_argument("trailing chars");
        } catch (...) {
            PublishRosFeedback("command rejected: temp_threshold, invalid number '" + val_str + "'");
            emit(kRejected);
            return;
        }
        if (key == "temp")
        {
            if (val < 30.0f || val > 120.0f)
            {
                PublishRosFeedback("command rejected: temp_threshold temp, must be 30~120°C");
                emit(kRejected);
                return;
            }
            this->motor_temp_alert_threshold_ = val;
            std::ostringstream oss;
            oss << "temp_threshold temp=" << val << "°C";
            PublishRosFeedback(oss.str());
            this->Notify("info", "电机温度告警阈值已设为 " + std::to_string(val) + "°C (滑动中值,仅提醒)");
            emit(kDone);
        }
        else if (key == "interval")
        {
            int iv = static_cast<int>(val);
            if (iv < 1 || iv > 3600)
            {
                PublishRosFeedback("command rejected: temp_threshold interval, must be 1~3600 秒");
                emit(kRejected);
                return;
            }
            this->motor_temp_alert_interval_sec_ = iv;
            std::ostringstream oss;
            oss << "temp_threshold interval=" << iv << "s";
            PublishRosFeedback(oss.str());
            this->Notify("info", "电机温度告警周期已设为 " + std::to_string(iv) + " 秒");
            emit(kDone);
        }
        else
        {
            PublishRosFeedback("command rejected: temp_threshold, unknown key '" + key + "' (use temp or interval)");
            emit(kRejected);
        }
    }
    else
    {
        PublishRosFeedback("unknown command: " + cmd);
        emit(kRejected);
    }
}

// ===== 命令状态广播与跟踪 =====
void RL_Real::EmitCommandState(int seq, const std::string &cmd, int code,
                               const std::string &target, double elapsed_ms)
{
    static const char *kPhase[] = {"RECEIVED", "EXECUTING", "DONE", "FAILED", "REJECTED"};
    const char *phase = (code >= 0 && code <= 4) ? kPhase[code] : "UNKNOWN";
    const std::string state =
        this->fsm.GetCurrentStateName();

    std::ostringstream oss;
    oss << "{\"type\":\"cmd_state\",\"data\":{"
        << "\"seq\":" << seq
        << ",\"cmd\":\"" << CmdJsonEscape(cmd) << "\""
        << ",\"code\":" << code
        << ",\"phase\":\"" << phase << "\""
        << ",\"target\":";
    if (target.empty()) oss << "null";
    else                oss << "\"" << CmdJsonEscape(target) << "\"";
    oss << ",\"state\":\"" << CmdJsonEscape(state) << "\""
        << ",\"elapsed_ms\":" << static_cast<long long>(elapsed_ms >= 0.0 ? elapsed_ms + 0.5
                                                                           : elapsed_ms - 0.5)
        << "}}";
    const std::string frame = oss.str();

#if JXG_ROS_LAYER
    if (this->cmd_state_publisher)
    {
        std_msgs::msg::String m;
        m.data = frame;
        this->cmd_state_publisher->publish(m);
    }
#endif
#if JXG_WEB_LAYER
    if (this->web_bridge_) this->web_bridge_->broadcastFrame(frame);
#endif
}

void RL_Real::MaybePublishHeartbeat()
{
    const double now = CmdSteadyNowSec();
    if (this->heartbeat_start_sec_ == 0.0) this->heartbeat_start_sec_ = now;
    if (now - this->last_heartbeat_sec_ < 0.49) return;
    this->last_heartbeat_sec_ = now;

    const std::string state =
        this->fsm.GetCurrentStateName();
    int active_seq = 0;
    {
        std::lock_guard<std::mutex> lk(this->pending_cmd_mutex_);
        if (this->pending_cmd_) active_seq = this->pending_cmd_->seq;
    }
    const long long uptime_ms =
        static_cast<long long>((now - this->heartbeat_start_sec_) * 1000.0 + 0.5);

    std::ostringstream oss;
    oss << "{\"type\":\"heartbeat\",\"data\":{"
        << "\"seq\":" << (++this->heartbeat_seq_)
        << ",\"state\":\"" << CmdJsonEscape(state) << "\""
        << ",\"nav\":" << (this->control.navigation_mode ? "true" : "false")
        << ",\"uptime_ms\":" << uptime_ms
        << ",\"active_seq\":" << active_seq
        << "}}";
    const std::string frame = oss.str();

#if JXG_ROS_LAYER
    if (this->heartbeat_publisher)
    {
        std_msgs::msg::String m;
        m.data = frame;
        this->heartbeat_publisher->publish(m);
    }
#endif
#if JXG_WEB_LAYER
    if (this->web_bridge_) this->web_bridge_->broadcastFrame(frame);
#endif
}

void RL_Real::TrackFsmCommand(int seq, const std::string &cmd,
                              const std::string &target_state, double timeout_sec,
                              double min_sec)
{
    auto p = std::make_unique<PendingCommand>();
    p->seq = seq;
    p->cmd = cmd;
    p->target_state = target_state;
    p->start_sec = CmdSteadyNowSec();
    p->timeout_sec = timeout_sec;
    p->min_sec = min_sec;
    std::lock_guard<std::mutex> lk(this->pending_cmd_mutex_);
    this->pending_cmd_ = std::move(p);
}

void RL_Real::UpdatePendingCommand()
{
    PendingCommand snap;
    bool send_exec = false, done = false, failed = false;
    {
        std::lock_guard<std::mutex> lk(this->pending_cmd_mutex_);
        if (!this->pending_cmd_) return;
        snap = *this->pending_cmd_;
        const double el = CmdSteadyNowSec() - snap.start_sec;
        if (!snap.executing_sent)
        {
            send_exec = true;
            this->pending_cmd_->executing_sent = true;
        }
        const std::string cur =
            this->fsm.GetCurrentStateName();
        if (cur == snap.target_state && this->behavior_settled.load() &&
            el >= snap.min_sec)
        {
            done = true;
            this->pending_cmd_.reset();
        }
        else if (el > snap.timeout_sec)
        {
            failed = true;
            this->pending_cmd_.reset();
        }
    }
    const double el_ms = (CmdSteadyNowSec() - snap.start_sec) * 1000.0;
    if (send_exec) this->EmitCommandState(snap.seq, snap.cmd, kExecuting, snap.target_state, el_ms);
    if (done)      this->EmitCommandState(snap.seq, snap.cmd, kDone,      snap.target_state, el_ms);
    if (failed)    this->EmitCommandState(snap.seq, snap.cmd, kFailed,    snap.target_state, el_ms);
}
