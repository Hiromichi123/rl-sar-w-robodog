// ============================================================================
// web_server —— 独立 Web 上位机进程(与 rl_real_JXG 解耦)
//
// 作用:
//   把内置 WebBridge(HTTP 静态页 + WebSocket :8088 + WebSSH + WebFTP +
//   实时日志)从 rl_real_JXG 进程中剥离为独立常驻服务。它本身不碰 CAN/电机,
//   只做 ROS2 话题 <-> WebSocket 的协议桥:
//
//     浏览器 ──WS──> web_server  ──ROS2──>  rl_real_JXG(可随时重启)
//     浏览器 <──WS──  web_server  <──ROS2── rl_real_JXG
//
//   rl_real_JXG 重启期间本进程与浏览器的 WS/PTY/FTP 连接均保持不断;
//   rl_real 重新上线后 DDS 自动发现,遥测与命令自然恢复。
//
// 话题映射(与 rl_real 现有话题一一对应, std_msgs/String 载荷即 WS JSON 帧):
//   订阅: /rl_real/motor_state / imu_state / pose2d (Float32MultiArray)
//         /rl_real/feedback / notify / check_stand_result / posture
//         /rl_real/check_standing_result (String)
//         /rl_real/cmd_state / heartbeat (String, 内容为完整 WS 帧, 原样透传)
//   发布: /rl_real/web_cmd_vel (Twist, 浏览器速度独占话题) / command (String) /
//         imu_stream_enable (Bool) /rl_real/notify_ack (String) / check_stand (String)
//
// 环境变量:
//   JXG_WEB_PORT   监听端口(默认 8088)
//   WEB_ROOT       静态页根目录(默认编译期宏)
//   WEB_ADMIN_TOKEN WebSSH/WebFTP/电源操作鉴权令牌(空=局域网放行,启动告警)
//   JXG_LOG_UNIT   实时日志跟踪的 systemd 单元(默认 rl-real-jxg.service)
//
// 服务管理(等价 scripts/service.sh, 在本独立进程内执行 —— 可安全
// stop/restart 控制程序自身):
//   WS 入站 {"type":"command","data":"svc <status|start|stop|restart|enable|disable>"}
//   WS 出站 {"type":"svc_status","data":{unit,installed,active,enabled,pid,
//             fifo,fifo_total,cgroup,op,ok,error}}
// ============================================================================
#include <algorithm>
#include <atomic>
#include <cctype>
#include <cerrno>
#include <chrono>

// 版本/构建时间宏兜底(正常由 CMake target_compile_definitions 提供)
#ifndef APP_VERSION
#define APP_VERSION "0.0.0-dev"
#define BUILD_NUMBER "unknown"
#define BUILD_TIMESTAMP "unknown"
#endif
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <dirent.h>
#include <fcntl.h>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/statvfs.h>
#include <sys/sysmacros.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
#include <vector>

#include <arpa/inet.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <netinet/in.h>
#include <poll.h>
#include <termios.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "web_bridge.hpp"
#include "bms_ble.hpp"

#ifndef WEB_ROOT
#define WEB_ROOT "/home/jinjiao/rl_sar-w2_recovered_v4/web"
#endif

namespace
{
std::atomic<bool> g_shutdown{false};

void OnSignal(int) { g_shutdown.store(true); }

std::vector<float> ToVector(const std_msgs::msg::Float32MultiArray::SharedPtr &m)
{
    return std::vector<float>(m->data.begin(), m->data.end());
}

// ── rl-real-jxg 服务管理(等价 scripts/service.sh) ─────────────────────────
constexpr const char *kUnitName = "rl-real-jxg";
constexpr const char *kProcName = "rl_real_JXG";
constexpr int kExpectedFifoThreads = 11;
std::atomic<bool> g_svc_busy{false};  // start/stop/restart/enable/disable 串行化

std::string Trim(const std::string &s)
{
    size_t a = s.find_first_not_of(" \t\r\n");
    if (a == std::string::npos) return "";
    size_t b = s.find_last_not_of(" \t\r\n");
    return s.substr(a, b - a + 1);
}

std::string JsonEscape(const std::string &s)
{
    std::string o;
    o.reserve(s.size() + 8);
    for (char ch : s) {
        switch (ch) {
            case '"':  o += "\\\""; break;
            case '\\': o += "\\\\"; break;
            case '\n': o += "\\n"; break;
            case '\r': o += "\\r"; break;
            case '\t': o += "\\t"; break;
            default:
                if (static_cast<unsigned char>(ch) < 0x20) {
                    char buf[8]; std::snprintf(buf, sizeof(buf), "\\u%04x", ch & 0xFF);
                    o += buf;
                } else o += ch;
        }
    }
    return o;
}

// 直接 fork+exec 执行命令(不经 /bin/sh 与 popen): waitpid 自行处理 EINTR,
// 可明确区分退出码/致死信号, 不受进程内其它 waitpid/SIGCHLD 时序影响。
// stdout+stderr 合并收集。
int RunShell(const std::vector<std::string> &args, std::string *out)
{
    if (args.empty()) return -1;
    int pf[2];
    if (::pipe(pf) != 0) return -1;
    pid_t pid = ::fork();
    if (pid < 0) { ::close(pf[0]); ::close(pf[1]); return -1; }
    if (pid == 0) {
        ::dup2(pf[1], STDOUT_FILENO);
        ::dup2(pf[1], STDERR_FILENO);
        if (pf[0] > 2) ::close(pf[0]);
        if (pf[1] > 2) ::close(pf[1]);
        std::vector<char *> argv;
        argv.reserve(args.size() + 1);
        for (const auto &a : args) argv.push_back(const_cast<char *>(a.c_str()));
        argv.push_back(nullptr);
        ::execvp(argv[0], argv.data());
        _exit(127);  // exec 失败
    }
    ::close(pf[1]);
    std::string collected;
    char buf[512];
    ssize_t n;
    while ((n = ::read(pf[0], buf, sizeof(buf))) > 0) collected.append(buf, size_t(n));
    ::close(pf[0]);
    int status = 0;
    while (::waitpid(pid, &status, 0) < 0) {
        if (errno != EINTR) {
            std::perror("[web_server] waitpid");
            return -1;
        }
    }
    if (out) *out = Trim(collected);
    if (WIFEXITED(status)) return WEXITSTATUS(status);
    if (WIFSIGNALED(status)) {
        std::cerr << "[web_server] child '" << args[0] << "' killed by signal "
                  << WTERMSIG(status) << std::endl;
        return 128 + WTERMSIG(status);
    }
    return -1;
}

int SystemCtl(const std::string &args, std::string *out = nullptr, const std::string &unit = kUnitName)
{
    // systemd 环境下 PATH 通常含 /usr/bin; 兜底常见绝对路径
    for (const char *p : {"/usr/bin/systemctl", "/bin/systemctl", "systemctl"}) {
        if (p[0] != '/' || ::access(p, X_OK) == 0) {
            std::vector<std::string> v;
            v.push_back(p);
            v.push_back(args);
            if (!unit.empty()) v.push_back(unit);
            return RunShell(v, out);
        }
    }
    if (out) *out = "systemctl not found";
    return -1;
}

// 统计主进程 SCHED_FIFO(policy==1) 线程数, 与 service.sh fifo_count_of 一致
int FifoThreadCount(int pid)
{
    if (pid <= 0) return 0;
    int count = 0;
    std::string task_dir = "/proc/" + std::to_string(pid) + "/task";
    DIR *d = ::opendir(task_dir.c_str());
    if (!d) return 0;
    struct dirent *de;
    while ((de = ::readdir(d)) != nullptr) {
        if (de->d_name[0] == '.') continue;
        std::ifstream f(task_dir + "/" + de->d_name + "/sched");
        std::string line;
        while (std::getline(f, line)) {
            if (line.rfind("policy", 0) == 0) {
                // 形如 "policy                    :\t\t1"
                auto p = line.find(':');
                if (p != std::string::npos && Trim(line.substr(p + 1)) == "1") ++count;
                break;
            }
        }
    }
    ::closedir(d);
    return count;
}

// 看门狗用:扫描 /proc/<pid>/comm 判断 rl_real_JXG 进程是否在运行(不依赖 systemctl,
// 不 fork)。web_server 自身名为 web_server,天然不会误匹配。
bool IsRlRealProcessAlive()
{
    DIR *d = ::opendir("/proc");
    if (!d) return false;
    bool alive = false;
    struct dirent *de;
    while ((de = ::readdir(d)) != nullptr) {
        if (de->d_name[0] == '.') continue;
        std::ifstream f(std::string("/proc/") + de->d_name + "/comm");
        std::string comm;
        if (std::getline(f, comm) && Trim(comm) == kProcName) { alive = true; break; }
    }
    ::closedir(d);
    return alive;
}

// 采集当前服务状态并广播(帧格式见文件头注释)
void PublishServiceStatus(WebBridge &bridge, const std::string &op, bool ok,
                          const std::string &error, bool busy)
{
    const std::string unit_path = "/etc/systemd/system/" + std::string(kUnitName) + ".service";
    struct stat st;
    bool installed = ::stat(unit_path.c_str(), &st) == 0;

    std::string active, enabled, pid_str;
    SystemCtl("is-active", &active);
    SystemCtl("is-enabled", &enabled);
    for (const char *p : {"/usr/bin/pgrep", "/bin/pgrep"}) {
        if (::access(p, X_OK) == 0) { RunShell({p, "-x", kProcName}, &pid_str); break; }
    }
    // pgrep 可能输出多行(异常残留多实例), 取首行
    if (auto nl = pid_str.find('\n'); nl != std::string::npos) pid_str = pid_str.substr(0, nl);
    pid_str = Trim(pid_str);
    int pid = 0, fifo = 0;
    if (!pid_str.empty()) {
        pid = std::atoi(pid_str.c_str());
        fifo = FifoThreadCount(pid);
    }
    std::string cgroup = "-";
    if (pid > 0) {
        std::ifstream f("/proc/" + std::to_string(pid) + "/cgroup");
        std::string line;
        while (std::getline(f, line)) {
            // 形如 "0::/..." 或 "0:name:/..."
            if (!line.empty() && line[0] == '0') {
                auto p1 = line.find(':');
                auto p2 = line.find(':', p1 + 1);
                if (p2 != std::string::npos) cgroup = line.substr(p2 + 1);
                break;
            }
        }
    }

    std::ostringstream os;
    os << "{\"type\":\"svc_status\",\"data\":{"
       << "\"unit\":\"" << kUnitName << "\","
       << "\"installed\":" << (installed ? "true" : "false") << ","
       << "\"active\":\"" << JsonEscape(active.empty() ? "unknown" : active) << "\","
       << "\"enabled\":\"" << JsonEscape(enabled.empty() ? "disabled" : enabled) << "\","
       << "\"pid\":" << pid << ","
       << "\"fifo\":" << fifo << ","
       << "\"fifo_total\":" << kExpectedFifoThreads << ","
       << "\"cgroup\":\"" << JsonEscape(cgroup) << "\","
       << "\"op\":\"" << JsonEscape(op) << "\","
       << "\"busy\":" << (busy ? "true" : "false") << ","
       << "\"ok\":" << (ok ? "true" : "false") << ","
       << "\"error\":\"" << JsonEscape(error) << "\"}}";
    bridge.broadcastFrame(os.str());
}

// ── 系统信息采集(CPU/内存/Swap/Tasks/Load/Uptime) ────────────────────────
void PublishSysInfo(WebBridge &bridge)
{
    // CPU 使用率: 取 /proc/stat 两次采样间隔
    static unsigned long long prev_used = 0, prev_total = 0;
    std::ifstream statf("/proc/stat");
    std::string line;
    unsigned long long cpu_used = 0, cpu_total = 0;
    if (std::getline(statf, line) && line.rfind("cpu ", 0) == 0) {
        std::istringstream iss(line.substr(4));
        unsigned long long user, nice, sys, idle, iowait, irq, softirq, steal;
        iss >> user >> nice >> sys >> idle >> iowait >> irq >> softirq >> steal;
        cpu_total = user + nice + sys + idle + iowait + irq + softirq + steal;
        cpu_used = cpu_total - idle - iowait;
    }
    double cpu_pct = 0.0;
    if (prev_total > 0) {
        unsigned long long dt = cpu_total - prev_total;
        unsigned long long du = cpu_used - prev_used;
        if (dt > 0) cpu_pct = 100.0 * du / dt;
    }
    prev_used = cpu_used;
    prev_total = cpu_total;

    // 内存 / Swap
    std::ifstream memf("/proc/meminfo");
    long mem_total = 0, mem_available = 0, swap_total = 0, swap_free = 0;
    while (std::getline(memf, line)) {
        if (line.rfind("MemTotal:", 0) == 0) mem_total = std::stol(Trim(line.substr(10)));
        else if (line.rfind("MemAvailable:", 0) == 0) mem_available = std::stol(Trim(line.substr(13)));
        else if (line.rfind("SwapTotal:", 0) == 0) swap_total = std::stol(Trim(line.substr(10)));
        else if (line.rfind("SwapFree:", 0) == 0) swap_free = std::stol(Trim(line.substr(9)));
    }

    // Load average
    std::ifstream loadf("/proc/loadavg");
    double load1 = 0, load5 = 0, load15 = 0;
    if (loadf >> load1 >> load5 >> load15) {}

    // Uptime
    std::ifstream upf("/proc/uptime");
    double uptime_s = 0;
    if (upf >> uptime_s) {}

    // 任务数
    int tasks = 0;
    DIR *d = ::opendir("/proc");
    if (d) {
        struct dirent *de;
        while ((de = ::readdir(d)) != nullptr) {
            if (de->d_name[0] >= '0' && de->d_name[0] <= '9') ++tasks;
        }
        ::closedir(d);
    }

    // 磁盘占用:读取 /proc/mounts,过滤虚拟文件系统,用 statvfs 统计
    std::ostringstream disks_json;
    disks_json << "\"disks\":[";
    {
        std::ifstream mntf("/proc/mounts");
        std::string mline;
        bool first_disk = true;
        while (std::getline(mntf, mline)) {
            std::istringstream mls(mline);
            std::string dev, mnt, fstype;
            mls >> dev >> mnt >> fstype;
            // 只统计真实块设备挂载点
            if (dev.empty() || mnt.empty() || fstype.empty()) continue;
            if (dev.rfind("/dev/", 0) != 0 && dev.rfind("/dev2/", 0) != 0) continue;
            // 排除虚拟/特殊文件系统
            static const std::vector<std::string> skip_fs = {
                "tmpfs", "devtmpfs", "proc", "sysfs", "cgroup", "cgroup2",
                "pstore", "bpf", "debugfs", "tracefs", "securityfs",
                "devpts", "mqueue", "hugetlbfs", "configfs", "fusectl",
                "binfmt_misc", "autofs", "rpc_pipefs", "nfsd"
            };
            bool skip = false;
            for (const auto &sf : skip_fs) {
                if (fstype == sf) { skip = true; break; }
            }
            if (skip) continue;
            struct statvfs sv;
            if (::statvfs(mnt.c_str(), &sv) != 0) continue;
            unsigned long long total = (unsigned long long)sv.f_blocks * sv.f_frsize;
            unsigned long long free  = (unsigned long long)sv.f_bavail * sv.f_frsize;
            unsigned long long used  = total - free;
            if (total == 0) continue;
            if (!first_disk) disks_json << ",";
            first_disk = false;
            disks_json << "{\"mount\":\"" << JsonEscape(mnt) << "\",\"fs\":\"" << JsonEscape(fstype)
                       << "\",\"total_kb\":" << (total / 1024)
                       << ",\"used_kb\":" << (used / 1024)
                       << ",\"free_kb\":" << (free / 1024) << "}";
        }
    }
    disks_json << "]";

    std::ostringstream os;
    os << "{\"type\":\"sys_info\",\"data\":{"
       << "\"cpu_pct\":" << std::fixed << std::setprecision(1) << cpu_pct << ","
       << "\"mem_total_kb\":" << mem_total << ","
       << "\"mem_used_kb\":" << (mem_total - mem_available) << ","
       << "\"swap_total_kb\":" << swap_total << ","
       << "\"swap_used_kb\":" << (swap_total - swap_free) << ","
       << "\"tasks\":" << tasks << ","
       << "\"load1\":" << load1 << ",\"load5\":" << load5 << ",\"load15\":" << load15 << ","
       << "\"uptime_s\":" << std::fixed << std::setprecision(0) << uptime_s << ","
       << disks_json.str()
       << "}}";
    bridge.broadcastFrame(os.str());
}

// ── 服务列表(systemctl list-units --type=service --all) ─────────────────────
void PublishServiceList(WebBridge &bridge)
{
    std::string out;
    // list-units 含多个参数,需拆开传入 RunShell
    std::vector<std::string> v = {"/usr/bin/systemctl", "list-units", "--type=service", "--all", "--no-legend", "--no-pager"};
    if (::access(v[0].c_str(), X_OK) != 0) v[0] = "systemctl";
    RunShell(v, &out);
    std::ostringstream os;
    os << "{\"type\":\"svc_list\",\"data\":{\"units\":[";
    bool first = true;
    std::istringstream iss(out);
    std::string line;
    while (std::getline(iss, line)) {
        line = Trim(line);
        if (line.empty()) continue;
        // 格式: UNIT LOAD ACTIVE SUB DESCRIPTION(字段间多空格)
        std::istringstream ls(line);
        std::string unit, load, active, sub;
        ls >> unit >> load >> active >> sub;
        if (unit.empty()) continue;
        // 剩余部分为描述
        std::string desc;
        std::getline(ls, desc);
        desc = Trim(desc);
        if (!first) os << ",";
        first = false;
        os << "{\"unit\":\"" << JsonEscape(unit) << "\",\"load\":\"" << JsonEscape(load)
           << "\",\"active\":\"" << JsonEscape(active) << "\",\"sub\":\"" << JsonEscape(sub)
           << "\",\"desc\":\"" << JsonEscape(desc) << "\"}";
    }
    os << "]}}";
    bridge.broadcastFrame(os.str());
}

// ── 进程列表(遍历 /proc/<pid>) ─────────────────────────────────────────────
void PublishProcessList(WebBridge &bridge)
{
    std::ostringstream os;
    os << "{\"type\":\"ps_list\",\"data\":{\"procs\":[";
    bool first = true;
    DIR *d = ::opendir("/proc");
    if (d) {
        struct dirent *de;
        while ((de = ::readdir(d)) != nullptr) {
            if (de->d_name[0] < '0' || de->d_name[0] > '9') continue;
            int pid = std::atoi(de->d_name);
            std::string base = "/proc/" + std::string(de->d_name);
            // comm
            std::ifstream cf(base + "/comm");
            std::string comm;
            std::getline(cf, comm);
            // cmdline
            std::ifstream clf(base + "/cmdline");
            std::string cmdline;
            char ch;
            while (clf.get(ch)) { if (ch == '\0') cmdline += ' '; else cmdline += ch; }
            cmdline = Trim(cmdline);
            if (cmdline.empty()) cmdline = comm;
            // stat: pid (comm) state ppid ...
            std::ifstream sf(base + "/stat");
            std::string stat_line;
            std::getline(sf, stat_line);
            std::string state = "?";
            int ppid = 0;
            long rss_pages = 0;
            unsigned long long utime = 0, stime = 0;
            if (!stat_line.empty()) {
                auto rp = stat_line.rfind(')');
                if (rp != std::string::npos) {
                    std::istringstream iss(stat_line.substr(rp + 1));
                    iss >> state >> ppid;
                    // ppid(4) 后跳过 pgrp(5) session(6) tty_nr(7) tpgid(8) flags(9)
                    // minflt(10) cminflt(11) majflt(12) cmajflt(13) → 9 个字段到 utime(14)
                    for (int i = 0; i < 9; ++i) { std::string skip; iss >> skip; }
                    iss >> utime >> stime;
                    // stime(15) 后跳过 cutime(16) cstime(17) priority(18) nice(19)
                    // num_threads(20) itrealvalue(21) starttime(22) vsize(23) → 8 个字段到 rss(24)
                    for (int i = 0; i < 8; ++i) { std::string skip; iss >> skip; }
                    iss >> rss_pages;
                }
            }
            // CPU 时间(秒)
            long hz = sysconf(_SC_CLK_TCK);
            double cpu_time = (utime + stime) / (double)hz;
            // RSS (KB)
            long rss_kb = rss_pages * (sysconf(_SC_PAGESIZE) / 1024);
            if (!first) os << ",";
            first = false;
            os << "{\"pid\":" << pid << ",\"ppid\":" << ppid
               << ",\"comm\":\"" << JsonEscape(comm) << "\""
               << ",\"state\":\"" << state << "\""
               << ",\"rss_kb\":" << rss_kb
               << ",\"cpu_s\":" << std::fixed << std::setprecision(1) << cpu_time
               << ",\"cmd\":\"" << JsonEscape(cmdline) << "\"}";
        }
        ::closedir(d);
    }
    os << "]}}";
    bridge.broadcastFrame(os.str());
}

// ── 网络管理(结构化接口表: getifaddrs + /sys/class/net + /proc/net/dev) ───
void PublishNetList(WebBridge &bridge)
{
    struct IfStat { unsigned long long rx = 0, tx = 0, rx_err = 0, tx_err = 0; };
    struct IfInfo {
        bool up = false, running = false, loopback = false;
        std::string operstate, mac;
        long mtu = 0, type = 0;
        std::vector<std::string> v4, v6;
        IfStat st;
    };
    std::map<std::string, IfInfo> ifaces;

    // 1) IP 地址 + IFF_UP/RUNNING/LOOPBACK 标志
    ifaddrs *ifa_list = nullptr;
    if (::getifaddrs(&ifa_list) == 0) {
        for (ifaddrs *ifa = ifa_list; ifa; ifa = ifa->ifa_next) {
            if (!ifa->ifa_name) continue;
            IfInfo &info = ifaces[ifa->ifa_name];
            info.up = info.up || (ifa->ifa_flags & IFF_UP);
            info.running = info.running || (ifa->ifa_flags & IFF_RUNNING);
            info.loopback = info.loopback || (ifa->ifa_flags & IFF_LOOPBACK);
            if (!ifa->ifa_addr) continue;
            char buf[INET6_ADDRSTRLEN] = {0};
            if (ifa->ifa_addr->sa_family == AF_INET) {
                if (::inet_ntop(AF_INET, &reinterpret_cast<sockaddr_in *>(ifa->ifa_addr)->sin_addr,
                                buf, INET_ADDRSTRLEN))
                    info.v4.push_back(buf);
            } else if (ifa->ifa_addr->sa_family == AF_INET6) {
                if (::inet_ntop(AF_INET6, &reinterpret_cast<sockaddr_in6 *>(ifa->ifa_addr)->sin6_addr,
                                buf, INET6_ADDRSTRLEN)) {
                    // 链路本地地址补 scope 后缀(与 ip addr 输出一致, 如 fe80::1%3)
                    if (reinterpret_cast<sockaddr_in6 *>(ifa->ifa_addr)->sin6_scope_id)
                        info.v6.push_back(std::string(buf) + "%" +
                                          std::to_string(reinterpret_cast<sockaddr_in6 *>(ifa->ifa_addr)->sin6_scope_id));
                    else
                        info.v6.push_back(buf);
                }
            }
        }
        ::freeifaddrs(ifa_list);
    }

    // 2) /sys/class/net: operstate / MAC / MTU / ARPHRD 类型
    if (DIR *d = ::opendir("/sys/class/net")) {
        struct dirent *de;
        while ((de = ::readdir(d)) != nullptr) {
            if (de->d_name[0] == '.') continue;
            const std::string name = de->d_name;
            const std::string base = "/sys/class/net/" + name;
            IfInfo &info = ifaces[name];
            { std::ifstream f(base + "/operstate"); std::getline(f, info.operstate); info.operstate = Trim(info.operstate); }
            { std::ifstream f(base + "/address");   std::getline(f, info.mac);      info.mac = Trim(info.mac); }
            { std::ifstream f(base + "/mtu"); f >> info.mtu; }
            { std::ifstream f(base + "/type"); long t = 0; if (f >> t) info.type = t; }
        }
        ::closedir(d);
    }

    // 3) /proc/net/dev: 收发字节与错误包(rx bytes packets errs drop fifo frame compressed multicast
    //    tx bytes packets errs ...)
    {
        std::ifstream f("/proc/net/dev");
        std::string line;
        std::getline(f, line); std::getline(f, line);  // 丢弃两行表头
        while (std::getline(f, line)) {
            auto cp = line.find(':');
            if (cp == std::string::npos) continue;
            const std::string name = Trim(line.substr(0, cp));
            if (name.empty()) continue;
            std::istringstream iss(line.substr(cp + 1));
            unsigned long long c[16] = {0};
            for (int i = 0; i < 16 && (iss >> c[i]); ++i) {}
            IfStat s;
            s.rx = c[0]; s.rx_err = c[2]; s.tx = c[8]; s.tx_err = c[10];
            ifaces[name].st = s;
        }
    }

    std::ostringstream os;
    os << "{\"type\":\"net_list\",\"data\":{\"ifaces\":[";
    bool first = true;
    for (const auto &kv : ifaces) {
        const std::string &name = kv.first;
        const IfInfo &n = kv.second;
        if (!first) os << ",";
        first = false;
        os << "{\"name\":\"" << JsonEscape(name) << "\""
           << ",\"up\":" << (n.up ? "true" : "false")
           << ",\"running\":" << (n.running ? "true" : "false")
           << ",\"loopback\":" << (n.loopback ? "true" : "false")
           << ",\"operstate\":\"" << JsonEscape(n.operstate) << "\""
           << ",\"mac\":\"" << JsonEscape(n.mac) << "\""
           << ",\"mtu\":" << n.mtu
           << ",\"type\":" << n.type
           << ",\"v4\":[";
        for (size_t i = 0; i < n.v4.size(); ++i) {
            if (i) os << ",";
            os << "\"" << JsonEscape(n.v4[i]) << "\"";
        }
        os << "],\"v6\":[";
        for (size_t i = 0; i < n.v6.size(); ++i) {
            if (i) os << ",";
            os << "\"" << JsonEscape(n.v6[i]) << "\"";
        }
        os << "],\"rx\":" << n.st.rx
           << ",\"tx\":" << n.st.tx
           << ",\"rx_err\":" << n.st.rx_err
           << ",\"tx_err\":" << n.st.tx_err << "}";
    }
    os << "]}}";
    bridge.broadcastFrame(os.str());
}

// ── 网络原始输出(等价 ip addr 命令行文本) ─────────────────────────────────
void PublishNetRaw(WebBridge &bridge)
{
    std::string out;
    std::vector<std::string> v = {"/usr/sbin/ip", "addr"};
    if (::access(v[0].c_str(), X_OK) != 0) v[0] = "/usr/bin/ip";
    if (::access(v[0].c_str(), X_OK) != 0) v[0] = "/bin/ip";
    if (::access(v[0].c_str(), X_OK) != 0) v[0] = "ip";
    RunShell(v, &out);
    std::ostringstream os;
    os << "{\"type\":\"net_raw\",\"data\":{\"lines\":[";
    std::istringstream iss(out);
    std::string line;
    bool first = true;
    while (std::getline(iss, line)) {
        if (!first) os << ",";
        first = false;
        os << "\"" << JsonEscape(line) << "\"";
    }
    os << "]}}";
    bridge.broadcastFrame(os.str());
}

// ── 设备管理器(/dev 顶层枚举, lstat 分类) ─────────────────────────────────
void PublishDevList(WebBridge &bridge)
{
    struct DevEntry {
        std::string name, type, link;
        unsigned major = 0, minor = 0;
        long long size = 0;
        long mtime = 0;
    };
    std::vector<DevEntry> entries;
    if (DIR *d = ::opendir("/dev")) {
        struct dirent *de;
        while ((de = ::readdir(d)) != nullptr) {
            if (de->d_name[0] == '.') continue;
            const std::string path = std::string("/dev/") + de->d_name;
            struct stat lst {};
            if (::lstat(path.c_str(), &lst) != 0) continue;
            DevEntry e;
            e.name = de->d_name;
            if (S_ISCHR(lst.st_mode)) e.type = "char";
            else if (S_ISBLK(lst.st_mode)) e.type = "block";
            else if (S_ISDIR(lst.st_mode)) e.type = "dir";
            else if (S_ISLNK(lst.st_mode)) e.type = "link";
            else if (S_ISFIFO(lst.st_mode)) e.type = "fifo";
            else if (S_ISSOCK(lst.st_mode)) e.type = "sock";
            else e.type = "file";
            if (S_ISCHR(lst.st_mode) || S_ISBLK(lst.st_mode)) {
                e.major = ::major(lst.st_rdev);
                e.minor = ::minor(lst.st_rdev);
            }
            e.size = static_cast<long long>(lst.st_size);
            e.mtime = static_cast<long>(lst.st_mtime);
            if (e.type == "link") {
                char buf[1024];
                ssize_t n = ::readlink(path.c_str(), buf, sizeof(buf) - 1);
                if (n > 0) { buf[n] = '\0'; e.link = buf; }
            }
            entries.push_back(std::move(e));
        }
        ::closedir(d);
    }
    std::sort(entries.begin(), entries.end(),
              [](const DevEntry &a, const DevEntry &b) { return a.name < b.name; });

    std::ostringstream os;
    os << "{\"type\":\"dev_list\",\"data\":{\"devs\":[";
    bool first = true;
    for (const auto &e : entries) {
        if (!first) os << ",";
        first = false;
        os << "{\"name\":\"" << JsonEscape(e.name) << "\",\"type\":\"" << e.type << "\""
           << ",\"major\":" << e.major << ",\"minor\":" << e.minor
           << ",\"size\":" << e.size
           << ",\"mtime\":" << e.mtime
           << ",\"link\":\"" << JsonEscape(e.link) << "\"}";
    }
    os << "]}}";
    bridge.broadcastFrame(os.str());
}

// ── 网络配置(netcfg): netplan 持久化多IP/掩码/网关/DNS ────────────────────
// 前端命令:
//   netcfg get            → 推送 net_config 帧(接口/IP/网关/DNS/覆盖文件快照)
//   netcfg apply\n<行协议> → 写 /etc/netplan/90-jxg-net.yaml + netplan apply
//       iface <name>
//       ip <a.b.c.d> <prefix>
//       gw <a.b.c.d>      (可选,缺省不设默认路由)
//       dns <d1,d2,...>   (可选)
//   netcfg restore        → 删除覆盖文件,回到 10-dhcp-all-interfaces.yaml
static const char *kNetCfgPath = "/etc/netplan/90-jxg-net.yaml";

static bool IsValidIPv4(const std::string &s)
{
    in_addr a;
    return ::inet_pton(AF_INET, s.c_str(), &a) == 1;
}

static bool ListNetIfaces(std::vector<std::string> &out)
{
    out.clear();
    if (DIR *d = ::opendir("/sys/class/net")) {
        while (dirent *de = ::readdir(d)) {
            if (de->d_name[0] == '.') continue;
            out.emplace_back(de->d_name);
        }
        ::closedir(d);
    }
    std::sort(out.begin(), out.end());
    return !out.empty();
}

void PublishNetConfig(WebBridge &bridge)
{
    std::vector<std::string> names;
    ListNetIfaces(names);

    // 每接口 IPv4 addr/prefix(netmask → 前缀长度)
    std::map<std::string, std::vector<std::pair<std::string, int>>> v4;
    ifaddrs *ifa_list = nullptr;
    if (::getifaddrs(&ifa_list) == 0) {
        for (ifaddrs *ifa = ifa_list; ifa; ifa = ifa->ifa_next) {
            if (!ifa->ifa_name || !ifa->ifa_addr || ifa->ifa_addr->sa_family != AF_INET) continue;
            char ab[INET_ADDRSTRLEN] = {0};
            if (!::inet_ntop(AF_INET, &reinterpret_cast<sockaddr_in *>(ifa->ifa_addr)->sin_addr, ab, sizeof(ab)))
                continue;
            int prefix = 0;
            if (ifa->ifa_netmask) {
                uint32_t m = ntohl(reinterpret_cast<sockaddr_in *>(ifa->ifa_netmask)->sin_addr.s_addr);
                while (m & 0x80000000u) { ++prefix; m <<= 1; }
            }
            v4[ifa->ifa_name].emplace_back(ab, prefix);
        }
        ::freeifaddrs(ifa_list);
    }

    // 默认网关(/proc/net/route 首条 Destination==0 且 Gateway!=0 的行, 小端 hex)
    std::string gw_iface, gw_ip;
    {
        std::ifstream f("/proc/net/route");
        std::string line;
        std::getline(f, line);  // 丢弃表头
        while (std::getline(f, line)) {
            std::istringstream iss(line);
            std::string iface, dest, gate;
            if (!(iss >> iface >> dest >> gate)) continue;
            if (dest != "00000000" || gate == "00000000") continue;
            uint32_t g = 0;
            std::istringstream hx(gate);
            if (hx >> std::hex >> g) {  // 文本即网络序数值, 无需字节交换
                char b[INET_ADDRSTRLEN] = {0};
                if (::inet_ntop(AF_INET, &g, b, sizeof(b))) { gw_iface = iface; gw_ip = b; }
            }
            break;
        }
    }

    // 全局 DNS(systemd-resolved 运行态)
    std::vector<std::string> dns;
    {
        std::ifstream f("/run/systemd/resolve/resolv.conf");
        std::string line;
        while (std::getline(f, line)) {
            auto pos = line.find("nameserver");
            if (pos == std::string::npos) continue;
            std::string rest = Trim(line.substr(pos + 10));
            if (!rest.empty() && rest.find(' ') == std::string::npos && IsValidIPv4(rest))
                dns.push_back(rest);
        }
    }

    // 静态覆盖文件内容(存在=该接口已被静态接管)
    std::string override_yaml;
    {
        std::ifstream f(kNetCfgPath);
        if (f) { std::ostringstream oss; oss << f.rdbuf(); override_yaml = oss.str(); }
    }

    // 运行态静态路由(ip route show table all)
    struct RouteRt { std::string to, via, table; };
    std::vector<RouteRt> routes_rt;
    {
        std::string ipout;
        RunShell({"ip", "-4", "route", "show", "table", "all"}, &ipout);
        std::istringstream riss(ipout);
        std::string rl;
        while (std::getline(riss, rl)) {
            if (rl.empty()) continue;
            std::istringstream liss(rl);
            std::string dst;
            liss >> dst;
            if (dst == "default" || dst == "local" || dst == "broadcast") continue;
            if (dst.find('/') == std::string::npos) continue;  // 只保留 CIDR 目标
            std::string tail; std::getline(liss, tail); tail = Trim(tail);
            size_t tp = tail.find(" table ");
            std::string table;
            if (tp != std::string::npos) { std::istringstream ts(tail.substr(tp + 7)); ts >> table; }
            if (table == "local") continue;  // 跳过 local 表的自动条目
            RouteRt r; r.to = dst; r.table = table;
            size_t wp = tail.find(" via ");
            if (wp != std::string::npos) { std::istringstream vs(tail.substr(wp + 5)); vs >> r.via; }
            routes_rt.push_back(r);
        }
    }

    // 运行态策略路由(ip rule show)
    struct RuleRt { std::string from, table; };
    std::vector<RuleRt> rules_rt;
    {
        std::string ipout;
        RunShell({"ip", "rule", "show"}, &ipout);
        std::istringstream riss(ipout);
        std::string rl;
        while (std::getline(riss, rl)) {
            auto pos = rl.find(":");
            if (pos == std::string::npos) continue;
            std::string tail = Trim(rl.substr(pos + 1));
            RuleRt rule;
            size_t fp = tail.find("from ");
            if (fp != std::string::npos) { std::istringstream vs(tail.substr(fp + 5)); vs >> rule.from; }
            size_t tp = tail.find("lookup ");
            if (tp != std::string::npos) { std::istringstream vs(tail.substr(tp + 7)); vs >> rule.table; }
            // 跳过内置规则
            if (rule.from == "all") continue;
            if (rule.table == "local" || rule.table == "main" || rule.table == "default") continue;
            if (!rule.from.empty() || !rule.table.empty()) rules_rt.push_back(rule);
        }
    }

    std::ostringstream os;
    os << "{\"type\":\"net_config\",\"data\":{\"ifaces\":[";
    for (size_t i = 0; i < names.size(); ++i) {
        if (i) os << ",";
        os << "{\"name\":\"" << JsonEscape(names[i]) << "\",\"ips\":[";
        auto it = v4.find(names[i]);
        if (it != v4.end())
            for (size_t j = 0; j < it->second.size(); ++j) {
                if (j) os << ",";
                os << "{\"addr\":\"" << JsonEscape(it->second[j].first)
                   << "\",\"prefix\":" << it->second[j].second << "}";
            }
        os << "]}";
    }
    os << "],\"gateway\":{\"iface\":\"" << JsonEscape(gw_iface)
       << "\",\"ip\":\"" << JsonEscape(gw_ip) << "\"},\"dns\":[";
    for (size_t i = 0; i < dns.size(); ++i) {
        if (i) os << ",";
        os << "\"" << JsonEscape(dns[i]) << "\"";
    }
    os << "],\"routes\":[";
    for (size_t i = 0; i < routes_rt.size(); ++i) {
        if (i) os << ",";
        os << "{\"to\":\"" << JsonEscape(routes_rt[i].to) << "\"";
        if (!routes_rt[i].via.empty()) os << ",\"via\":\"" << JsonEscape(routes_rt[i].via) << "\"";
        if (!routes_rt[i].table.empty()) os << ",\"table\":" << routes_rt[i].table;
        os << "}";
    }
    os << "],\"rules\":[";
    for (size_t i = 0; i < rules_rt.size(); ++i) {
        if (i) os << ",";
        os << "{\"from\":\"" << JsonEscape(rules_rt[i].from) << "\"";
        if (!rules_rt[i].table.empty()) os << ",\"table\":" << rules_rt[i].table;
        os << "}";
    }
    os << "],\"override\":";
    if (override_yaml.empty()) os << "null";
    else os << "\"" << JsonEscape(override_yaml) << "\"";
    os << "}}";
    bridge.broadcastFrame(os.str());
}

// 行协议 → netplan yaml; 返回 false 时 err 给出可读原因
static bool BuildNetplanYaml(const std::string &body, std::string &yaml, std::string &err)
{
    std::string iface, gw, dns_csv;
    std::vector<std::pair<std::string, int>> ips;
    std::istringstream iss(body);
    std::string line;
    while (std::getline(iss, line)) {
        line = Trim(line);
        if (line.empty()) continue;
        std::istringstream ls(line);
        std::string key;
        if (!(ls >> key)) continue;
        if (key == "iface") {
            if (!(ls >> iface)) { err = "iface 行缺少接口名"; return false; }
        } else if (key == "ip") {
            std::string a;
            int p = 24;
            if (!(ls >> a)) { err = "ip 行缺少地址"; return false; }
            ls >> p;
            if (!IsValidIPv4(a)) { err = "IP 格式错误: " + a; return false; }
            if (p < 1 || p > 32) { err = "前缀长度非法(1-32): " + std::to_string(p); return false; }
            ips.emplace_back(a, p);
        } else if (key == "gw") {
            if (!(ls >> gw)) { err = "gw 行缺少地址"; return false; }
        } else if (key == "dns") {
            std::string rest;
            std::getline(ls, rest);
            rest = Trim(rest);
            size_t st = 0;
            while (st < rest.size()) {
                auto en = rest.find(',', st);
                std::string one = Trim(rest.substr(st, en == std::string::npos ? std::string::npos : en - st));
                if (!one.empty()) dns_csv += (dns_csv.empty() ? "" : ",") + one;
                if (en == std::string::npos) break;
                st = en + 1;
            }
        }
    }
    if (iface.empty() || ips.empty()) { err = "缺少接口或至少一个 IP"; return false; }
    for (char c : iface)
        if (!std::isalnum((unsigned char)c) && c != '.' && c != '_' && c != '-') {
            err = "接口名含非法字符"; return false;
        }
    std::vector<std::string> names;
    ListNetIfaces(names);
    if (std::find(names.begin(), names.end(), iface) == names.end()) {
        err = "接口不存在: " + iface; return false;
    }
    if (!gw.empty() && !IsValidIPv4(gw)) { err = "网关格式错误: " + gw; return false; }
    {
        size_t st = 0;
        while (st < dns_csv.size()) {
            auto en = dns_csv.find(',', st);
            std::string one = Trim(dns_csv.substr(st, en == std::string::npos ? std::string::npos : en - st));
            if (!one.empty() && !IsValidIPv4(one)) { err = "DNS 格式错误: " + one; return false; }
            if (en == std::string::npos) break;
            st = en + 1;
        }
    }

    yaml = "# generated by JXG web console (netcfg)\nnetwork:\n  version: 2\n"
           "  renderer: networkd\n  ethernets:\n    " + iface + ":\n"
           "      dhcp4: false\n      dhcp6: false\n      accept-ra: false\n      addresses:\n";
    for (auto &pr : ips)
        yaml += "        - " + pr.first + "/" + std::to_string(pr.second) + "\n";
    if (!gw.empty())
        yaml += "      routes:\n        - to: default\n          via: " + gw + "\n";
    if (!dns_csv.empty())
        yaml += "      nameservers:\n        addresses: [" + dns_csv + "]\n";
    return true;
}

bool HandleNetConfigCommand(WebBridge &bridge, const std::string &text)
{
    if (text == "netcfg get") {
        std::thread([&bridge] { PublishNetConfig(bridge); }).detach();
        return true;
    }
    if (text.rfind("netcfg apply\n", 0) == 0) {
        std::string body = text.substr(13);
        std::thread([&bridge, body] {
            std::string yaml, err;
            bool ok = false;
            std::string msg;
            if (!BuildNetplanYaml(body, yaml, err)) {
                msg = "配置校验失败: " + err;
            } else {
                {   // 备份旧覆盖文件
                    std::ifstream src(kNetCfgPath);
                    if (src) { std::ofstream dst(std::string(kNetCfgPath) + ".bak"); dst << src.rdbuf(); }
                }
                std::ofstream f(kNetCfgPath);
                f << yaml;
                f.close();
                msg = "已写入 " + std::string(kNetCfgPath);
                ok = true;
            }
            if (ok) {
                std::string out;
                std::vector<std::string> v = {"/usr/sbin/netplan", "apply"};
                if (::access(v[0].c_str(), X_OK) != 0) v[0] = "netplan";
                RunShell(v, &out);
                msg += "; netplan apply 已执行";
                if (!out.empty()) msg += ": " + out.substr(0, 300);
            }
            std::ostringstream os;
            os << "{\"type\":\"net_config_result\",\"data\":{\"ok\":"
               << (ok ? "true" : "false") << ",\"message\":\"" << JsonEscape(msg) << "\"}}";
            bridge.broadcastFrame(os.str());
            if (ok)  // 等 networkd 生效后推送新状态
                std::thread([&bridge] {
                    std::this_thread::sleep_for(std::chrono::seconds(2));
                    PublishNetConfig(bridge);
                }).detach();
        }).detach();
        return true;
    }
    if (text == "netcfg restore") {
        std::thread([&bridge] {
            ::unlink(kNetCfgPath);
            std::string out;
            std::vector<std::string> v = {"/usr/sbin/netplan", "apply"};
            if (::access(v[0].c_str(), X_OK) != 0) v[0] = "netplan";
            RunShell(v, &out);
            std::ostringstream os;
            os << "{\"type\":\"net_config_result\",\"data\":{\"ok\":true,\"message\":\""
               << JsonEscape(std::string("已删除静态覆盖,恢复 DHCP(10-dhcp-all-interfaces.yaml); netplan apply 已执行"))
               << "\"}}";
            bridge.broadcastFrame(os.str());
            std::thread([&bridge] {
                std::this_thread::sleep_for(std::chrono::seconds(2));
                PublishNetConfig(bridge);
            }).detach();
        }).detach();
        return true;
    }
    return false;
}

// ── 服务内容读取(用于编辑) ─────────────────────────────────────────────────
void PublishServiceFile(WebBridge &bridge, const std::string &unit)
{
    std::string name = unit;
    if (name.size() < 8 || name.substr(name.size() - 8) != ".service") name += ".service";
    std::string path = "/etc/systemd/system/" + name;
    std::ifstream f(path);
    std::ostringstream oss;
    oss << f.rdbuf();
    std::ostringstream os;
    os << "{\"type\":\"svc_file\",\"data\":{"
       << "\"unit\":\"" << JsonEscape(name) << "\","
       << "\"content\":\"" << JsonEscape(oss.str()) << "\"}}";
    bridge.broadcastFrame(os.str());
}

// 处理 "svc <op>" 命令; 返回 true 表示已消费(不转发 ROS)
bool HandleServiceCommand(WebBridge &bridge, const std::string &text)
{
    // 精确匹配 "svc <子命令>"
    if (text != "svc" && text.rfind("svc ", 0) != 0) return false;
    std::string op = (text == "svc") ? "status" : Trim(text.substr(4));

    // ── 新扩展: list / file / create / delete ──
    if (op == "list") {
        std::thread([&bridge] { PublishServiceList(bridge); }).detach();
        return true;
    }
    if (op.rfind("file ", 0) == 0) {
        std::string unit = Trim(op.substr(5));
        std::thread([&bridge, unit] { PublishServiceFile(bridge, unit); }).detach();
        return true;
    }
    if (op.rfind("log ", 0) == 0) {
        std::string unit = Trim(op.substr(4));
        if (!unit.empty()) {
            std::thread([&bridge, unit] {
                std::string out;
                std::vector<std::string> v = {"/usr/bin/journalctl", "-u", unit, "-n", "500", "--no-pager"};
                if (::access(v[0].c_str(), X_OK) != 0) v[0] = "journalctl";
                RunShell(v, &out);
                std::ostringstream os;
                os << "{\"type\":\"svc_log\",\"data\":{\"unit\":\"" << JsonEscape(unit) << "\",\"lines\":[";
                std::istringstream iss(out);
                std::string line;
                bool first = true;
                while (std::getline(iss, line)) {
                    if (!first) os << ",";
                    first = false;
                    os << "\"" << JsonEscape(line) << "\"";
                }
                os << "]}}";
                bridge.broadcastFrame(os.str());
            }).detach();
        }
        return true;
    }
    if (op.rfind("create ", 0) == 0) {
        // create <unit>\n<content>
        std::string rest = op.substr(7);
        auto nl = rest.find('\n');
        std::string unit = (nl != std::string::npos) ? Trim(rest.substr(0, nl)) : Trim(rest);
        std::string content = (nl != std::string::npos) ? rest.substr(nl + 1) : "";
        if (!unit.empty()) {
            if (unit.size() < 8 || unit.substr(unit.size() - 8) != ".service") unit += ".service";
            std::string path = "/etc/systemd/system/" + unit;
            std::ofstream f(path);
            f << content;
            f.close();
            SystemCtl("daemon-reload", nullptr, "");
        }
        std::thread([&bridge] { PublishServiceList(bridge); }).detach();
        return true;
    }
    if (op.rfind("delete ", 0) == 0) {
        std::string unit = Trim(op.substr(7));
        if (!unit.empty()) {
            if (unit.size() < 8 || unit.substr(unit.size() - 8) != ".service") unit += ".service";
            std::string path = "/etc/systemd/system/" + unit;
            ::unlink(path.c_str());
            SystemCtl("daemon-reload", nullptr, "");
        }
        std::thread([&bridge] { PublishServiceList(bridge); }).detach();
        return true;
    }
    if (op.rfind("save ", 0) == 0) {
        // save <unit>\n<content>
        std::string rest = op.substr(5);
        auto nl = rest.find('\n');
        std::string unit = (nl != std::string::npos) ? Trim(rest.substr(0, nl)) : Trim(rest);
        std::string content = (nl != std::string::npos) ? rest.substr(nl + 1) : "";
        if (!unit.empty()) {
            if (unit.size() < 8 || unit.substr(unit.size() - 8) != ".service") unit += ".service";
            std::string path = "/etc/systemd/system/" + unit;
            std::ofstream f(path);
            f << content;
            f.close();
            SystemCtl("daemon-reload", nullptr, "");
        }
        std::thread([&bridge, unit] { PublishServiceFile(bridge, unit); }).detach();
        return true;
    }

    // ── 支持任意服务: op 可能是 "start xxx.service" ──
    // 解析 op 和可选的 unit
    std::string unit = kUnitName;
    auto sp = op.find(' ');
    if (sp != std::string::npos) {
        std::string real_op = Trim(op.substr(0, sp));
        std::string extra = Trim(op.substr(sp + 1));
        if (!extra.empty()) unit = extra;
        op = real_op;
    }

    static const std::vector<std::string> kOps =
        {"status", "start", "stop", "restart", "enable", "disable"};
    bool known = false;
    for (const auto &k : kOps) if (op == k) { known = true; break; }
    if (!known) {
        PublishServiceStatus(bridge, op, false, "unknown svc op: " + op, false);
        return true;
    }

    if (op == "status") {
        // 查询很快, 但仍放到独立线程避免阻塞 WS net loop(systemctl 偶发卡顿)
        std::thread([&bridge, unit] { 
            if (unit == kUnitName) PublishServiceStatus(bridge, "status", true, "", false);
            else {
                std::string out;
                SystemCtl("is-active", &out, unit);
                std::string active = Trim(out);
                SystemCtl("is-enabled", &out, unit);
                std::string enabled = Trim(out);
                std::ostringstream os;
                os << "{\"type\":\"svc_status\",\"data\":{"
                   << "\"unit\":\"" << JsonEscape(unit) << "\","
                   << "\"installed\":true,"
                   << "\"active\":\"" << JsonEscape(active) << "\","
                   << "\"enabled\":\"" << JsonEscape(enabled) << "\","
                   << "\"op\":\"status\",\"ok\":true,\"error\":\"\",\"busy\":false}}";
                bridge.broadcastFrame(os.str());
            }
        }).detach();
        return true;
    }

    // 变更类操作串行化: restart 优雅停机可达十余秒
    bool expected = false;
    if (!g_svc_busy.compare_exchange_strong(expected, true)) {
        PublishServiceStatus(bridge, op, false, "另一个服务操作正在执行,请稍候", true);
        return true;
    }
    std::thread([&bridge, op, unit] {
        // 先广播一次 busy, 前端按钮进入等待态
        PublishServiceStatus(bridge, op, true, "", true);
        std::string out;
        int rc = SystemCtl(op, &out, unit);
        if (op == "stop") {
            SystemCtl("reset-failed", nullptr, unit);
            rc = 0;
        }
        if ((op == "start" || op == "restart") && rc == 0 && unit == kUnitName)
            std::this_thread::sleep_for(std::chrono::seconds(6));
        PublishServiceStatus(bridge, op, rc == 0,
                             rc == 0 ? "" : ("systemctl " + op + " 失败(rc=" + std::to_string(rc) + "): " + out),
                             false);
        g_svc_busy.store(false);
    }).detach();
    return true;
}

// 处理 "sysinfo" / "ps" 命令
bool HandleSysCommand(WebBridge &bridge, const std::string &text)
{
    if (text == "sysinfo") {
        std::thread([&bridge] { PublishSysInfo(bridge); }).detach();
        return true;
    }
    if (text == "syslog" || text.rfind("syslog ", 0) == 0) {
        // 系统启动以来的日志 journalctl -b
        std::string arg = (text == "syslog") ? "" : Trim(text.substr(7));
        int lines = 2000;
        if (!arg.empty()) lines = std::atoi(arg.c_str());
        if (lines <= 0) lines = 2000;
        if (lines > 10000) lines = 10000;
        std::thread([&bridge, lines] {
            std::string out;
            std::vector<std::string> v = {"/usr/bin/journalctl", "-b", "--no-pager", "-n", std::to_string(lines)};
            if (::access(v[0].c_str(), X_OK) != 0) v[0] = "journalctl";
            RunShell(v, &out);
            std::ostringstream os;
            os << "{\"type\":\"syslog\",\"data\":{\"lines\":[";
            std::istringstream iss(out);
            std::string line;
            bool first = true;
            while (std::getline(iss, line)) {
                if (!first) os << ",";
                first = false;
                os << "\"" << JsonEscape(line) << "\"";
            }
            os << "]}}";
            bridge.broadcastFrame(os.str());
        }).detach();
        return true;
    }
    if (text == "ps" || text.rfind("ps ", 0) == 0) {
        std::string op = (text == "ps") ? "list" : Trim(text.substr(3));
        if (op == "list") {
            std::thread([&bridge] { PublishProcessList(bridge); }).detach();
            return true;
        }
        if (op.rfind("kill ", 0) == 0) {
            int pid = std::atoi(Trim(op.substr(5)).c_str());
            if (pid > 0) ::kill(pid, SIGTERM);
            std::thread([&bridge] { PublishProcessList(bridge); }).detach();
            return true;
        }
        if (op.rfind("kill9 ", 0) == 0) {
            int pid = std::atoi(Trim(op.substr(6)).c_str());
            if (pid > 0) ::kill(pid, SIGKILL);
            std::thread([&bridge] { PublishProcessList(bridge); }).detach();
            return true;
        }
        if (op.rfind("start ", 0) == 0) {
            std::string cmd = Trim(op.substr(6));
            if (!cmd.empty()) {
                pid_t pid = ::fork();
                if (pid == 0) {
                    ::setsid();
                    ::execl("/bin/sh", "sh", "-c", cmd.c_str(), (char *)nullptr);
                    ::_exit(127);
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            std::thread([&bridge] { PublishProcessList(bridge); }).detach();
            return true;
        }
    }
    // 网络管理: net list(结构化接口表) / net raw(ip addr 原始输出)
    if (text == "net" || text.rfind("net ", 0) == 0) {
        std::string op = (text == "net") ? "list" : Trim(text.substr(4));
        if (op == "raw") {
            std::thread([&bridge] { PublishNetRaw(bridge); }).detach();
        } else {  // 默认 list
            std::thread([&bridge] { PublishNetList(bridge); }).detach();
        }
        return true;
    }
    // 设备管理器: /dev 顶层枚举
    if (text == "dev" || text.rfind("dev ", 0) == 0) {
        std::thread([&bridge] { PublishDevList(bridge); }).detach();
        return true;
    }
    return false;
}

// ════════════════════════════════════════════════════════════════════════════
//  遥控学习 / 克隆(CH340 USB 串口,独立线程,与 200Hz 控制环完全隔离)
//
//  硬件: CH340 串口(by-id 稳定软链,回退 /dev/ttyUSB0)。9600 8N1 raw,
//        无硬件流控。
//  协议: 接收 = 其它遥控器发出的 3 字节帧;发送(克隆)= 前置 00 03 共 5 字节。
//  持久化: <项目根>/config/rc_codes.json(web_server 以 WEB_ROOT 的上一级为项目根)。
//  帧收集: poll + 50ms 帧间隔超时(>=50ms 无新字节视为一帧结束),恰 3 字节才算码。
// ════════════════════════════════════════════════════════════════════════════
constexpr const char *kRcDevById = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0";
constexpr const char *kRcDevFallback = "/dev/ttyUSB0";
constexpr const char *kRcCodesRel = "/config/rc_codes.json";
constexpr int kRcFrameBytes = 3;       // 有效帧长度
constexpr int kRcGapMs = 50;           // 帧间隔超时(ms)
constexpr int kRcRepeatGapMs = 200;    // 连续发送间隔(ms)
constexpr int kRcReopenMs = 5000;      // 设备不可用时的重试打开周期(ms)
constexpr int kRcMaxRepeat = 100;      // 单次连发上限

struct RcEntry {
    int id = 0;
    std::string name;
    unsigned char b[3] = {0, 0, 0};
    long sends = 0;
    std::string time;
};

std::mutex g_rc_mutex;                  // 保护下方 g_rc_* 全部可变状态
std::vector<RcEntry> g_rc_entries;      // 已学码列表
int g_rc_next_id = 1;
bool g_rc_learning = false;             // 学习开关
bool g_rc_has_capture = false;          // 是否有待保存捕获
unsigned char g_rc_capture[3] = {0, 0, 0};
long g_rc_last_id = -1;                 // 最近一次发送过的条目 id(-1=无)
int g_rc_fd = -1;                       // 串口 fd(<0=设备不可用)
std::string g_rc_dev_path = kRcDevById; // 当前尝试/使用的设备路径
std::string g_rc_dev_err;               // 最近一次打开失败原因(""=正常)
std::atomic<bool> g_rc_run{false};
std::thread g_rc_thread;
std::string g_rc_codes_path;            // rc_codes.json 绝对路径

// 3 字节(或杂帧前若干字节)格式化为 "50 3C 7A"
std::string RcHex(const unsigned char *b, int n)
{
    char buf[32];
    int off = 0;
    for (int i = 0; i < n && i < 8 && off < (int)sizeof(buf) - 4; ++i)
        off += std::snprintf(buf + off, sizeof(buf) - off, i ? " %02X" : "%02X", b[i]);
    buf[off] = '\0';
    return buf;
}

std::string RcNowString()
{
    std::time_t t = std::time(nullptr);
    std::tm tm{};
    localtime_r(&t, &tm);
    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &tm);
    return buf;
}

// ── 极简 JSON 取值/转义(仅用于本文件自写的 rc_codes.json,不追求通用) ──
std::string RcJsonUnescape(const std::string &s)
{
    std::string o;
    o.reserve(s.size());
    for (size_t i = 0; i < s.size(); ++i) {
        if (s[i] == '\\' && i + 1 < s.size()) {
            char c = s[++i];
            switch (c) {
                case 'n': o += '\n'; break;
                case 'r': o += '\r'; break;
                case 't': o += '\t'; break;
                case 'u': {
                    if (i + 4 < s.size()) {
                        long v = std::strtol(s.substr(i + 1, 4).c_str(), nullptr, 16);
                        if (v < 0x80) o += char(v);
                        else if (v < 0x800) { o += char(0xC0 | (v >> 6)); o += char(0x80 | (v & 0x3F)); }
                        else { o += char(0xE0 | (v >> 12)); o += char(0x80 | ((v >> 6) & 0x3F)); o += char(0x80 | (v & 0x3F)); }
                        i += 4;
                    }
                    break;
                }
                default: o += c;
            }
        } else o += s[i];
    }
    return o;
}

// 在 obj(单个 JSON 对象文本)中取字符串键
std::string RcJsonStr(const std::string &obj, const std::string &key, const std::string &def = "")
{
    std::string pat = "\"" + key + "\"";
    auto p = obj.find(pat);
    if (p == std::string::npos) return def;
    p = obj.find(':', p + pat.size());
    if (p == std::string::npos) return def;
    p = obj.find('"', p + 1);
    if (p == std::string::npos) return def;
    std::string raw;
    for (size_t i = p + 1; i < obj.size(); ++i) {
        if (obj[i] == '\\') { raw += obj[i]; if (i + 1 < obj.size()) raw += obj[++i]; continue; }
        if (obj[i] == '"') break;
        raw += obj[i];
    }
    return RcJsonUnescape(raw);
}

long RcJsonNum(const std::string &obj, const std::string &key, long def = 0)
{
    std::string pat = "\"" + key + "\"";
    auto p = obj.find(pat);
    if (p == std::string::npos) return def;
    p = obj.find(':', p + pat.size());
    if (p == std::string::npos) return def;
    return std::strtol(obj.c_str() + p + 1, nullptr, 10);
}

// 严格解析 hex → 3 字节:允许空格/逗号/冒号/短横/下划线/点作分隔,大小写不限;
// 去掉分隔符后必须恰好 6 个十六进制字符(即 3 字节,每字节 0~255),否则返回 false。
bool RcParseHex(const std::string &s, unsigned char out[3])
{
    std::string hex;
    hex.reserve(6);
    for (char c : s) {
        unsigned char uc = (unsigned char)c;
        if (std::isxdigit(uc)) hex += (char)std::toupper(uc);
        else if (std::isspace(uc) || c == ',' || c == ':' || c == '-' || c == '_' || c == '.')
            continue;                          // 允许的分隔符
        else return false;                     // 出现非十六进制/非分隔字符
    }
    if (hex.size() != 6) return false;         // 必须恰好 3 字节
    for (int i = 0; i < 3; ++i)
        out[i] = (unsigned char)std::strtol(hex.substr(i * 2, 2).c_str(), nullptr, 16);
    return true;
}

// 落盘(调用者须持有 g_rc_mutex)
bool RcSaveFileLocked()
{
    if (g_rc_codes_path.empty()) return false;
    auto slash = g_rc_codes_path.rfind('/');
    if (slash != std::string::npos) ::mkdir(g_rc_codes_path.substr(0, slash).c_str(), 0755);
    std::ofstream f(g_rc_codes_path);
    if (!f) return false;
    f << "{\n  \"next_id\": " << g_rc_next_id << ",\n  \"entries\": [\n";
    for (size_t i = 0; i < g_rc_entries.size(); ++i) {
        const RcEntry &e = g_rc_entries[i];
        f << "    {\"id\": " << e.id
          << ", \"name\": \"" << JsonEscape(e.name) << "\""
          << ", \"hex\": \"" << RcHex(e.b, 3) << "\""
          << ", \"sends\": " << e.sends
          << ", \"time\": \"" << JsonEscape(e.time) << "\"}";
        if (i + 1 < g_rc_entries.size()) f << ",";
        f << "\n";
    }
    f << "  ]\n}\n";
    return true;
}

// 载入(调用者须持有 g_rc_mutex)。解析失败/文件缺失均视为空表,不报错
void RcLoadFileLocked()
{
    g_rc_entries.clear();
    g_rc_next_id = 1;
    std::ifstream f(g_rc_codes_path);
    if (!f) return;
    std::string text((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    auto pos = text.find("\"entries\"");
    if (pos == std::string::npos) return;
    pos = text.find('[', pos);
    if (pos == std::string::npos) return;
    size_t i = pos + 1;
    while (i < text.size()) {
        while (i < text.size() && (std::isspace((unsigned char)text[i]) || text[i] == ',')) ++i;
        if (i >= text.size() || text[i] == ']') break;
        if (text[i] != '{') { ++i; continue; }
        int depth = 0;
        size_t start = i;
        while (i < text.size()) {
            if (text[i] == '{') ++depth;
            else if (text[i] == '}') { --depth; if (depth == 0) { ++i; break; } }
            ++i;
        }
        std::string obj = text.substr(start, i - start);
        RcEntry e;
        e.id = (int)RcJsonNum(obj, "id", 0);
        e.name = RcJsonStr(obj, "name");
        e.sends = RcJsonNum(obj, "sends", 0);
        e.time = RcJsonStr(obj, "time");
        if (!RcParseHex(RcJsonStr(obj, "hex"), e.b)) continue;
        if (e.id <= 0) e.id = g_rc_next_id;
        g_rc_entries.push_back(e);
        if (e.id >= g_rc_next_id) g_rc_next_id = e.id + 1;
    }
}

// 尝试打开串口(调用者须持有 g_rc_mutex)
void RcOpenLocked()
{
    g_rc_dev_err.clear();
    for (const char *path : {kRcDevById, kRcDevFallback}) {
        if (::access(path, R_OK | W_OK) != 0) continue;
        int fd = ::open(path, O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd < 0) { g_rc_dev_err = std::string(path) + ": " + std::strerror(errno); continue; }
        termios tio{};
        if (::tcgetattr(fd, &tio) != 0) {
            g_rc_dev_err = std::string(path) + ": tcgetattr: " + std::strerror(errno);
            ::close(fd);
            continue;
        }
        ::cfmakeraw(&tio);                 // raw 模式
        ::cfsetispeed(&tio, B9600);
        ::cfsetospeed(&tio, B9600);        // 9600 8N1
        tio.c_cflag |= (CLOCAL | CREAD);
        tio.c_cflag &= ~CRTSCTS;           // 关闭硬件流控
        tio.c_cc[VMIN] = 1;
        tio.c_cc[VTIME] = 0;
        if (::tcsetattr(fd, TCSANOW, &tio) != 0) {
            g_rc_dev_err = std::string(path) + ": tcsetattr: " + std::strerror(errno);
            ::close(fd);
            continue;
        }
        ::tcflush(fd, TCIOFLUSH);
        g_rc_fd = fd;
        g_rc_dev_path = path;
        g_rc_dev_err.clear();
        return;
    }
    if (g_rc_dev_err.empty())
        g_rc_dev_err = std::string(kRcDevById) + " 不存在(未插遥控接收器?)";
}

// 统一结果回报
void RcResult(WebBridge *bridge, bool ok, const char *action, const std::string &msg)
{
    std::ostringstream os;
    os << "{\"type\":\"rc_result\",\"ok\":" << (ok ? "true" : "false")
       << ",\"action\":\"" << action << "\",\"msg\":\"" << JsonEscape(msg) << "\"}";
    bridge->broadcastFrame(os.str());
}

void RcPublishList(WebBridge *bridge)
{
    std::ostringstream os;
    os << "{\"type\":\"rc_list\",\"data\":{\"entries\":[";
    {
        std::lock_guard<std::mutex> lk(g_rc_mutex);
        bool first = true;
        for (const auto &e : g_rc_entries) {
            if (!first) os << ",";
            first = false;
            os << "{\"id\":" << e.id << ",\"name\":\"" << JsonEscape(e.name)
               << "\",\"hex\":\"" << RcHex(e.b, 3) << "\",\"sends\":" << e.sends
               << ",\"time\":\"" << JsonEscape(e.time) << "\"}";
        }
    }
    os << "]}}";
    bridge->broadcastFrame(os.str());
}

void RcPublishInfo(WebBridge *bridge)
{
    int fd_val, count;
    bool learning = false, cap = false;
    std::string dev, err;
    unsigned char capb[3] = {0, 0, 0};
    {
        std::lock_guard<std::mutex> lk(g_rc_mutex);
        fd_val = g_rc_fd; dev = g_rc_dev_path; err = g_rc_dev_err;
        learning = g_rc_learning; cap = g_rc_has_capture;
        std::memcpy(capb, g_rc_capture, 3);
        count = (int)g_rc_entries.size();
    }
    std::ostringstream os;
    os << "{\"type\":\"rc_info\",\"data\":{"
       << "\"available\":" << (fd_val >= 0 ? "true" : "false") << ","
       << "\"dev\":\"" << JsonEscape(dev) << "\","
       << "\"byid_exists\":" << (::access(kRcDevById, F_OK) == 0 ? "true" : "false") << ","
       << "\"fallback_exists\":" << (::access(kRcDevFallback, F_OK) == 0 ? "true" : "false") << ","
       << "\"baud\":9600,"
       << "\"learning\":" << (learning ? "true" : "false") << ","
       << "\"count\":" << count << ","
       << "\"has_capture\":" << (cap ? "true" : "false") << ","
       << "\"capture_hex\":\"" << (cap ? RcHex(capb, 3) : "") << "\","
       << "\"error\":\"" << JsonEscape(err) << "\"}}";
    bridge->broadcastFrame(os.str());
}

// 写一帧 00 03 b0 b1 b2 并 flush。返回是否成功
bool RcWriteFrame(const unsigned char b[3])
{
    unsigned char frame[5] = {0x00, 0x03, b[0], b[1], b[2]};
    std::lock_guard<std::mutex> lk(g_rc_mutex);
    if (g_rc_fd < 0) return false;
    ssize_t n = ::write(g_rc_fd, frame, sizeof(frame));
    if (n != (ssize_t)sizeof(frame)) return false;
    ::tcdrain(g_rc_fd);  // 等待字节全部发出
    return true;
}

// 连发一帧 count 次(interval 200ms);entry_id>0 时累计发送次数并落盘
void RcSendBytes(WebBridge *bridge, const unsigned char b[3], int count,
                 const std::string &label, long entry_id)
{
    for (int i = 0; i < count; ++i) {
        if (!g_rc_run.load()) return;
        if (!RcWriteFrame(b)) {
            RcResult(bridge, false, "send", "串口不可用,发送失败(检查设备是否插好)");
            RcPublishInfo(bridge);
            return;
        }
        if (i + 1 < count) std::this_thread::sleep_for(std::chrono::milliseconds(kRcRepeatGapMs));
    }
    if (entry_id > 0) {
        std::lock_guard<std::mutex> lk(g_rc_mutex);
        g_rc_last_id = entry_id;
        for (auto &e : g_rc_entries) if (e.id == entry_id) { e.sends += count; break; }
        RcSaveFileLocked();
    }
    RcResult(bridge, true, "send", (label.empty() ? std::string("帧") : label) +
                                 " 已发送 " + std::to_string(count) + " 次");
    RcPublishList(bridge);
}

// 一帧结束后的处理(仅学习模式记录并推送)
void RcProcessFrame(WebBridge *bridge, const std::vector<unsigned char> &buf)
{
    int n = (int)buf.size();
    bool learning;
    {
        std::lock_guard<std::mutex> lk(g_rc_mutex);
        learning = g_rc_learning;
        if (learning && n == kRcFrameBytes) {
            g_rc_has_capture = true;
            std::memcpy(g_rc_capture, buf.data(), kRcFrameBytes);
        }
    }
    if (!learning) return;  // 非学习模式不推送
    std::ostringstream os;
    os << "{\"type\":\"rc_capture\",\"raw\":" << (n == kRcFrameBytes ? "false" : "true")
       << ",\"len\":" << n << ",\"hex\":\"" << RcHex(buf.data(), n) << "\"}";
    bridge->broadcastFrame(os.str());
}

// 串口线程:热插拔自愈 + poll 读帧 + 50ms 帧间隔
void RcSerialThread(WebBridge *bridge)
{
    std::vector<unsigned char> buf;
    auto last_open = std::chrono::steady_clock::now() - std::chrono::milliseconds(kRcReopenMs);
    while (g_rc_run.load()) {
        int fd;
        {
            std::lock_guard<std::mutex> lk(g_rc_mutex);
            if (g_rc_fd < 0) {  // 周期重试打开(热插拔后自动恢复)
                auto now = std::chrono::steady_clock::now();
                if (now - last_open >= std::chrono::milliseconds(kRcReopenMs)) {
                    last_open = now;
                    RcOpenLocked();
                    if (g_rc_fd >= 0)
                        std::cout << "[web_server] rc: 打开串口 " << g_rc_dev_path << std::endl;
                }
            }
            fd = g_rc_fd;
        }
        if (fd < 0) { std::this_thread::sleep_for(std::chrono::milliseconds(200)); continue; }

        struct pollfd pfd{};
        pfd.fd = fd;
        pfd.events = POLLIN;
        int pr = ::poll(&pfd, 1, kRcGapMs);

        if (pr > 0 && (pfd.revents & POLLIN)) {
            unsigned char tmp[64];
            ssize_t n = ::read(fd, tmp, sizeof(tmp));
            if (n > 0) {
                buf.insert(buf.end(), tmp, tmp + n);
                if (buf.size() > 256) { RcProcessFrame(bridge, buf); buf.clear(); }  // 异常长流兜底
            } else if (n <= 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
                std::lock_guard<std::mutex> lk(g_rc_mutex);
                if (g_rc_fd >= 0) ::close(g_rc_fd);
                g_rc_fd = -1;
                g_rc_dev_err = "读错误/设备断开: " + std::string(std::strerror(errno));
                buf.clear();
            }
            continue;  // 继续 poll,等待可能的后续字节
        }
        if (pr > 0 && (pfd.revents & (POLLERR | POLLHUP | POLLNVAL))) {
            std::lock_guard<std::mutex> lk(g_rc_mutex);
            if (g_rc_fd >= 0) ::close(g_rc_fd);
            g_rc_fd = -1;
            g_rc_dev_err = "设备断开(POLLERR/HUP)";
            buf.clear();
            continue;
        }
        if (pr == 0 && !buf.empty()) {  // 50ms 无新字节:一帧结束
            RcProcessFrame(bridge, buf);
            buf.clear();
        }
        if (!g_rc_run.load()) break;
    }
    std::lock_guard<std::mutex> lk(g_rc_mutex);
    if (g_rc_fd >= 0) { ::close(g_rc_fd); g_rc_fd = -1; }
}

// 从 "hh hh hh [次数]" 解析 raw 命令参数
bool RcParseRawArgs(const std::string &rest, unsigned char b[3], int &count)
{
    std::vector<std::string> toks;
    std::istringstream iss(rest);
    std::string t;
    while (iss >> t) toks.push_back(t);
    count = 1;
    if (toks.size() == 4) {
        bool numeric = !toks[3].empty();
        for (char c : toks[3]) if (!std::isdigit((unsigned char)c)) numeric = false;
        if (numeric && RcParseHex(toks[0] + " " + toks[1] + " " + toks[2], b)) {
            count = std::atoi(toks[3].c_str());
            return true;
        }
    }
    if (toks.size() == 3) return RcParseHex(toks[0] + " " + toks[1] + " " + toks[2], b);
    return RcParseHex(rest, b);
}

// 处理 "rc <op>" 命令;返回 true 表示已消费(不转发 ROS)
bool HandleRcCommand(WebBridge &bridge, const std::string &text)
{
    if (text.rfind("rc ", 0) != 0) return false;
    std::string op = Trim(text.substr(3));

    if (op == "info") { RcPublishInfo(&bridge); return true; }
    if (op == "list") { RcPublishList(&bridge); return true; }

    if (op == "learn on" || op == "learn off") {
        bool on = (op == "learn on");
        { std::lock_guard<std::mutex> lk(g_rc_mutex); g_rc_learning = on; }
        RcResult(&bridge, true, "learn", on ? "学习已开启,触发遥控器即可捕获" : "学习已停止");
        RcPublishInfo(&bridge);
        return true;
    }

    if (op == "save" || op.rfind("save ", 0) == 0) {
        std::string rest = Trim(op.substr(4));
        // 名称 = 第一个参数(支持引号包裹空格);其后可选 3 字节 hex(用于保存前端点选的某条捕获)
        std::string name, hexArg;
        if (!rest.empty() && (rest.front() == '"' || rest.front() == '\'')) {
            char q = rest.front();
            auto e = rest.find(q, 1);
            if (e == std::string::npos) name = rest.substr(1);
            else { name = rest.substr(1, e - 1); hexArg = Trim(rest.substr(e + 1)); }
        } else {
            auto sp = rest.find(' ');
            name = (sp == std::string::npos) ? rest : rest.substr(0, sp);
            hexArg = (sp == std::string::npos) ? "" : Trim(rest.substr(sp + 1));
        }
        unsigned char chosen[3] = {0, 0, 0};
        bool haveHex = false;
        bool ok = false;
        std::string err;
        if (name.empty()) {
            err = "缺少名称,用法: rc save <名称>";
        } else if (!hexArg.empty() && !(haveHex = RcParseHex(hexArg, chosen))) {
            err = "附加的 hex 参数无效: " + hexArg;
        } else {
            std::lock_guard<std::mutex> lk(g_rc_mutex);
            if (!haveHex) {  // 未显式给 hex:落盘最近一条待保存捕获
                if (!g_rc_has_capture) err = "没有待保存捕获,请先开启学习并触发遥控";
                else { std::memcpy(chosen, g_rc_capture, 3); haveHex = true; }
            }
            if (haveHex) {
                RcEntry e;
                e.id = g_rc_next_id++;
                e.name = name;
                std::memcpy(e.b, chosen, 3);
                e.sends = 0;
                e.time = RcNowString();
                g_rc_entries.push_back(e);
                if (RcSaveFileLocked()) { g_rc_has_capture = false; ok = true; }
                else { g_rc_entries.pop_back(); g_rc_next_id--; err = "写入 " + g_rc_codes_path + " 失败"; }
            }
        }
        RcResult(&bridge, ok, "save", ok ? ("已保存: " + name + " [" + RcHex(chosen, 3) + "]") : err);
        if (ok) { RcPublishList(&bridge); RcPublishInfo(&bridge); }
        return true;
    }

    if (op.rfind("del ", 0) == 0) {
        int id = std::atoi(Trim(op.substr(4)).c_str());
        bool ok = false;
        std::string err;
        {
            std::lock_guard<std::mutex> lk(g_rc_mutex);
            for (auto it = g_rc_entries.begin(); it != g_rc_entries.end(); ++it) {
                if (it->id == id) { g_rc_entries.erase(it); RcSaveFileLocked(); ok = true; break; }
            }
            if (!ok) err = "未找到 id=" + std::to_string(id);
        }
        RcResult(&bridge, ok, "del", ok ? ("已删除 id=" + std::to_string(id)) : err);
        RcPublishList(&bridge);
        return true;
    }

    if (op.rfind("rename ", 0) == 0) {
        std::string rest = Trim(op.substr(7));
        auto sp = rest.find(' ');
        int id = std::atoi(rest.substr(0, sp == std::string::npos ? rest.size() : sp).c_str());
        std::string name = (sp == std::string::npos) ? "" : Trim(rest.substr(sp + 1));
        if (name.size() >= 2 && ((name.front() == '"' && name.back() == '"') ||
                                 (name.front() == '\'' && name.back() == '\'')))
            name = name.substr(1, name.size() - 2);
        bool ok = false;
        std::string err;
        if (name.empty()) err = "缺少新名称,用法: rc rename <id> <名称>";
        else {
            std::lock_guard<std::mutex> lk(g_rc_mutex);
            for (auto &e : g_rc_entries) {
                if (e.id == id) {
                    e.name = name;
                    ok = RcSaveFileLocked();
                    if (!ok) err = "写盘失败";
                    break;
                }
            }
            if (!ok && err.empty()) err = "未找到 id=" + std::to_string(id);
        }
        RcResult(&bridge, ok, "rename", ok ? ("已重命名为 " + name) : err);
        RcPublishList(&bridge);
        return true;
    }

    if (op.rfind("hex ", 0) == 0) {
        // 修改已存在条目的 hex(不改 sends 计数语义)
        std::string rest = Trim(op.substr(4));
        auto sp = rest.find(' ');
        int id = std::atoi(rest.substr(0, sp == std::string::npos ? rest.size() : sp).c_str());
        std::string hexArg = (sp == std::string::npos) ? "" : Trim(rest.substr(sp + 1));
        unsigned char nb[3] = {0, 0, 0};
        bool ok = false;
        std::string err;
        if (hexArg.empty()) err = "缺少 hex,用法: rc hex <id> <hh hh hh>";
        else if (!RcParseHex(hexArg, nb)) err = "非法 hex(须恰好 3 字节,如 50 3C 7A): " + hexArg;
        else {
            std::lock_guard<std::mutex> lk(g_rc_mutex);
            for (auto &e : g_rc_entries) {
                if (e.id == id) {
                    unsigned char old[3];
                    std::memcpy(old, e.b, 3);
                    std::memcpy(e.b, nb, 3);
                    if (RcSaveFileLocked()) ok = true;
                    else { std::memcpy(e.b, old, 3); err = "写盘失败"; }
                    break;
                }
            }
            if (!ok && err.empty()) err = "未找到 id=" + std::to_string(id);
        }
        RcResult(&bridge, ok, "hex", ok ? ("id=" + std::to_string(id) + " 已更新为 " + RcHex(nb, 3)) : err);
        if (ok) RcPublishList(&bridge);
        return true;
    }

    if (op.rfind("send ", 0) == 0) {
        std::string rest = Trim(op.substr(5));
        auto sp = rest.find(' ');
        int id = std::atoi(rest.substr(0, sp == std::string::npos ? rest.size() : sp).c_str());
        int count = (sp == std::string::npos) ? 1 : std::atoi(Trim(rest.substr(sp + 1)).c_str());
        if (count < 1) count = 1;
        if (count > kRcMaxRepeat) count = kRcMaxRepeat;
        unsigned char b[3];
        std::string name;
        bool found = false;
        {
            std::lock_guard<std::mutex> lk(g_rc_mutex);
            for (const auto &e : g_rc_entries)
                if (e.id == id) { std::memcpy(b, e.b, 3); name = e.name; found = true; break; }
        }
        if (!found) { RcResult(&bridge, false, "send", "未找到 id=" + std::to_string(id)); return true; }
        std::thread([&bridge, id, count, name, c0 = b[0], c1 = b[1], c2 = b[2]] {
            unsigned char bb[3] = {c0, c1, c2};
            RcSendBytes(&bridge, bb, count, name, id);
        }).detach();
        return true;
    }

    if (op == "last" || op.rfind("last ", 0) == 0) {
        int count = (op == "last") ? 1 : std::atoi(Trim(op.substr(5)).c_str());
        if (count < 1) count = 1;
        if (count > kRcMaxRepeat) count = kRcMaxRepeat;
        unsigned char b[3] = {0, 0, 0};
        std::string name;
        long id = -1;
        {
            std::lock_guard<std::mutex> lk(g_rc_mutex);
            if (g_rc_last_id >= 0) {
                id = g_rc_last_id;
                for (const auto &e : g_rc_entries)
                    if (e.id == g_rc_last_id) { std::memcpy(b, e.b, 3); name = e.name; break; }
            }
        }
        if (id < 0) { RcResult(&bridge, false, "last", "尚无发送记录"); return true; }
        std::thread([&bridge, id, count, name, c0 = b[0], c1 = b[1], c2 = b[2]] {
            unsigned char bb[3] = {c0, c1, c2};
            RcSendBytes(&bridge, bb, count, name, id);
        }).detach();
        return true;
    }

    if (op.rfind("raw ", 0) == 0) {
        unsigned char b[3];
        int count = 1;
        if (!RcParseRawArgs(Trim(op.substr(4)), b, count)) {
            RcResult(&bridge, false, "raw", "需要 3 个字节,用法: rc raw 50 3C 7A [次数]");
            return true;
        }
        if (count < 1) count = 1;
        if (count > kRcMaxRepeat) count = kRcMaxRepeat;
        std::string label = RcHex(b, 3);
        std::thread([&bridge, count, label, c0 = b[0], c1 = b[1], c2 = b[2]] {
            unsigned char bb[3] = {c0, c1, c2};
            RcSendBytes(&bridge, bb, count, label, -1);
        }).detach();
        return true;
    }

    RcResult(&bridge, false, "unknown", "未知 rc 子命令: " + op);
    return true;
}
}  // namespace

int main(int argc, char **argv)
{
    std::signal(SIGINT, OnSignal);
    std::signal(SIGTERM, OnSignal);
    std::signal(SIGPIPE, SIG_IGN);

    std::cout << "[version] jxg-web v" << APP_VERSION
              << " build " << BUILD_NUMBER
              << " built " << BUILD_TIMESTAMP << std::endl;

    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("rl_web_bridge_node");

    // 与 rl_real 发布端一致: reliable + volatile, KeepLast(10)
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();

    // ---- 上行: 浏览器 -> ROS ----
    // 浏览器速度走「独占话题」/rl_real/web_cmd_vel:rl_real 据此区分来源,实现 Web 对
    // ROS 大脑(/rl_real/cmd_vel)的事件触发式速度独占。行为命令等仍走原话题。
    auto pub_cmd_vel  = node->create_publisher<geometry_msgs::msg::Twist>("/rl_real/web_cmd_vel", qos);
    auto pub_command  = node->create_publisher<std_msgs::msg::String>("/rl_real/web_command", qos);
    auto pub_imu      = node->create_publisher<std_msgs::msg::Bool>("/rl_real/imu_stream_enable", qos);
    auto pub_ack      = node->create_publisher<std_msgs::msg::String>("/rl_real/notify_ack", qos);
    auto pub_check    = node->create_publisher<std_msgs::msg::String>("/rl_real/check_stand", qos);
    // BMS 电池状态:1Hz 发布给 rl_real_JXG 做低电量判断
    // data=[soc%, voltage_V, current_A, alarm_bits, online(0/1)]
    auto pub_bms = node->create_publisher<std_msgs::msg::Float32MultiArray>("/rl_real/bms_state", qos);

    WebBridge bridge;

    // 订阅 shared_ptr 必须持有到进程结束,否则 create_subscription 返回值析构即退订
    std::vector<rclcpp::SubscriptionBase::SharedPtr> subs;

    // ---- 下行: ROS -> 浏览器 ----
    subs.push_back(node->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/rl_real/motor_state", qos,
        [&bridge](const std_msgs::msg::Float32MultiArray::SharedPtr m) {
            bridge.publishMotorState(ToVector(m));
        }));
    subs.push_back(node->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/rl_real/imu_state", qos,
        [&bridge](const std_msgs::msg::Float32MultiArray::SharedPtr m) {
            bridge.publishImuState(ToVector(m));
        }));
    subs.push_back(node->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/rl_real/pose2d", qos,
        [&bridge](const std_msgs::msg::Float32MultiArray::SharedPtr m) {
            bridge.publishPose2D(ToVector(m));
        }));
    subs.push_back(node->create_subscription<std_msgs::msg::String>(
        "/rl_real/feedback", qos,
        [&bridge](const std_msgs::msg::String::SharedPtr m) {
            bridge.publishFeedback(m->data);
        }));
    subs.push_back(node->create_subscription<std_msgs::msg::String>(
        "/rl_real/notify", qos,
        [&bridge](const std_msgs::msg::String::SharedPtr m) {
            bridge.publishNotify(m->data);
        }));
    subs.push_back(node->create_subscription<std_msgs::msg::String>(
        "/rl_real/check_stand_result", qos,
        [&bridge](const std_msgs::msg::String::SharedPtr m) {
            bridge.publishCheckStandResult(m->data);
        }));
    subs.push_back(node->create_subscription<std_msgs::msg::String>(
        "/rl_real/posture", qos,
        [&bridge](const std_msgs::msg::String::SharedPtr m) {
            bridge.publishPosture(m->data);
        }));
    subs.push_back(node->create_subscription<std_msgs::msg::String>(
        "/rl_real/check_standing_result", qos,
        [&bridge](const std_msgs::msg::String::SharedPtr m) {
            bridge.publishCheckStandingResult(m->data);
        }));
    // cmd_state / heartbeat 的 String 载荷本身就是完整 WS JSON 帧, 原样广播
    subs.push_back(node->create_subscription<std_msgs::msg::String>(
        "/rl_real/cmd_state", qos,
        [&bridge](const std_msgs::msg::String::SharedPtr m) {
            bridge.broadcastFrame(m->data);
        }));
    subs.push_back(node->create_subscription<std_msgs::msg::String>(
        "/rl_real/heartbeat", qos,
        [&bridge](const std_msgs::msg::String::SharedPtr m) {
            bridge.broadcastFrame(m->data);
        }));

    int port = 8088;
    if (const char *p = std::getenv("JXG_WEB_PORT")) {
        int v = std::atoi(p);
        if (v > 0 && v < 65536) port = v;
    }
    std::string web_root = WEB_ROOT;
    if (const char *r = std::getenv("WEB_ROOT")) {
        if (*r) web_root = r;
    }

    // 遥控学习:码文件放在项目根(config 目录),即 WEB_ROOT 静态页目录的上一级
    {
        std::string root = web_root;
        while (root.size() > 1 && root.back() == '/') root.pop_back();
        auto slash = root.rfind('/');
        std::string proj = (slash == std::string::npos) ? std::string(".") : root.substr(0, slash);
        g_rc_codes_path = proj + kRcCodesRel;
        std::lock_guard<std::mutex> lk(g_rc_mutex);
        RcLoadFileLocked();
    }

    if (!bridge.start(
            port, web_root,
            [&pub_cmd_vel](const WebBridge::CmdVel &v) {
                geometry_msgs::msg::Twist t;
                t.linear.x = v.x;
                t.linear.y = v.y;
                t.angular.z = v.yaw;
                pub_cmd_vel->publish(t);
            },
            [&pub_command, &bridge](const std::string &text, int /*seq*/) {
                // 服务管理命令在本进程执行(可安全 stop/restart rl_real),
                // 系统信息/进程管理也在本进程执行,
                // 遥控学习/克隆同样在本进程执行(独占串口),
                // 其余控制命令转发 ROS /rl_real/command。
                if (HandleServiceCommand(bridge, text)) return;
                if (HandleSysCommand(bridge, text)) return;
                if (HandleNetConfigCommand(bridge, text)) return;
                if (HandleRcCommand(bridge, text)) return;
                // 电池 BMS 蓝牙直读(独立线程,不阻塞控制环)
                if (bms_ble::HandleCommand(text, [&bridge](const std::string &f) {
                        bridge.broadcastFrame(f);
                    })) return;
                std_msgs::msg::String m;
                m.data = text;
                pub_command->publish(m);
            },
            [&pub_imu](bool flag) {
                std_msgs::msg::Bool m;
                m.data = flag;
                pub_imu->publish(m);
            },
            [&pub_ack](const std::string &text) {
                std_msgs::msg::String m;
                m.data = text;
                pub_ack->publish(m);
            },
            [&pub_check]() {
                std_msgs::msg::String m;  // rl_real 不解析载荷
                pub_check->publish(m);
            }))
    {
        std::cerr << "[web_server] FATAL: failed to bind port " << port
                  << " (another web server running?)" << std::endl;
        rclcpp::shutdown();
        return 1;
    }

    std::cout << "[web_server] standalone web console on 0.0.0.0:" << port
              << " root=" << web_root << std::endl;
    std::cout << "[web_server] bridging ROS2 <-> WebSocket; rl_real_JXG may restart freely"
              << std::endl;

    // 遥控学习串口线程(独占设备,与 200Hz 控制环无关;设备缺失时降级不崩溃)
    g_rc_run.store(true);
    g_rc_thread = std::thread([&bridge] { RcSerialThread(&bridge); });

    // 电池 BMS 蓝牙线程(独占 BlueZ D-Bus;默认关闭,由前端「电池」页签 on/off)
    bms_ble::Start([&bridge](const std::string &f) { bridge.broadcastFrame(f); });

    // ROS 回调在独立 executor 线程; WebBridge 内部自带网络/分发线程。
    // 跨线程调用 publisher/subscription 与 bridge 广播均为线程安全。
    std::thread spin_thread([&node] {
        rclcpp::executors::SingleThreadedExecutor exec;
        exec.add_node(node);
        while (rclcpp::ok() && !g_shutdown.load()) exec.spin_some(std::chrono::milliseconds(50));
    });

    // DDS 看门狗(自愈):实测网络接口 flap(netplan apply/插拔网线)或对端重启后,
    // FastDDS(UDP-only)可能不重新建立发现关系——表现为本进程 HTTP/WS 仍正常,
    // 但本节点已从 DDS 发现图消失、订阅收不到任何数据且不自愈。
    // 判据:rl_real_JXG 进程确实在运行,但本节点 DDS 图里连续 kMissLimit 秒看不到
    // "/rl_real_node" → 判定 DDS 桥僵死 → 直接退出(非0码),由 systemd Restart=always
    // 拉起干净进程;浏览器 WS 侧自动重连。rl_real 停止/重启窗口(进程不存在)不检测,
    // 避免误杀。启动后先给 30s 宽限,等待自身首轮发现完成。
    std::thread wd_thread([&node] {
        constexpr int kGraceSec = 30;
        constexpr int kMissLimit = 20;
        std::this_thread::sleep_for(std::chrono::seconds(kGraceSec));
        int miss = 0;
        while (!g_shutdown.load()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            if (!IsRlRealProcessAlive()) { miss = 0; continue; }  // 主程序停止/重启中
            bool seen = false;
            try {
                for (const auto &n : node->get_node_names()) {
                    if (n.find("rl_real_node") != std::string::npos) { seen = true; break; }
                }
            } catch (...) {
                seen = false;
            }
            if (seen) {
                miss = 0;
            } else if (++miss >= kMissLimit) {
                std::cerr << "[web_server] FATAL watchdog: rl_real_JXG 在运行但本进程 DDS 连续 "
                          << kMissLimit << "s 发现不到它(网络变化/对端重启后未自愈),退出码 42 "
                             "以触发 systemd 自动重启" << std::endl;
                std::fflush(nullptr);
                std::_Exit(42);
            }
        }
    });

    // BMS 状态 1Hz 发布给 rl_real_JXG(主循环 200ms tick,用计数器分频到 1s)
    int bms_tick = 0;
    while (!g_shutdown.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        if (++bms_tick >= 5) {  // 5 × 200ms = 1s
            bms_tick = 0;
            bms_ble::Snapshot s = bms_ble::GetSnapshot();
            std_msgs::msg::Float32MultiArray bms_msg;
            bms_msg.data = {
                static_cast<float>(s.soc),
                static_cast<float>(s.voltage),
                static_cast<float>(s.current),
                static_cast<float>(s.alarm),
                s.online ? 1.0f : 0.0f
            };
            pub_bms->publish(bms_msg);
        }
    }

    std::cout << "[web_server] shutting down ..." << std::endl;
    bms_ble::Stop();                                      // 先停 BMS 蓝牙线程(其仍会调用 bridge)
    g_rc_run.store(false);                                // 再停串口线程(其仍会调用 bridge)
    if (g_rc_thread.joinable()) g_rc_thread.join();
    bridge.stop();
    rclcpp::shutdown();
    if (spin_thread.joinable()) spin_thread.join();
    if (wd_thread.joinable()) wd_thread.join();
    return 0;
}
