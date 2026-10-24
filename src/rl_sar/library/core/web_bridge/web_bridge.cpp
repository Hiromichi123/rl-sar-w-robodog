#include "web_bridge.hpp"

// 版本宏兜底(由 CMake target_compile_definitions 注入; 独立编译时给默认值)
#ifndef APP_VERSION
#define APP_VERSION "dev"
#endif
#ifndef BUILD_NUMBER
#define BUILD_NUMBER "-"
#endif
#ifndef BUILD_TIMESTAMP
#define BUILD_TIMESTAMP "-"
#endif

#include <arpa/inet.h>
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <grp.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <pty.h>
#include <pwd.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/sendfile.h>
#include <sys/wait.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <thread>

// POLICY_DIR 由 CMake 编译期注入(指向 <repo>/policy);此处兜底防裸 g++ 编译失败
#ifndef POLICY_DIR
#define POLICY_DIR "./policy"
#endif

// ============================================================================
// 最小 JSON 工具（与 rl_real_JXG.cpp 中 JsonEscape 同风格，无第三方库）
// ============================================================================
static std::string JsonEscape(const std::string &s)
{
    static const char kHex[] = "0123456789abcdef";
    std::string out;
    out.reserve(s.size() + 8);
    for (unsigned char c : s) {
        switch (c) {
            case '\\': out += "\\\\"; break;
            case '"':  out += "\\\""; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            case '\b': out += "\\b"; break;
            case '\f': out += "\\f"; break;
            default:
                // JSON 字符串中 0x00-0x1F 禁止裸出现,必须转义;
                // PTY 输出里的 ESC(0x1b 颜色/光标/OSC 序列)转成 \u001b 后,
                // 浏览器 JSON.parse 会还原成 ESC 字节再交给 xterm.js 解析。
                // 裸传会导致整帧成为非法 JSON 而被丢弃(输出/提示符随之消失)。
                if (c < 0x20) {
                    out += "\\u00";
                    out += kHex[(c >> 4) & 0xF];
                    out += kHex[c & 0xF];
                } else {
                    out += static_cast<char>(c);
                }
                break;
        }
    }
    return out;
}

// URL 解码(%XX / +)
static std::string UrlDecode(const std::string &s)
{
    std::string out;
    out.reserve(s.size());
    for (size_t i = 0; i < s.size(); ++i) {
        if (s[i] == '+') { out += ' '; continue; }
        if (s[i] == '%' && i + 2 < s.size()) {
            auto hexv = [](char ch) -> int {
                if (ch >= '0' && ch <= '9') return ch - '0';
                if (ch >= 'a' && ch <= 'f') return ch - 'a' + 10;
                if (ch >= 'A' && ch <= 'F') return ch - 'A' + 10;
                return -1;
            };
            int hi = hexv(s[i + 1]), lo = hexv(s[i + 2]);
            if (hi >= 0 && lo >= 0) { out += char((hi << 4) | lo); i += 2; continue; }
        }
        out += s[i];
    }
    return out;
}

// URL 编码(用于 Set-Cookie 值;字母数字及 -_.~ 保留,其余 %XX)
static std::string UrlEncode(const std::string &s)
{
    static const char *hex = "0123456789ABCDEF";
    std::string out;
    out.reserve(s.size() * 3);
    for (unsigned char ch : s) {
        if ((ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z') ||
            (ch >= '0' && ch <= '9') || ch == '-' || ch == '_' || ch == '.' || ch == '~') {
            out += char(ch);
        } else {
            out += '%'; out += hex[ch >> 4]; out += hex[ch & 0xF];
        }
    }
    return out;
}

// 从 Cookie 头中取指定字段(值按 URL 解码)
static std::string CookieGet(const std::map<std::string, std::string> &hdr,
                             const std::string &name)
{
    auto it = hdr.find("cookie");
    if (it == hdr.end()) return "";
    auto trim = [](const std::string &s) -> std::string {
        size_t a = s.find_first_not_of(" \t");
        if (a == std::string::npos) return "";
        size_t b = s.find_last_not_of(" \t");
        return s.substr(a, b - a + 1);
    };
    std::istringstream ss(it->second);
    std::string part;
    while (std::getline(ss, part, ';')) {
        size_t eq = part.find('=');
        if (eq == std::string::npos) continue;
        if (trim(part.substr(0, eq)) == name)
            return UrlDecode(trim(part.substr(eq + 1)));
    }
    return "";
}

// 从 JSON 文本中提取 "key":"value" 的字符串 value（处理 \\" \\\\ 最小转义）
// 把一个 Unicode 码点按 UTF-8 追加到 out(终端按键多为 U+0000~U+009F,
// 如方向键/控制键经 JSON.stringify 后是 \u001b / \u0003 等转义)
static void AppendUtf8(unsigned cp, std::string &out)
{
    if (cp <= 0x7F) {
        out += static_cast<char>(cp);
    } else if (cp <= 0x7FF) {
        out += static_cast<char>(0xC0 | (cp >> 6));
        out += static_cast<char>(0x80 | (cp & 0x3F));
    } else {
        out += static_cast<char>(0xE0 | (cp >> 12));
        out += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
        out += static_cast<char>(0x80 | (cp & 0x3F));
    }
}

// 解析 4 位十六进制;失败返回 false
static bool Hex4(const std::string &s, size_t pos, unsigned &cp)
{
    cp = 0;
    for (int k = 0; k < 4; ++k) {
        char hc = s[pos + k];
        unsigned hv;
        if (hc >= '0' && hc <= '9') hv = unsigned(hc - '0');
        else if (hc >= 'a' && hc <= 'f') hv = unsigned(hc - 'a' + 10);
        else if (hc >= 'A' && hc <= 'F') hv = unsigned(hc - 'A' + 10);
        else return false;
        cp = (cp << 4) | hv;
    }
    return true;
}

static bool JsonGetString(const std::string &json, const std::string &key, std::string &out)
{
    std::string pat = "\"" + key + "\"";
    size_t p = json.find(pat);
    if (p == std::string::npos) return false;
    p = json.find(':', p + pat.size());
    if (p == std::string::npos) return false;
    ++p;
    while (p < json.size() && (json[p] == ' ' || json[p] == '\t')) ++p;
    if (p >= json.size() || json[p] != '"') return false;
    ++p;
    out.clear();
    while (p < json.size()) {
        char c = json[p];
        if (c == '\\' && p + 1 < json.size()) {
            char n = json[p + 1];
            if (n == '"' || n == '\\') { out += n; p += 2; continue; }
            if (n == 'n') { out += '\n'; p += 2; continue; }
            if (n == 't') { out += '\t'; p += 2; continue; }
            if (n == 'r') { out += '\r'; p += 2; continue; }
            if (n == 'b') { out += '\b'; p += 2; continue; }
            if (n == 'f') { out += '\f'; p += 2; continue; }
            if (n == '/' ) { out += '/';  p += 2; continue; }
            if (n == 'u' && p + 5 < json.size()) {
                unsigned cp = 0;
                if (!Hex4(json, p + 2, cp)) { out += '?'; p += 6; continue; }
                // UTF-16 代理对: \uD800-\uDBFF 后须紧跟 \uDC00-\uDFFF
                if ((cp & 0xFC00) == 0xD800 && p + 11 < json.size() &&
                    json[p + 6] == '\\' && json[p + 7] == 'u') {
                    unsigned lo = 0;
                    if (Hex4(json, p + 8, lo) && (lo & 0xFC00) == 0xDC00) {
                        cp = 0x10000 + ((cp - 0xD800) << 10) + (lo - 0xDC00);
                        p += 12;
                        // 增补平面 → 4 字节 UTF-8
                        out += static_cast<char>(0xF0 | (cp >> 18));
                        out += static_cast<char>(0x80 | ((cp >> 12) & 0x3F));
                        out += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
                        out += static_cast<char>(0x80 | (cp & 0x3F));
                        continue;
                    }
                }
                AppendUtf8(cp, out);
                p += 6;
                continue;
            }
            out += n; p += 2; continue;
        }
        if (c == '"') return true;
        out += c;
        ++p;
    }
    return false;
}

static bool JsonGetNumber(const std::string &json, const std::string &key, double &out)
{
    std::string pat = "\"" + key + "\"";
    size_t p = json.find(pat);
    if (p == std::string::npos) return false;
    p = json.find(':', p + pat.size());
    if (p == std::string::npos) return false;
    ++p;
    while (p < json.size() && (json[p] == ' ' || json[p] == '\t')) ++p;
    if (p >= json.size()) return false;
    out = std::strtod(json.c_str() + p, nullptr);
    return true;
}

static bool JsonGetBool(const std::string &json, const std::string &key, bool &out)
{
    std::string pat = "\"" + key + "\"";
    size_t p = json.find(pat);
    if (p == std::string::npos) return false;
    p = json.find(':', p + pat.size());
    if (p == std::string::npos) return false;
    size_t t = json.find("true", p);
    size_t f = json.find("false", p);
    if (t == std::string::npos && f == std::string::npos) return false;
    out = (t != std::string::npos && (f == std::string::npos || t < f));
    return true;
}

// ============================================================================
// SHA1（RFC 3174 紧凑实现）+ Base64，仅用于 WebSocket 握手
// ============================================================================
namespace {

struct Sha1
{
    uint32_t h[5] = {0x67452301, 0xEFCDAB89, 0x98BADCFE, 0x10325476, 0xC3D2E1F0};
    uint8_t buf[64];
    size_t buf_len = 0;
    uint64_t len = 0;

    void update(const uint8_t *p, size_t n)
    {
        len += n;
        while (n > 0) {
            size_t take = std::min(n, size_t(64) - buf_len);
            std::memcpy(buf + buf_len, p, take);
            buf_len += take; p += take; n -= take;
            if (buf_len == 64) { block(buf); buf_len = 0; }
        }
    }

    void block(const uint8_t *b)
    {
        uint32_t w[80];
        for (int i = 0; i < 16; ++i)
            w[i] = (uint32_t(b[i*4]) << 24) | (uint32_t(b[i*4+1]) << 16) |
                   (uint32_t(b[i*4+2]) << 8) | uint32_t(b[i*4+3]);
        for (int i = 16; i < 80; ++i) {
            uint32_t x = w[i-3] ^ w[i-8] ^ w[i-14] ^ w[i-16];
            w[i] = (x << 1) | (x >> 31);
        }
        uint32_t a=h[0], bb=h[1], c=h[2], d=h[3], e=h[4];
        for (int i = 0; i < 80; ++i) {
            uint32_t f, k;
            if (i < 20)      { f = (bb & c) | ((~bb) & d); k = 0x5A827999; }
            else if (i < 40) { f = bb ^ c ^ d;             k = 0x6ED9EBA1; }
            else if (i < 60) { f = (bb & c) | (bb & d) | (c & d); k = 0x8F1BBCDC; }
            else             { f = bb ^ c ^ d;             k = 0xCA62C1D6; }
            uint32_t tmp = ((a << 5) | (a >> 27)) + f + e + k + w[i];
            e = d; d = c; c = (bb << 30) | (bb >> 2); bb = a; a = tmp;
        }
        h[0]+=a; h[1]+=bb; h[2]+=c; h[3]+=d; h[4]+=e;
    }

    std::string final_raw()
    {
        uint64_t bits = len * 8;
        uint8_t pad = 0x80;
        update(&pad, 1);
        uint8_t z = 0;
        while (buf_len != 56) update(&z, 1);
        uint8_t b[8];
        for (int i = 0; i < 8; ++i) b[i] = uint8_t(bits >> (56 - 8 * i));
        update(b, 8);
        std::string out(20, '\0');
        for (int i = 0; i < 5; ++i) {
            out[i * 4 + 0] = char((h[i] >> 24) & 0xFF);
            out[i * 4 + 1] = char((h[i] >> 16) & 0xFF);
            out[i * 4 + 2] = char((h[i] >> 8) & 0xFF);
            out[i * 4 + 3] = char(h[i] & 0xFF);
        }
        return out;
    }
};

std::string Sha1Raw(const std::string &s)
{
    Sha1 sha;
    sha.update(reinterpret_cast<const uint8_t *>(s.data()), s.size());
    return sha.final_raw();
}

std::string Base64Encode(const std::string &in)
{
    static const char *tbl = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string out;
    out.reserve((in.size() + 2) / 3 * 4);
    size_t i = 0;
    while (i + 3 <= in.size()) {
        uint32_t v = (uint8_t(in[i]) << 16) | (uint8_t(in[i + 1]) << 8) | uint8_t(in[i + 2]);
        out += tbl[(v >> 18) & 63]; out += tbl[(v >> 12) & 63];
        out += tbl[(v >> 6) & 63];  out += tbl[v & 63];
        i += 3;
    }
    if (in.size() - i == 1) {
        uint32_t v = uint8_t(in[i]) << 16;
        out += tbl[(v >> 18) & 63]; out += tbl[(v >> 12) & 63];
        out += "==";
    } else if (in.size() - i == 2) {
        uint32_t v = (uint8_t(in[i]) << 16) | (uint8_t(in[i + 1]) << 8);
        out += tbl[(v >> 18) & 63]; out += tbl[(v >> 12) & 63];
        out += tbl[(v >> 6) & 63];
        out += "=";
    }
    return out;
}

std::string WsAcceptKey(const std::string &client_key)
{
    return Base64Encode(Sha1Raw(client_key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"));
}

// ============================================================================
// WebSocket 帧编解码（文本帧为主）
// ============================================================================
constexpr uint64_t kMaxFramePayload = 1u << 20;      // 1MB
constexpr size_t kMaxOutBuf = 512u * 1024;          // 慢客户端写缓冲上限(终端输出较大,放宽)

std::string WsEncodeText(const std::string &payload)
{
    std::string out;
    out.reserve(payload.size() + 10);
    out += char(0x81);  // FIN + text
    size_t n = payload.size();
    if (n < 126) {
        out += char(n);
    } else if (n <= 0xFFFF) {
        out += char(126);
        out += char((n >> 8) & 0xFF); out += char(n & 0xFF);
    } else {
        out += char(127);
        for (int i = 7; i >= 0; --i) out += char((uint64_t(n) >> (8 * i)) & 0xFF);
    }
    out += payload;
    return out;
}

// ============================================================================
// MIME / 静态文件
// ============================================================================
const char *MimeType(const std::string &path)
{
    static const std::map<std::string, const char *> mime = {
        {".html", "text/html; charset=utf-8"}, {".htm", "text/html; charset=utf-8"},
        {".js", "text/javascript; charset=utf-8"}, {".mjs", "text/javascript; charset=utf-8"},
        {".css", "text/css; charset=utf-8"}, {".json", "application/json"},
        {".png", "image/png"}, {".jpg", "image/jpeg"}, {".jpeg", "image/jpeg"},
        {".gif", "image/gif"}, {".svg", "image/svg+xml"}, {".webp", "image/webp"},
        {".ico", "image/x-icon"}, {".gltf", "model/gltf+json"}, {".glb", "model/gltf-binary"},
        {".bin", "application/octet-stream"}, {".wav", "audio/wav"}, {".mp3", "audio/mpeg"},
        {".woff", "font/woff"}, {".woff2", "font/woff2"}, {".ttf", "font/ttf"},
        {".map", "application/json"}, {".txt", "text/plain; charset=utf-8"},
        {".log", "text/plain; charset=utf-8"}, {".sh", "text/plain; charset=utf-8"},
    };
    size_t dot = path.rfind('.');
    if (dot == std::string::npos) return "application/octet-stream";
    auto it = mime.find(path.substr(dot));
    return it != mime.end() ? it->second : "application/octet-stream";
}

// 解析后的入站消息
struct Inbound
{
    enum Kind {
        CmdVelMsg, CommandMsg, ImuStreamMsg, NotifyAckMsg, CheckStandMsg,
        LogSubMsg, LogUnsubMsg, LogHistoryMsg,
        SshStartMsg, SshDataMsg, SshResizeMsg, SshStopMsg
    } kind = CommandMsg;
    double vx = 0, vy = 0, yaw = 0;   // CmdVel / SshStart(cols,rows) / LogHistory(before,n)
    std::string text;
    int seq = 0;
};

}  // namespace

// ============================================================================
// WebBridge::Impl
// ============================================================================
struct WebBridge::Impl
{
    // ---- 配置 ----
    int port = 8088;
    std::string web_root;
    CmdVelCb on_cmd_vel;
    CommandCb on_command;
    BoolCb on_imu_stream;
    StringCb on_notify_ack;
    VoidCb on_check_stand;

    // 系统管理鉴权令牌(环境变量 WEB_ADMIN_TOKEN;空=放行,局域网信任模型)
    std::string admin_token;
    std::string log_unit = "rl-real-jxg.service";

    // ---- socket ----
    int listen_fd = -1;
    std::atomic<bool> running{false};
    std::thread net_thread;
    std::vector<struct pollfd> pfds;

    // ---- WS 客户端状态 ----
    struct Client
    {
        int fd = -1;
        bool ws = false;              // 已完成 WS 握手
        bool admin_ok = false;        // 本次连接是否通过系统管理鉴权
        std::string in_buf;           // 未解析的原始输入
        std::string out_buf;          // 待发送
        // HTTP 请求缓冲
        std::string http_buf;
        bool http_done = false;
        bool close_after_flush = false;
        // 静态文件流式发送(sendfile)
        int file_fd = -1;
        off_t file_off = 0;
        off_t file_size = 0;
        // WS 分片消息累积
        std::string frag_buf;
        bool in_frag = false;
        // 心跳
        double last_recv = 0.0;
        double last_ping = 0.0;
        bool ping_outstanding = false;
        // 实时日志订阅
        bool log_on = false;
        // WebSSH PTY
        int pty_fd = -1;
        pid_t pty_pid = -1;
        // WebFTP 上传:请求体流式落盘
        int up_fd = -1;
        std::string up_tmp;
        std::string up_dst;
        size_t up_left = 0;
    };
    std::map<int, Client> clients;

    // ---- 分发线程 ----
    std::thread dispatch_thread;
    std::mutex q_mutex;
    std::condition_variable q_cv;
    std::deque<Inbound> in_queue;
    static constexpr size_t kQueueCap = 32;

    // ---- 广播锁 ----
    std::mutex bcast_mutex;

    // ---- 实时日志: 全客户端共享一个 journalctl -f 进程 ----
    std::mutex log_mtx;
    int log_pipe_fd = -1;
    pid_t log_pid = -1;
    std::thread log_thread;
    int log_refs = 0;

    double NowSec() const
    {
        return std::chrono::duration<double>(
                   std::chrono::steady_clock::now().time_since_epoch()).count();
    }

    bool AdminTokenOk(const std::string &token) const
    {
        return admin_token.empty() || token == admin_token;
    }

    // 统一提取请求中的管理令牌(优先级: query token → X-Admin-Token 头 → Cookie)
    std::string RequestToken(const std::map<std::string, std::string> &q,
                             const std::map<std::string, std::string> &hdr) const
    {
        auto it = q.find("token");
        if (it != q.end() && !it->second.empty()) return it->second;
        auto h2 = hdr.find("x-admin-token");
        if (h2 != hdr.end() && !h2->second.empty()) return h2->second;
        return CookieGet(hdr, "jxg_admin");
    }

    // ------------------------------------------------------------------
    bool StartListen()
    {
        listen_fd = ::socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK, 0);
        if (listen_fd < 0) return false;
        int one = 1;
        ::setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_ANY);
        addr.sin_port = htons(uint16_t(port));
        if (::bind(listen_fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0 ||
            ::listen(listen_fd, 8) < 0)
        {
            ::close(listen_fd);
            listen_fd = -1;
            return false;
        }
        return true;
    }

    void SetNonBlock(int fd)
    {
        int fl = ::fcntl(fd, F_GETFL, 0);
        ::fcntl(fd, F_SETFL, fl | O_NONBLOCK);
        int one = 1;
        ::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    }

    void PtyClose(Client &c)
    {
        if (c.pty_pid > 0) {
            ::kill(c.pty_pid, SIGHUP);
            int st = 0;
            ::waitpid(c.pty_pid, &st, WNOHANG);  // 通常 pty 关闭即退出;未退出由内核回收
            c.pty_pid = -1;
        }
        if (c.pty_fd >= 0) { ::close(c.pty_fd); c.pty_fd = -1; }
    }

    void LogLeaveLocked(Client &c)
    {
        if (!c.log_on) return;
        c.log_on = false;
        StopLogStream();
    }

    void CloseClient(int fd)
    {
        auto it = clients.find(fd);
        if (it == clients.end()) return;
        PtyClose(it->second);
        if (it->second.log_on) { it->second.log_on = false; StopLogStream(); }
        if (it->second.up_fd >= 0) { ::close(it->second.up_fd); if (!it->second.up_tmp.empty()) ::unlink(it->second.up_tmp.c_str()); }
        if (it->second.file_fd >= 0) ::close(it->second.file_fd);
        ::close(fd);
        clients.erase(it);
    }

    void SendTo(Client &c, const std::string &data)
    {
        if (c.out_buf.size() + data.size() > kMaxOutBuf) {
            std::cout << "[web_bridge] client " << c.fd << " too slow, dropping" << std::endl;
            ::close(c.fd);
            c.fd = -1;
            return;
        }
        c.out_buf += data;
    }

    // 广播给所有完成握手的 WS 客户端
    void Broadcast(const std::string &payload)
    {
        std::string frame = WsEncodeText(payload);
        std::lock_guard<std::mutex> lock(bcast_mutex);
        for (auto &kv : clients) {
            Client &c = kv.second;
            if (!c.ws) continue;
            if (c.out_buf.size() + frame.size() > kMaxOutBuf) {
                std::cout << "[web_bridge] client " << c.fd << " too slow, dropping" << std::endl;
                ::close(c.fd);
                c.fd = -1;
                continue;
            }
            c.out_buf += frame;
        }
    }

    // 仅投递给日志订阅者(调用方无需持锁)
    void BroadcastLog(const std::string &payload)
    {
        std::string frame = WsEncodeText(payload);
        std::lock_guard<std::mutex> lock(bcast_mutex);
        for (auto &kv : clients) {
            Client &c = kv.second;
            if (!c.ws || !c.log_on) continue;
            if (c.out_buf.size() + frame.size() > kMaxOutBuf) { c.log_on = false; continue; }
            c.out_buf += frame;
        }
    }

    void SendToFd(int fd, const std::string &payload)
    {
        std::lock_guard<std::mutex> lock(bcast_mutex);
        auto it = clients.find(fd);
        if (it != clients.end()) SendTo(it->second, WsEncodeText(payload));
    }

    void PublishString(const std::string &type, const std::string &text)
    {
        Broadcast("{\"type\":\"" + type + "\",\"data\":\"" + JsonEscape(text) + "\"}");
    }

    void PublishFloatArray(const std::string &type, const std::vector<float> &data)
    {
        std::string out = "{\"type\":\"" + type + "\",\"data\":[";
        char buf[32];
        for (size_t i = 0; i < data.size(); ++i) {
            if (i) out += ',';
            std::snprintf(buf, sizeof(buf), "%.6g", double(data[i]));
            out += buf;
        }
        out += "]}";
        Broadcast(out);
    }

    // cmd_state / heartbeat 帧由 rl_real 统一构造(ROS 话题与 WS 同帧),经 Broadcast 转发

    // ------------------------------------------------------------------
    // 实时日志(journalctl -f)
    // ------------------------------------------------------------------
    // 解析一行 short-unix 输出: "<epoch秒.微秒> <主机名> <标识[pid]>: 正文"
    // 返回 true 时 out_time=epoch 秒(double), out_data=去掉时间戳与主机名后的部分
    static bool ParseShortUnixLine(const char *line, ssize_t n, double &out_time, std::string &out_data)
    {
        const char *sp1 = static_cast<const char *>(::memchr(line, ' ', n));
        if (!sp1) return false;
        char *end = nullptr;
        double ts = ::strtod(line, &end);
        if (end != sp1 || ts <= 0) return false;
        const char *p = sp1 + 1;
        const char *line_end = line + n;
        const char *sp2 = static_cast<const char *>(::memchr(p, ' ', line_end - p));
        if (!sp2) return false;   // 无正文段(不应发生)
        out_time = ts;
        out_data.assign(sp2 + 1, line_end);   // 保留 "标识[pid]: 正文"
        return true;
    }

    static std::string MakeLogFrame(const char *type, double ts, const std::string &data)
    {
        char tbuf[40];
        std::snprintf(tbuf, sizeof(tbuf), "%.6f", ts);
        return std::string("{\"type\":\"") + type + "\",\"time\":" + tbuf +
               ",\"data\":\"" + JsonEscape(data) + "\"}";
    }

    void LogReader()
    {
        FILE *fp = ::fdopen(log_pipe_fd, "r");
        if (!fp) { ::close(log_pipe_fd); log_pipe_fd = -1; return; }
        char *line = nullptr;
        size_t cap = 0;
        ssize_t n;
        while (running.load() && (n = ::getline(&line, &cap, fp)) != -1) {
            while (n > 0 && (line[n-1] == '\n' || line[n-1] == '\r')) line[--n] = '\0';
            if (n == 0) continue;
            double ts = 0;
            std::string data;
            if (ParseShortUnixLine(line, n, ts, data))
                BroadcastLog(MakeLogFrame("log", ts, data));
            else
                BroadcastLog(MakeLogFrame("log", 0, line));   // 非标准行原样透传(time=0)
        }
        ::free(line);
        ::fclose(fp);   // 同时关闭 log_pipe_fd
        {
            std::lock_guard<std::mutex> lock(bcast_mutex);
            for (auto &kv : clients) kv.second.log_on = false;
        }
        int st = 0;
        if (log_pid > 0) ::waitpid(log_pid, &st, 0);
        std::lock_guard<std::mutex> lk(log_mtx);
        log_pipe_fd = -1; log_pid = -1; log_refs = 0;
        Broadcast("{\"type\":\"log_status\",\"data\":\"日志流已结束\"}");
    }

    // 首次订阅启动 journalctl -u <unit> -n 0 -f(纯实时,不回放历史;历史走 SendLogHistory)
    // 多客户端共享同一进程
    void StartLogStream()
    {
        std::lock_guard<std::mutex> lk(log_mtx);
        ++log_refs;
        if (log_pipe_fd >= 0) return;
        // 上一个日志进程已退出:先回收线程(其尾部在本锁内复位 fd),再启动新的
        if (log_thread.joinable()) log_thread.join();

        int pfd[2];
        if (::pipe2(pfd, O_CLOEXEC) != 0) { --log_refs; return; }
        pid_t pid = ::fork();
        if (pid < 0) { ::close(pfd[0]); ::close(pfd[1]); --log_refs; return; }
        if (pid == 0) {
            // 子进程: stdout/stderr → 管道
            ::dup2(pfd[1], STDOUT_FILENO);
            ::dup2(pfd[1], STDERR_FILENO);
            ::close(pfd[0]); ::close(pfd[1]);
            ::setsid();
            ::execlp("journalctl", "journalctl", "-u", log_unit.c_str(),
                     "--no-pager", "-o", "short-unix", "-n", "0",
                     "-f", (char *)nullptr);
            ::_exit(127);
        }
        ::close(pfd[1]);
        log_pipe_fd = pfd[0];
        log_pid = pid;
        log_thread = std::thread([this] { LogReader(); });
        std::cout << "[web_bridge] log stream started: journalctl -u " << log_unit << " -n 0 -f" << std::endl;
    }

    void StopLogStream()
    {
        std::lock_guard<std::mutex> lk(log_mtx);
        --log_refs;
        if (log_refs > 0 || log_pid <= 0) return;
        ::kill(log_pid, SIGTERM);   // LogReader 读到 EOF 后自行 waitpid;stop() 负责 join
    }

    // 按需拉取历史日志: journalctl -u <unit> -n <n> --until @<before>(一次性,不跟随)
    // 结果仅回发给请求客户端: {"type":"log_history","lines":[{time,data},...]}
    void SendLogHistory(Client &c, double before, int n)
    {
        if (before <= 0) return;
        if (n <= 0 || n > 1000) n = 200;
        int pfd[2];
        if (::pipe2(pfd, O_CLOEXEC) != 0) return;
        pid_t pid = ::fork();
        if (pid < 0) { ::close(pfd[0]); ::close(pfd[1]); return; }
        if (pid == 0) {
            ::dup2(pfd[1], STDOUT_FILENO);
            ::dup2(pfd[1], STDERR_FILENO);
            ::close(pfd[0]); ::close(pfd[1]);
            ::setsid();
            char until_arg[48];
            std::snprintf(until_arg, sizeof(until_arg), "@%lld", (long long)before);
            std::string n_arg = std::to_string(n);
            ::execlp("journalctl", "journalctl", "-u", log_unit.c_str(),
                     "--no-pager", "-o", "short-unix",
                     "-n", n_arg.c_str(), "--until", until_arg, (char *)nullptr);
            ::_exit(127);
        }
        ::close(pfd[1]);
        int rfd = pfd[0];
        int client_fd = c.fd;
        // 一次性读取:独立短线程,读完组帧回发后 waitpid 回收
        std::thread([this, rfd, pid, before, client_fd] {
            std::string lines_json;
            size_t count = 0;
            FILE *fp = ::fdopen(rfd, "r");
            if (fp) {
                char *line = nullptr;
                size_t cap = 0;
                ssize_t m;
                while ((m = ::getline(&line, &cap, fp)) != -1) {
                    while (m > 0 && (line[m-1] == '\n' || line[m-1] == '\r')) line[--m] = '\0';
                    if (m == 0) continue;
                    double ts = 0;
                    std::string data;
                    if (!ParseShortUnixLine(line, m, ts, data)) continue;   // 跳过 "-- Logs begin" 等头
                    if (ts >= before) continue;   // 与已有首行去重(只看更早的)
                    if (count++) lines_json += ',';
                    char tbuf[40];
                    std::snprintf(tbuf, sizeof(tbuf), "%.6f", ts);
                    lines_json += "{\"time\":";
                    lines_json += tbuf;
                    lines_json += ",\"data\":\"";
                    lines_json += JsonEscape(data);
                    lines_json += "\"}";
                }
                ::free(line);
                ::fclose(fp);
            } else {
                ::close(rfd);
            }
            int st = 0;
            ::waitpid(pid, &st, 0);
            char hdr[96];
            std::snprintf(hdr, sizeof(hdr), "{\"type\":\"log_history\",\"n\":%zu,\"lines\":[", count);
            SendToFd(client_fd, std::string(hdr) + lines_json + "]}");
        }).detach();
    }

    // ------------------------------------------------------------------
    // WebSSH PTY
    // ------------------------------------------------------------------
    void SshStart(Client &c, int cols, int rows)
    {
        if (c.pty_fd >= 0) { SendTo(c, WsEncodeText("{\"type\":\"ssh_ready\"}")); return; }
        if (cols < 10 || cols > 300) cols = 100;
        if (rows < 4 || rows > 100) rows = 30;

        int am = -1, sl = -1;
        struct winsize ws{};
        ws.ws_col = uint16_t(cols); ws.ws_row = uint16_t(rows);
        if (::openpty(&am, &sl, nullptr, nullptr, &ws) != 0) {
            SendTo(c, WsEncodeText("{\"type\":\"ssh_exit\",\"data\":\"openpty failed\"}"));
            return;
        }
        pid_t pid = ::fork();
        if (pid < 0) { ::close(am); ::close(sl);
            SendTo(c, WsEncodeText("{\"type\":\"ssh_exit\",\"data\":\"fork failed\"}")); return; }
        if (pid == 0) {
            // 子进程: 新会话 + 控制终端 + 登录 shell
            ::setsid();
            ::ioctl(sl, TIOCSCTTY, 0);
            ::dup2(sl, STDIN_FILENO);
            ::dup2(sl, STDOUT_FILENO);
            ::dup2(sl, STDERR_FILENO);
            if (sl > 2) ::close(sl);
            ::close(am);
            ::setenv("TERM", "xterm-256color", 1);
            ::setenv("HOME", "/root", 1);
            ::setenv("USER", "root", 1);
            ::setenv("SHELL", "/bin/bash", 1);
            ::execl("/bin/bash", "bash", "-l", (char *)nullptr);
            ::_exit(127);
        }
        ::close(sl);
        SetNonBlock(am);
        c.pty_fd = am;
        c.pty_pid = pid;
        SendTo(c, WsEncodeText("{\"type\":\"ssh_ready\"}"));
        std::cout << "[web_bridge] ssh session opened, fd=" << c.fd << " pid=" << pid << std::endl;
    }

    void SshWrite(Client &c, const std::string &data)
    {
        if (c.pty_fd < 0 || data.empty()) return;
        size_t off = 0;
        while (off < data.size()) {
            ssize_t n = ::write(c.pty_fd, data.data() + off, data.size() - off);
            if (n > 0) off += size_t(n);
            else { if (errno == EINTR) continue; break; }  // EAGAIN 时丢弃本批剩余(极少见)
        }
    }

    void SshResize(Client &c, int cols, int rows)
    {
        if (c.pty_fd < 0 || cols < 2 || rows < 2) return;
        struct winsize ws{};
        ws.ws_col = uint16_t(cols); ws.ws_row = uint16_t(rows);
        ::ioctl(c.pty_fd, TIOCSWINSZ, &ws);
    }

    // ------------------------------------------------------------------
    // 入站 JSON → Inbound → 队列
    // ------------------------------------------------------------------
    void HandleWsMessage(Client &c, const std::string &msg)
    {
        std::string type;
        if (!JsonGetString(msg, "type", type)) return;

        // 系统管理类指令的鉴权拦截(token 不匹配直接拒绝,不入控制队列)
        if (!admin_token.empty() && !c.admin_ok &&
            (type == "ssh_start" || type == "ssh_data" || type == "ssh_resize"))
        {
            SendTo(c, WsEncodeText("{\"type\":\"ssh_exit\",\"data\":\"unauthorized: invalid admin token\"}"));
            return;
        }

        Inbound in;
        if (type == "cmd_vel") {
            in.kind = Inbound::CmdVelMsg;
            JsonGetNumber(msg, "vx", in.vx);
            JsonGetNumber(msg, "vy", in.vy);
            JsonGetNumber(msg, "yaw", in.yaw);
        } else if (type == "command") {
            in.kind = Inbound::CommandMsg;
            if (!JsonGetString(msg, "data", in.text)) return;
            double seqd = 0.0;
            if (JsonGetNumber(msg, "seq", seqd) && seqd > 0.0)
                in.seq = static_cast<int>(seqd);
            // 电源/日志开关/服务管理/进程管理属于系统操作, 同样受 token 保护
            if (!admin_token.empty() && !c.admin_ok &&
                (in.text.rfind("sys_", 0) == 0 || in.text.rfind("log ", 0) == 0 || in.text == "log" ||
                 in.text.rfind("svc", 0) == 0 || in.text == "sysinfo" ||
                 in.text == "ps" || in.text.rfind("ps ", 0) == 0))
            {
                SendTo(c, WsEncodeText("{\"type\":\"feedback\",\"data\":\"unauthorized: admin token required for " + JsonEscape(in.text) + "\"}"));
                return;
            }
        } else if (type == "imu_stream") {
            in.kind = Inbound::ImuStreamMsg;
            bool flag = false;
            if (!JsonGetBool(msg, "data", flag)) return;
            in.yaw = flag ? 1.0 : 0.0;
        } else if (type == "notify_ack") {
            in.kind = Inbound::NotifyAckMsg;
            if (!JsonGetString(msg, "data", in.text)) return;
        } else if (type == "check_stand") {
            in.kind = Inbound::CheckStandMsg;
        } else if (type == "log_subscribe") {
            in.kind = Inbound::LogSubMsg;
        } else if (type == "log_unsubscribe") {
            in.kind = Inbound::LogUnsubMsg;
        } else if (type == "log_history") {
            in.kind = Inbound::LogHistoryMsg;
            JsonGetNumber(msg, "before", in.vx);
            double n = 200;
            JsonGetNumber(msg, "n", n);
            in.vy = n;
        } else if (type == "ssh_start") {
            in.kind = Inbound::SshStartMsg;
            double cols = 100, rows = 30;
            JsonGetNumber(msg, "cols", cols);
            JsonGetNumber(msg, "rows", rows);
            in.vx = cols; in.vy = rows;
        } else if (type == "ssh_data") {
            in.kind = Inbound::SshDataMsg;
            if (!JsonGetString(msg, "data", in.text)) return;
        } else if (type == "ssh_resize") {
            in.kind = Inbound::SshResizeMsg;
            double cols = 100, rows = 30;
            JsonGetNumber(msg, "cols", cols);
            JsonGetNumber(msg, "rows", rows);
            in.vx = cols; in.vy = rows;
        } else if (type == "ssh_stop") {
            in.kind = Inbound::SshStopMsg;
        } else {
            return;  // 未知类型忽略
        }

        // 日志/终端会话与连接强相关, 不走串行队列(避免被控制指令阻塞),立即处理
        switch (in.kind) {
            case Inbound::LogSubMsg:
                if (!c.log_on) {
                    c.log_on = true;
                    StartLogStream();
                }
                return;
            case Inbound::LogUnsubMsg:
                LogLeaveLocked(c);
                return;
            case Inbound::LogHistoryMsg:
                SendLogHistory(c, in.vx, int(in.vy));
                return;
            case Inbound::SshStartMsg:
                SshStart(c, int(in.vx), int(in.vy));
                return;
            case Inbound::SshDataMsg:
                SshWrite(c, in.text);
                return;
            case Inbound::SshResizeMsg:
                SshResize(c, int(in.vx), int(in.vy));
                return;
            case Inbound::SshStopMsg:
                SendTo(c, WsEncodeText("{\"type\":\"ssh_exit\",\"data\":\"stopped by client\"}"));
                PtyClose(c);
                return;
            default:
                break;
        }

        {
            std::lock_guard<std::mutex> lock(q_mutex);
            if (in_queue.size() >= kQueueCap) in_queue.pop_front();
            in_queue.push_back(std::move(in));
        }
        q_cv.notify_one();
    }

    void DispatchLoop()
    {
        while (running.load()) {
            Inbound in;
            {
                std::unique_lock<std::mutex> lock(q_mutex);
                q_cv.wait_for(lock, std::chrono::milliseconds(200),
                              [this] { return !in_queue.empty(); });
                if (in_queue.empty()) continue;
                in = std::move(in_queue.front());
                in_queue.pop_front();
            }
            switch (in.kind) {
                case Inbound::CmdVelMsg:
                    if (on_cmd_vel) on_cmd_vel({in.vx, in.vy, in.yaw});
                    break;
                case Inbound::CommandMsg:
                    if (on_command) on_command(in.text, in.seq);
                    break;
                case Inbound::ImuStreamMsg:
                    if (on_imu_stream) on_imu_stream(in.yaw != 0.0);
                    break;
                case Inbound::NotifyAckMsg:
                    if (on_notify_ack) on_notify_ack(in.text);
                    break;
                case Inbound::CheckStandMsg:
                    if (on_check_stand) on_check_stand();
                    break;
                default:
                    break;
            }
        }
    }

    // ------------------------------------------------------------------
    // HTTP 小工具
    // ------------------------------------------------------------------
    static std::map<std::string, std::string> ParseQuery(const std::string &query)
    {
        std::map<std::string, std::string> q;
        size_t i = 0;
        while (i < query.size()) {
            size_t amp = query.find('&', i);
            std::string kv = query.substr(i, amp == std::string::npos ? std::string::npos : amp - i);
            size_t eq = kv.find('=');
            if (eq != std::string::npos)
                q[UrlDecode(kv.substr(0, eq))] = UrlDecode(kv.substr(eq + 1));
            else
                q[UrlDecode(kv)] = "";
            if (amp == std::string::npos) break;
            i = amp + 1;
        }
        return q;
    }

    static std::string Basename(const std::string &p)
    {
        size_t s = p.find_last_of('/');
        return s == std::string::npos ? p : p.substr(s + 1);
    }

    void SendJson(Client &c, int code, const std::string &json, const std::string &extra = "")
    {
        auto reason_of = [](int cd) -> const char * {
            switch (cd) {
                case 200: return "200 OK";
                case 400: return "400 Bad Request";
                case 401: return "401 Unauthorized";
                case 403: return "403 Forbidden";
                case 404: return "404 Not Found";
                case 405: return "405 Method Not Allowed";
                case 500: return "500 Internal Server Error";
                default:  return "200 OK";
            }
        };
        std::ostringstream h;
        h << "HTTP/1.1 " << reason_of(code) << "\r\n"
          << "Content-Type: application/json; charset=utf-8\r\n"
          << "Content-Length: " << json.size() << "\r\n"
          << "Cache-Control: no-store\r\n"
          << "Connection: close\r\n" << extra << "\r\n" << json;
        c.http_done = true;
        c.close_after_flush = true;
        SendTo(c, h.str());
    }

    // ------------------------------------------------------------------
    // WebFTP API
    // ------------------------------------------------------------------
    void FsList(Client &c, const std::string &path)
    {
        DIR *d = ::opendir(path.c_str());
        if (!d) { SendJson(c, 404, "{\"ok\":false,\"error\":\"opendir failed\"}"); return; }
        struct Entry { std::string name; bool isdir; off_t size; time_t mtime; };
        std::vector<Entry> entries;
        struct dirent *de;
        while ((de = ::readdir(d)) != nullptr) {
            std::string name = de->d_name;
            if (name == "." || name == "..") continue;
            std::string full = path + (path.back() == '/' ? "" : "/") + name;
            struct stat st{};
            bool isdir = false; off_t size = 0; time_t mt = 0;
            if (::stat(full.c_str(), &st) == 0) {
                isdir = S_ISDIR(st.st_mode); size = st.st_size; mt = st.st_mtime;
            }
            entries.push_back({name, isdir, size, mt});
        }
        ::closedir(d);
        std::sort(entries.begin(), entries.end(), [](const Entry &a, const Entry &b) {
            if (a.isdir != b.isdir) return a.isdir > b.isdir;
            return a.name < b.name;
        });
        std::ostringstream o;
        o << "{\"ok\":true,\"path\":\"" << JsonEscape(path) << "\",\"entries\":[";
        bool first = true;
        if (path != "/") {
            o << "{\"name\":\"..\",\"dir\":true,\"size\":0,\"mtime\":0}";
            first = false;
        }
        for (const auto &e : entries) {
            if (!first) o << ',';
            first = false;
            o << "{\"name\":\"" << JsonEscape(e.name) << "\",\"dir\":" << (e.isdir ? "true" : "false")
              << ",\"size\":" << e.size << ",\"mtime\":" << e.mtime << "}";
        }
        o << "]}";
        SendJson(c, 200, o.str());
    }

    void FsDownload(Client &c, const std::string &path)
    {
        int fd = ::open(path.c_str(), O_RDONLY);
        struct stat st{};
        if (fd < 0 || ::fstat(fd, &st) != 0 || !S_ISREG(st.st_mode)) {
            if (fd >= 0) ::close(fd);
            SendJson(c, 404, "{\"ok\":false,\"error\":\"file not found\"}");
            return;
        }
        std::string name = Basename(path);
        std::replace(name.begin(), name.end(), '"', '_');
        std::ostringstream h;
        h << "HTTP/1.1 200 OK\r\n"
          << "Content-Type: application/octet-stream\r\n"
          << "Content-Length: " << st.st_size << "\r\n"
          << "Content-Disposition: attachment; filename=\"" << name << "\"\r\n"
          << "Cache-Control: no-store\r\nConnection: close\r\n\r\n";
        c.http_buf.clear();
        c.http_done = true;
        SendTo(c, h.str());
        c.file_fd = fd;
        c.file_off = 0;
        c.file_size = st.st_size;
        c.close_after_flush = true;
    }

    static void RmRecursive(const std::string &p)
    {
        struct stat st{};
        if (::lstat(p.c_str(), &st) != 0) return;
        if (S_ISDIR(st.st_mode)) {
            DIR *d = ::opendir(p.c_str());
            if (d) {
                struct dirent *de;
                while ((de = ::readdir(d)) != nullptr) {
                    std::string n = de->d_name;
                    if (n == "." || n == "..") continue;
                    RmRecursive(p + (p.back() == '/' ? "" : "/") + n);
                }
                ::closedir(d);
            }
            ::rmdir(p.c_str());
        } else {
            ::unlink(p.c_str());
        }
    }

    void FsDelete(Client &c, const std::string &path, bool recursive)
    {
        if (path == "/" || path.empty()) { SendJson(c, 400, "{\"ok\":false,\"error\":\"refused\"}"); return; }
        struct stat st{};
        if (::stat(path.c_str(), &st) != 0) { SendJson(c, 404, "{\"ok\":false,\"error\":\"not found\"}"); return; }
        bool ok;
        if (S_ISDIR(st.st_mode)) {
            if (recursive) { RmRecursive(path); ok = (::access(path.c_str(), F_OK) != 0); }
            else ok = (::rmdir(path.c_str()) == 0);
        } else {
            ok = (::unlink(path.c_str()) == 0);
        }
        SendJson(c, ok ? 200 : 500, ok ? "{\"ok\":true}" : "{\"ok\":false,\"error\":\"delete failed\"}");
    }

    void FsMkdir(Client &c, const std::string &path)
    {
        if (::mkdir(path.c_str(), 0755) == 0) SendJson(c, 200, "{\"ok\":true}");
        else SendJson(c, 500, std::string("{\"ok\":false,\"error\":\"") + std::strerror(errno) + "\"}");
    }

    // ── WebFTP 增强: read / stat / copy / move ───────────────────────
    static constexpr off_t kFsReadMaxBytes = 2 * 1024 * 1024;  // 在线编辑最大读取 2 MiB

    static std::string PermString(mode_t m)
    {
        static const char kRwx[] = "rwxrwxrwx";
        std::string s(9, '-');
        for (int i = 0; i < 9; ++i)
            if (m & (1 << (8 - i))) s[size_t(i)] = kRwx[i];
        return s;
    }

    // GET /api/fs/?op=read&path=...  读取文本文件(>2MiB 截断,含 NUL 视为二进制不返回内容)
    void FsRead(Client &c, const std::string &path)
    {
        struct stat st{};
        if (::stat(path.c_str(), &st) != 0) { SendJson(c, 404, "{\"ok\":false,\"error\":\"not found\"}"); return; }
        if (S_ISDIR(st.st_mode))  { SendJson(c, 400, "{\"ok\":false,\"error\":\"is a directory\"}"); return; }
        if (!S_ISREG(st.st_mode)) { SendJson(c, 400, "{\"ok\":false,\"error\":\"not a regular file\"}"); return; }
        int fd = ::open(path.c_str(), O_RDONLY);
        if (fd < 0) { SendJson(c, 500, std::string("{\"ok\":false,\"error\":\"") + std::strerror(errno) + "\"}"); return; }
        bool truncated = st.st_size > kFsReadMaxBytes;
        size_t want = size_t(std::min<off_t>(st.st_size, kFsReadMaxBytes));
        std::string buf(want, '\0');
        size_t off = 0;
        while (off < want) {
            ssize_t r = ::read(fd, &buf[off], want - off);
            if (r > 0) off += size_t(r);
            else if (errno == EINTR) continue;
            else break;
        }
        ::close(fd);
        buf.resize(off);
        bool binary = buf.find('\0') != std::string::npos;
        std::ostringstream o;
        o << "{\"ok\":true,\"path\":\"" << JsonEscape(path) << "\""
          << ",\"size\":" << st.st_size
          << ",\"truncated\":" << (truncated ? "true" : "false")
          << ",\"binary\":" << (binary ? "true" : "false")
          << ",\"content\":";
        if (binary || (truncated && binary)) o << "null";
        else o << "\"" << JsonEscape(buf) << "\"";
        o << "}";
        SendJson(c, 200, o.str());
    }

    // GET /api/fs/?op=stat&path=...  文件属性详情
    void FsStat(Client &c, const std::string &path)
    {
        struct stat st{};
        struct stat lst{};
        if (::lstat(path.c_str(), &lst) != 0) { SendJson(c, 404, "{\"ok\":false,\"error\":\"not found\"}"); return; }
        bool link = S_ISLNK(lst.st_mode);
        const struct stat &use = (link && ::stat(path.c_str(), &st) == 0) ? st : lst;
        char modeOct[16];
        std::snprintf(modeOct, sizeof(modeOct), "%04o", use.st_mode & 07777);
        const char *ftype = "other";
        if (S_ISDIR(use.st_mode)) ftype = "dir";
        else if (S_ISREG(use.st_mode)) ftype = "file";
        else if (S_ISLNK(lst.st_mode)) ftype = "symlink";
        else if (S_ISCHR(use.st_mode)) ftype = "char-dev";
        else if (S_ISBLK(use.st_mode)) ftype = "block-dev";
        else if (S_ISFIFO(use.st_mode)) ftype = "fifo";
        else if (S_ISSOCK(use.st_mode)) ftype = "socket";
        std::string owner = std::to_string(use.st_uid);
        std::string group = std::to_string(use.st_gid);
        if (struct passwd *pw = ::getpwuid(use.st_uid); pw && pw->pw_name) owner = pw->pw_name;
        if (struct group  *gr = ::getgrgid(use.st_gid); gr && gr->gr_name) group = gr->gr_name;
        std::string linkTarget;
        if (link) {
            char tb[4096];
            ssize_t n = ::readlink(path.c_str(), tb, sizeof(tb) - 1);
            if (n > 0) { tb[n] = '\0'; linkTarget = tb; }
        }
        std::ostringstream o;
        o << "{\"ok\":true,\"path\":\"" << JsonEscape(path) << "\""
          << ",\"name\":\"" << JsonEscape(Basename(path)) << "\""
          << ",\"type\":\"" << ftype << "\""
          << ",\"dir\":" << (S_ISDIR(use.st_mode) ? "true" : "false")
          << ",\"symlink\":" << (link ? "true" : "false")
          << ",\"target\":\"" << JsonEscape(linkTarget) << "\""
          << ",\"size\":" << use.st_size
          << ",\"perm\":\"" << PermString(use.st_mode) << "\""
          << ",\"mode\":\"" << modeOct << "\""
          << ",\"uid\":" << use.st_uid << ",\"gid\":" << use.st_gid
          << ",\"owner\":\"" << JsonEscape(owner) << "\""
          << ",\"group\":\"" << JsonEscape(group) << "\""
          << ",\"mtime\":" << use.st_mtime
          << ",\"atime\":" << use.st_atime
          << ",\"ctime\":" << use.st_ctime << "}";
        SendJson(c, 200, o.str());
    }

    static bool CopyRegularFile(const std::string &src, const std::string &dst, mode_t mode)
    {
        int in = ::open(src.c_str(), O_RDONLY);
        if (in < 0) return false;
        int out = ::open(dst.c_str(), O_WRONLY | O_CREAT | O_TRUNC, mode & 07777);
        if (out < 0) { ::close(in); return false; }
        struct stat srcst{};
        bool ok = true;
        if (::fstat(in, &srcst) != 0) ok = false;
        off_t remaining = srcst.st_size;
        while (ok && remaining > 0) {
            ssize_t n = ::sendfile(out, in, nullptr, size_t(std::min<off_t>(remaining, 1 << 20)));
            if (n > 0) remaining -= n;
            else if (errno == EINTR) continue;
            else { ok = false; break; }
        }
        ::close(out);
        ::close(in);
        return ok;
    }

    static bool CopyTree(const std::string &src, const std::string &dst)
    {
        struct stat st{};
        if (::lstat(src.c_str(), &st) != 0) return false;
        if (S_ISDIR(st.st_mode)) {
            if (::mkdir(dst.c_str(), st.st_mode & 07777) != 0 && errno != EEXIST) return false;
            DIR *d = ::opendir(src.c_str());
            if (!d) return false;
            bool ok = true;
            struct dirent *de;
            while (ok && (de = ::readdir(d)) != nullptr) {
                std::string n = de->d_name;
                if (n == "." || n == "..") continue;
                std::string join = src + (src.back() == '/' ? "" : "/") + n;
                std::string jout = dst + (dst.back() == '/' ? "" : "/") + n;
                struct stat sub{};
                if (::lstat(join.c_str(), &sub) != 0) { ok = false; break; }
                if (S_ISDIR(sub.st_mode)) ok = CopyTree(join, jout);
                else if (S_ISREG(sub.st_mode)) ok = CopyRegularFile(join, jout, sub.st_mode);
                // 其他类型(软链/设备/fifo)跳过
            }
            ::closedir(d);
            return ok;
        }
        if (S_ISREG(st.st_mode)) return CopyRegularFile(src, dst, st.st_mode);
        return false;
    }

    // 目标路径拼装: dstDir/name(空名用 src 的 basename);拒绝源/目标相同
    static bool ResolveCopyTarget(const std::string &src, const std::string &dstDir,
                                  const std::string &nameIn, std::string &out)
    {
        std::string name = nameIn.empty() ? Basename(src) : Basename(nameIn);
        if (name.empty() || name == "." || name == ".." || name.find('/') != std::string::npos) return false;
        out = dstDir + (dstDir.empty() || dstDir.back() == '/' ? "" : "/") + name;
        // 归一比较(不去 symlink,仅字符串)
        std::string a = src, b = out;
        while (a.size() > 1 && a.back() == '/') a.pop_back();
        while (b.size() > 1 && b.back() == '/') b.pop_back();
        return a != b;
    }

    // GET op=copy src=<源> dir=<目标目录> [name=<新名>]
    void FsCopy(Client &c, const std::string &src, const std::string &dstDir, const std::string &name)
    {
        struct stat sst{};
        if (src == "/" || src.empty()) { SendJson(c, 400, "{\"ok\":false,\"error\":\"refused source\"}"); return; }
        if (::lstat(src.c_str(), &sst) != 0) { SendJson(c, 404, "{\"ok\":false,\"error\":\"source not found\"}"); return; }
        struct stat dd{};
        if (::stat(dstDir.c_str(), &dd) != 0 || !S_ISDIR(dd.st_mode)) { SendJson(c, 404, "{\"ok\":false,\"error\":\"target dir not found\"}"); return; }
        std::string target;
        if (!ResolveCopyTarget(src, dstDir, name, target)) { SendJson(c, 400, "{\"ok\":false,\"error\":\"bad target\"}"); return; }
        if (::access(target.c_str(), F_OK) == 0) { SendJson(c, 409, "{\"ok\":false,\"error\":\"目标已存在\"}"); return; }
        bool ok = CopyTree(src, target);
        SendJson(c, ok ? 200 : 500, ok
            ? "{\"ok\":true,\"path\":\"" + JsonEscape(target) + "\"}"
            : std::string("{\"ok\":false,\"error\":\"copy failed: ") + std::strerror(errno) + "\"}");
    }

    // GET op=move src=<源> dir=<目标目录> [name=<新名>]  (跨文件系统自动复制+删除)
    // 与 copy 不同: name 显式给出时必须是纯文件名(含路径分隔符直接拒绝,不做静默 basename 降级);
    // 源/目标同名时幂等成功(重命名未改名),绝不走覆盖删除流程
    void FsMove(Client &c, const std::string &src, const std::string &dstDir, const std::string &name)
    {
        struct stat sst{};
        if (src == "/" || src.empty()) { SendJson(c, 400, "{\"ok\":false,\"error\":\"refused source\"}"); return; }
        if (::lstat(src.c_str(), &sst) != 0) { SendJson(c, 404, "{\"ok\":false,\"error\":\"source not found\"}"); return; }
        struct stat dd{};
        if (::stat(dstDir.c_str(), &dd) != 0 || !S_ISDIR(dd.st_mode)) { SendJson(c, 404, "{\"ok\":false,\"error\":\"target dir not found\"}"); return; }
        // 严格名称校验: 空名回退为源 basename; 拒绝 . / .. / 含 / 或 \ 的名字
        std::string useName = name.empty() ? Basename(src) : name;
        if (useName.empty() || useName == "." || useName == ".." ||
            useName.find('/') != std::string::npos || useName.find('\\') != std::string::npos) {
            SendJson(c, 400, "{\"ok\":false,\"error\":\"非法名称: 不能包含路径分隔符\"}"); return;
        }
        std::string target = dstDir + (dstDir.empty() || dstDir.back() == '/' ? "" : "/") + useName;
        // 归一比较(仅去尾斜杠)
        std::string a = src, b = target;
        while (a.size() > 1 && a.back() == '/') a.pop_back();
        while (b.size() > 1 && b.back() == '/') b.pop_back();
        if (a == b) { SendJson(c, 200, "{\"ok\":true,\"path\":\"" + JsonEscape(target) + "\"}"); return; }
        // 同名文件覆盖,目录不覆盖
        struct stat tst{};
        if (::lstat(target.c_str(), &tst) == 0) {
            if (S_ISDIR(tst.st_mode)) { SendJson(c, 409, "{\"ok\":false,\"error\":\"目标目录已存在\"}"); return; }
            ::unlink(target.c_str());
        }
        if (::rename(src.c_str(), target.c_str()) == 0) {
            SendJson(c, 200, "{\"ok\":true,\"path\":\"" + JsonEscape(target) + "\"}");
            return;
        }
        if (errno != EXDEV && errno != ENOENT) {
            SendJson(c, 500, std::string("{\"ok\":false,\"error\":\"rename: ") + std::strerror(errno) + "\"}");
            return;
        }
        // 跨文件系统: 复制成功后递归删除源
        if (CopyTree(src, target)) {
            RmRecursive(src);
            SendJson(c, 200, "{\"ok\":true,\"path\":\"" + JsonEscape(target) + "\",\"cross_fs\":true}");
        } else {
            SendJson(c, 500, std::string("{\"ok\":false,\"error\":\"cross-fs copy failed: ") + std::strerror(errno) + "\"}");
        }
    }

    // PUT /api/fs/upload?dir=<目录>  X-Filename: <名字>  Body=文件字节
    void FsUpload(Client &c, const std::map<std::string, std::string> &q,
                  const std::map<std::string, std::string> &hdr,
                  const std::string &post_header)
    {
        auto itd = q.find("dir");
        auto itf = hdr.find("x-filename");
        if (itd == q.end() || itf == hdr.end() || itf->second.empty()) {
            SendJson(c, 400, "{\"ok\":false,\"error\":\"dir and X-Filename required\"}");
            return;
        }
        std::string dir = itd->second;
        std::string name = Basename(UrlDecode(itf->second));
        if (name.empty() || name == "." || name == "..") {
            SendJson(c, 400, "{\"ok\":false,\"error\":\"bad filename\"}");
            return;
        }
        struct stat dst{};
        if (::stat(dir.c_str(), &dst) != 0 || !S_ISDIR(dst.st_mode)) {
            SendJson(c, 404, "{\"ok\":false,\"error\":\"target dir not found\"}");
            return;
        }
        size_t content_length = 0;
        auto itl = hdr.find("content-length");
        if (itl != hdr.end()) content_length = size_t(std::strtoull(itl->second.c_str(), nullptr, 10));
        if (content_length == 0) { SendJson(c, 400, "{\"ok\":false,\"error\":\"empty file\"}"); return; }

        // 临时文件建在目标目录内:保证 rename 同文件系统(避免 EXDEV)
        std::string tmplStr = dir + (dir.back() == '/' ? "" : "/") + ".webftp_XXXXXX";
        std::vector<char> tmplBuf(tmplStr.begin(), tmplStr.end());
        tmplBuf.push_back('\0');
        int tfd = ::mkstemp(tmplBuf.data());
        if (tfd < 0) { SendJson(c, 500, "{\"ok\":false,\"error\":\"mkstemp failed\"}"); return; }
        ::fchmod(tfd, 0644);   // mkstemp 默认 0600,放宽便于非 root 读取

        c.up_fd = tfd;
        c.up_tmp = tmplBuf.data();
        c.up_dst = dir + (dir.back() == '/' ? "" : "/") + name;
        c.up_left = content_length;

        // 与 header 同包到达的 body 字节先落盘
        if (!post_header.empty()) {
            size_t take = std::min(post_header.size(), c.up_left);
            ::write(c.up_fd, post_header.data(), take);
            c.up_left -= take;
        }
        c.http_done = true;
        if (c.up_left == 0) FinishUpload(c);
        // 剩余 body 由 NetLoop 的 UploadAppend 处理
    }

    void UploadAppend(Client &c, const char *data, size_t n)
    {
        size_t take = std::min(n, c.up_left);
        if (take > 0) {
            size_t off = 0;
            while (off < take) {
                ssize_t w = ::write(c.up_fd, data + off, take - off);
                if (w > 0) off += size_t(w);
                else if (errno == EINTR) continue;
                else break;
            }
            c.up_left -= take;
        }
        if (c.up_left == 0) FinishUpload(c);
    }

    void FinishUpload(Client &c)
    {
        bool ok = false;
        std::string err;
        if (c.up_fd >= 0) {
            // 修复:写入完成后先 fsync 数据落盘,再 rename,防止断电后 0 字节
            if (::fsync(c.up_fd) != 0) { err = std::strerror(errno); }
            ::close(c.up_fd); c.up_fd = -1;
            if (err.empty()) {
                // rename 原子替换(直接覆盖,不先 unlink,避免掉电窗口内旧文件丢失)
                ok = (::rename(c.up_tmp.c_str(), c.up_dst.c_str()) == 0);
                if (!ok) err = std::strerror(errno);
            }
        }
        if (ok) {
            // fsync 目录,保证 rename 元数据落盘
            std::string dir = c.up_dst.substr(0, c.up_dst.find_last_of('/'));
            if (dir.empty()) dir = "/";
            int dfd = ::open(dir.c_str(), O_RDONLY | O_DIRECTORY);
            if (dfd >= 0) { ::fsync(dfd); ::close(dfd); }
        }
        ::unlink(c.up_tmp.c_str());
        c.up_tmp.clear();
        std::string body = ok
            ? "{\"ok\":true,\"path\":\"" + JsonEscape(c.up_dst) + "\"}"
            : "{\"ok\":false,\"error\":\"" + JsonEscape(err) + "\"}";
        c.up_dst.clear();
        SendJson(c, ok ? 200 : 500, body);
    }

    void HandleFsApi(Client &c, const std::string &method,
                     const std::map<std::string, std::string> &q,
                     const std::map<std::string, std::string> &hdr,
                     const std::string &post_header)
    {
        // 鉴权: query token / X-Admin-Token 头 / Cookie
        if (!AdminTokenOk(RequestToken(q, hdr))) {
            SendJson(c, 401, "{\"ok\":false,\"error\":\"unauthorized\"}"); return;
        }

        auto op = q.find("op");
        std::string opv = op != q.end() ? op->second : "";
        if (method == "GET" && opv == "list") {
            std::string p = q.count("path") ? q.at("path") : "/";
            if (p.empty()) p = "/";
            FsList(c, p);
        } else if (method == "GET" && opv == "download") {
            if (!q.count("path")) { SendJson(c, 400, "{\"ok\":false}"); return; }
            FsDownload(c, q.at("path"));
        } else if (method == "GET" && opv == "delete") {
            if (!q.count("path")) { SendJson(c, 400, "{\"ok\":false}"); return; }
            FsDelete(c, q.at("path"), q.count("recursive") && q.at("recursive") == "1");
        } else if (method == "GET" && opv == "mkdir") {
            if (!q.count("path")) { SendJson(c, 400, "{\"ok\":false}"); return; }
            FsMkdir(c, q.at("path"));
        } else if (method == "GET" && opv == "read") {
            if (!q.count("path")) { SendJson(c, 400, "{\"ok\":false}"); return; }
            FsRead(c, q.at("path"));
        } else if (method == "GET" && opv == "stat") {
            if (!q.count("path")) { SendJson(c, 400, "{\"ok\":false}"); return; }
            FsStat(c, q.at("path"));
        } else if (method == "GET" && opv == "copy") {
            if (!q.count("src") || !q.count("dir")) { SendJson(c, 400, "{\"ok\":false,\"error\":\"src and dir required\"}"); return; }
            FsCopy(c, q.at("src"), q.at("dir"), q.count("name") ? q.at("name") : "");
        } else if (method == "GET" && opv == "move") {
            if (!q.count("src") || !q.count("dir")) { SendJson(c, 400, "{\"ok\":false,\"error\":\"src and dir required\"}"); return; }
            FsMove(c, q.at("src"), q.at("dir"), q.count("name") ? q.at("name") : "");
        } else if (method == "PUT" && opv == "upload") {
            FsUpload(c, q, hdr, post_header);
        } else {
            SendJson(c, 400, "{\"ok\":false,\"error\":\"unknown fs api\"}");
        }
    }

    // ------------------------------------------------------------------
    // 模型管理 API (/api/model/)
    // 模型根目录 = POLICY_DIR;结构: <robot>/<submodel>/config.yaml + *.onnx|*.pt
    // 「启用」= 改写对应 config.yaml 中的 model_name 字段(需重启控制服务生效)
    // ------------------------------------------------------------------
    // 名称合法性: 只允许 [A-Za-z0-9._-],拒绝 /、..、控制字符
    static bool ModelNameSafe(const std::string &s)
    {
        if (s.empty() || s == "." || s == "..") return false;
        for (unsigned char ch : s) {
            if (ch == '/' || ch == '\\' || ch < 0x20) return false;
        }
        return true;
    }

    // 简单读取 config.yaml 中顶层 model_name: 字段(单行匹配,兼容单/双引号/裸值)
    static std::string ModelReadModelName(const std::string &cfg_path)
    {
        std::ifstream ifs(cfg_path);
        if (!ifs) return "";
        std::string line;
        while (std::getline(ifs, line)) {
            // 去掉 \r
            while (!line.empty() && (line.back() == '\r' || line.back() == ' ' || line.back() == '\t')) line.pop_back();
            // 跳过注释/空行
            size_t i = 0;
            while (i < line.size() && (line[i] == ' ' || line[i] == '\t')) ++i;
            if (i >= line.size() || line[i] == '#') continue;
            const char kKey[] = "model_name:";
            const size_t kLen = sizeof(kKey) - 1;
            if (line.compare(i, kLen, kKey) != 0) continue;
            std::string v = line.substr(i + kLen);
            // 去掉首尾空白
            size_t a = v.find_first_not_of(" \t");
            size_t b = v.find_last_not_of(" \t");
            if (a == std::string::npos) return "";
            v = v.substr(a, b - a + 1);
            // 去掉引号
            if (v.size() >= 2 && ((v.front() == '"' && v.back() == '"') ||
                                  (v.front() == '\'' && v.back() == '\''))) {
                v = v.substr(1, v.size() - 2);
            }
            // 去掉行尾注释
            size_t hash = v.find(" #");
            if (hash != std::string::npos) {
                v = v.substr(0, hash);
                size_t nb = v.find_last_not_of(" \t");
                v = nb == std::string::npos ? "" : v.substr(0, nb + 1);
            }
            return v;
        }
        return "";
    }

    // 把 config.yaml 里的 model_name 字段值替换为 new_name(保留缩进/引号风格)
    // 修复:改为 write-tmp -> fsync -> rename -> fsync(dir),防止断电后半截文件/0字节
    static bool ModelWriteModelName(const std::string &cfg_path, const std::string &new_name)
    {
        std::ifstream ifs(cfg_path);
        if (!ifs) return false;
        std::string all;
        {
            std::ostringstream ss; ss << ifs.rdbuf(); all = ss.str();
        }
        ifs.close();

        std::istringstream iss(all);
        std::ostringstream oss;
        std::string line;
        bool replaced = false;
        while (std::getline(iss, line)) {
            std::string out_line = line;
            if (!replaced) {
                std::string stripped = line;
                size_t i = 0;
                while (i < stripped.size() && (stripped[i] == ' ' || stripped[i] == '\t')) ++i;
                const char kKey[] = "model_name:";
                const size_t kLen = sizeof(kKey) - 1;
                if (i < stripped.size() && stripped[i] != '#' &&
                    stripped.compare(i, kLen, kKey) == 0) {
                    std::string indent = line.substr(0, i);
                    out_line = indent + "model_name: \"" + new_name + "\"";
                    replaced = true;
                }
            }
            oss << out_line << "\n";
        }
        if (!replaced) return false;

        // 1) 写临时文件
        std::string tmp_path = cfg_path + ".tmp_model_switch";
        int tfd = ::open(tmp_path.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
        if (tfd < 0) return false;
        const std::string content = oss.str();
        size_t off = 0;
        while (off < content.size()) {
            ssize_t w = ::write(tfd, content.data() + off, content.size() - off);
            if (w > 0) off += size_t(w);
            else if (errno == EINTR) continue;
            else { ::close(tfd); ::unlink(tmp_path.c_str()); return false; }
        }
        // 2) fsync 数据落盘
        if (::fsync(tfd) != 0) { ::close(tfd); ::unlink(tmp_path.c_str()); return false; }
        ::close(tfd);
        // 3) rename 原子替换(直接覆盖,不先 unlink)
        if (::rename(tmp_path.c_str(), cfg_path.c_str()) != 0) {
            ::unlink(tmp_path.c_str()); return false;
        }
        // 4) fsync 目录,保证 rename 元数据落盘
        std::string dir = cfg_path.substr(0, cfg_path.find_last_of('/'));
        if (dir.empty()) dir = "/";
        int dfd = ::open(dir.c_str(), O_RDONLY | O_DIRECTORY);
        if (dfd >= 0) { ::fsync(dfd); ::close(dfd); }
        return true;
    }

    // 列出 POLICY_DIR/<robot>/<submodel>/{*.onnx,*.pt} + config.yaml 中的 model_name
    void ModelList(Client &c)
    {
        std::string base = POLICY_DIR;
        std::ostringstream o;
        o << "{\"ok\":true,\"policy_dir\":\"" << JsonEscape(base) << "\",\"robots\":[";
        DIR *dr = ::opendir(base.c_str());
        bool first_robot = true;
        if (dr) {
            struct dirent *de;
            std::vector<std::string> robot_names;
            while ((de = ::readdir(dr)) != nullptr) {
                std::string nm = de->d_name;
                if (nm == "." || nm == "..") continue;
                std::string full = base + "/" + nm;
                struct stat st{};
                if (::stat(full.c_str(), &st) != 0 || !S_ISDIR(st.st_mode)) continue;
                robot_names.push_back(nm);
            }
            ::closedir(dr);
            std::sort(robot_names.begin(), robot_names.end());
            for (const auto &rname : robot_names) {
                if (!first_robot) o << ',';
                first_robot = false;
                o << "{\"name\":\"" << JsonEscape(rname) << "\",\"models\":[";
                std::string rdir = base + "/" + rname;
                DIR *d2 = ::opendir(rdir.c_str());
                bool first_model = true;
                if (d2) {
                    struct dirent *de2;
                    std::vector<std::string> sub_names;
                    while ((de2 = ::readdir(d2)) != nullptr) {
                        std::string nm = de2->d_name;
                        if (nm == "." || nm == "..") continue;
                        std::string full = rdir + "/" + nm;
                        struct stat st{};
                        if (::stat(full.c_str(), &st) != 0 || !S_ISDIR(st.st_mode)) continue;
                        sub_names.push_back(nm);
                    }
                    ::closedir(d2);
                    std::sort(sub_names.begin(), sub_names.end());
                    for (const auto &sname : sub_names) {
                        std::string sdir = rdir + "/" + sname;
                        std::string cfg = sdir + "/config.yaml";
                        struct stat cst{};
                        bool has_cfg = (::stat(cfg.c_str(), &cst) == 0 && S_ISREG(cst.st_mode));
                        std::string active = has_cfg ? ModelReadModelName(cfg) : "";
                        // 列出模型文件(*.onnx / *.pt / *.pt.ph / *.jit 等常见推理扩展)
                        std::vector<std::pair<std::string, std::pair<off_t, time_t>>> files;
                        DIR *d3 = ::opendir(sdir.c_str());
                        if (d3) {
                            struct dirent *de3;
                            while ((de3 = ::readdir(d3)) != nullptr) {
                                std::string fn = de3->d_name;
                                if (fn == "." || fn == "..") continue;
                                std::string full = sdir + "/" + fn;
                                struct stat fst{};
                                if (::stat(full.c_str(), &fst) != 0 || !S_ISREG(fst.st_mode)) continue;
                                // 过滤: 只显示模型类文件 + config.yaml
                                auto ends = [](const std::string &s, const char *suf) {
                                    size_t n = std::strlen(suf);
                                    return s.size() >= n && s.compare(s.size() - n, n, suf) == 0;
                                };
                                if (!(ends(fn, ".onnx") || ends(fn, ".pt") || ends(fn, ".pth") ||
                                      ends(fn, ".jit") || ends(fn, ".ckpt") || fn == "config.yaml"))
                                    continue;
                                files.push_back({fn, {fst.st_size, fst.st_mtime}});
                            }
                            ::closedir(d3);
                            std::sort(files.begin(), files.end(),
                                      [](const auto &a, const auto &b) { return a.first < b.first; });
                        }
                        if (!first_model) o << ',';
                        first_model = false;
                        o << "{\"name\":\"" << JsonEscape(sname) << "\""
                          << ",\"has_config\":" << (has_cfg ? "true" : "false")
                          << ",\"active\":\"" << JsonEscape(active) << "\""
                          << ",\"files\":[";
                        bool first_f = true;
                        for (const auto &f : files) {
                            if (!first_f) o << ',';
                            first_f = false;
                            o << "{\"name\":\"" << JsonEscape(f.first) << "\""
                              << ",\"size\":" << f.second.first
                              << ",\"mtime\":" << f.second.second << "}";
                        }
                        o << "]}";
                    }
                }
                o << "]}";
            }
        }
        o << "]}";
        SendJson(c, 200, o.str());
    }

    // 启用: 改写 POLICY_DIR/<robot>/<sub>/config.yaml 中 model_name
    void ModelActivate(Client &c, const std::string &robot, const std::string &sub, const std::string &file)
    {
        if (!ModelNameSafe(robot) || !ModelNameSafe(sub) || !ModelNameSafe(file)) {
            SendJson(c, 400, "{\"ok\":false,\"error\":\"bad name\"}"); return;
        }
        std::string dir = std::string(POLICY_DIR) + "/" + robot + "/" + sub;
        std::string cfg = dir + "/config.yaml";
        std::string mdl = dir + "/" + file;
        struct stat st{};
        if (::stat(mdl.c_str(), &st) != 0 || !S_ISREG(st.st_mode)) {
            SendJson(c, 404, "{\"ok\":false,\"error\":\"model file not found\"}"); return;
        }
        if (::stat(cfg.c_str(), &st) != 0 || !S_ISREG(st.st_mode)) {
            SendJson(c, 404, "{\"ok\":false,\"error\":\"config.yaml not found\"}"); return;
        }
        if (!ModelWriteModelName(cfg, file)) {
            SendJson(c, 500, "{\"ok\":false,\"error\":\"rewrite config failed (no model_name field?)\"}"); return;
        }
        std::ostringstream o;
        o << "{\"ok\":true,\"robot\":\"" << JsonEscape(robot) << "\",\"model\":\"" << JsonEscape(sub)
          << "\",\"active\":\"" << JsonEscape(file)
          << "\",\"hint\":\"已写入 config.yaml;无需重启服务:先趴下(退出强化学习),再重新进入强化学习即热加载新模型\"}";
        SendJson(c, 200, o.str());
    }

    // 删除单个模型文件
    void ModelDeleteFile(Client &c, const std::string &robot, const std::string &sub, const std::string &file)
    {
        if (!ModelNameSafe(robot) || !ModelNameSafe(sub) || !ModelNameSafe(file)) {
            SendJson(c, 400, "{\"ok\":false,\"error\":\"bad name\"}"); return;
        }
        if (file == "config.yaml") {
            SendJson(c, 400, "{\"ok\":false,\"error\":\"不允许删除 config.yaml\"}"); return;
        }
        std::string p = std::string(POLICY_DIR) + "/" + robot + "/" + sub + "/" + file;
        struct stat st{};
        if (::stat(p.c_str(), &st) != 0 || !S_ISREG(st.st_mode)) {
            SendJson(c, 404, "{\"ok\":false,\"error\":\"not found\"}"); return;
        }
        // 若删除的正是当前激活模型,先检查并提示(不阻止,但让前端确认)
        bool ok = (::unlink(p.c_str()) == 0);
        SendJson(c, ok ? 200 : 500, ok ? "{\"ok\":true}" : "{\"ok\":false,\"error\":\"unlink failed\"}");
    }

    // 删除整个子模型目录
    void ModelDeleteDir(Client &c, const std::string &robot, const std::string &sub)
    {
        if (!ModelNameSafe(robot) || !ModelNameSafe(sub)) {
            SendJson(c, 400, "{\"ok\":false,\"error\":\"bad name\"}"); return;
        }
        std::string p = std::string(POLICY_DIR) + "/" + robot + "/" + sub;
        struct stat st{};
        if (::stat(p.c_str(), &st) != 0 || !S_ISDIR(st.st_mode)) {
            SendJson(c, 404, "{\"ok\":false,\"error\":\"not found\"}"); return;
        }
        RmRecursive(p);
        bool ok = (::access(p.c_str(), F_OK) != 0);
        SendJson(c, ok ? 200 : 500, ok ? "{\"ok\":true}" : "{\"ok\":false,\"error\":\"delete failed\"}");
    }

    // 新建子模型目录(自动补一个最小 config.yaml 模板)
    void ModelMkdir(Client &c, const std::string &robot, const std::string &sub)
    {
        if (!ModelNameSafe(robot) || !ModelNameSafe(sub)) {
            SendJson(c, 400, "{\"ok\":false,\"error\":\"bad name\"}"); return;
        }
        std::string rdir = std::string(POLICY_DIR) + "/" + robot;
        std::string sdir = rdir + "/" + sub;
        struct stat st{};
        if (::stat(rdir.c_str(), &st) != 0 || !S_ISDIR(st.st_mode)) {
            SendJson(c, 404, "{\"ok\":false,\"error\":\"robot dir not found\"}"); return;
        }
        if (::stat(sdir.c_str(), &st) == 0) {
            SendJson(c, 409, "{\"ok\":false,\"error\":\"已存在\"}"); return;
        }
        if (::mkdir(sdir.c_str(), 0755) != 0) {
            SendJson(c, 500, std::string("{\"ok\":false,\"error\":\"") + std::strerror(errno) + "\"}");
            return;
        }
        SendJson(c, 200, "{\"ok\":true}");
    }

    // 下载模型文件
    void ModelDownload(Client &c, const std::string &robot, const std::string &sub, const std::string &file)
    {
        if (!ModelNameSafe(robot) || !ModelNameSafe(sub) || !ModelNameSafe(file)) {
            SendJson(c, 400, "{\"ok\":false,\"error\":\"bad name\"}"); return;
        }
        std::string p = std::string(POLICY_DIR) + "/" + robot + "/" + sub + "/" + file;
        FsDownload(c, p);
    }

    // 上传模型文件(复用 FsUpload 的流式落盘机制,仅覆盖 dir 到目标子模型目录)
    void ModelUpload(Client &c, const std::string &robot, const std::string &sub,
                     const std::map<std::string, std::string> &hdr,
                     const std::string &post_header)
    {
        if (!ModelNameSafe(robot) || !ModelNameSafe(sub)) {
            SendJson(c, 400, "{\"ok\":false,\"error\":\"bad name\"}"); return;
        }
        std::string dir = std::string(POLICY_DIR) + "/" + robot + "/" + sub;
        struct stat st{};
        if (::stat(dir.c_str(), &st) != 0 || !S_ISDIR(st.st_mode)) {
            SendJson(c, 404, "{\"ok\":false,\"error\":\"submodel dir not found\"}"); return;
        }
        std::map<std::string, std::string> q2;
        q2["dir"] = dir;
        FsUpload(c, q2, hdr, post_header);
    }

    void HandleModelApi(Client &c, const std::string &method,
                        const std::map<std::string, std::string> &q,
                        const std::map<std::string, std::string> &hdr,
                        const std::string &post_header)
    {
        // 鉴权: 与 /api/fs/ 相同(query token / X-Admin-Token 头 / Cookie)
        if (!AdminTokenOk(RequestToken(q, hdr))) {
            SendJson(c, 401, "{\"ok\":false,\"error\":\"unauthorized\"}"); return;
        }

        auto op = q.find("op");
        std::string opv = op != q.end() ? op->second : "";
        auto getq = [&](const char *k) -> std::string {
            auto it2 = q.find(k);
            return it2 != q.end() ? it2->second : "";
        };
        if (method == "GET" && opv == "list") {
            ModelList(c);
        } else if (method == "GET" && opv == "activate") {
            ModelActivate(c, getq("robot"), getq("model"), getq("file"));
        } else if (method == "GET" && opv == "delete") {
            std::string f = getq("file");
            if (f.empty()) ModelDeleteDir(c, getq("robot"), getq("model"));
            else ModelDeleteFile(c, getq("robot"), getq("model"), f);
        } else if (method == "GET" && opv == "mkdir") {
            ModelMkdir(c, getq("robot"), getq("model"));
        } else if (method == "GET" && opv == "download") {
            ModelDownload(c, getq("robot"), getq("model"), getq("file"));
        } else if (method == "PUT" && opv == "upload") {
            ModelUpload(c, getq("robot"), getq("model"), hdr, post_header);
        } else {
            SendJson(c, 400, "{\"ok\":false,\"error\":\"unknown model api\"}");
        }
    }

    // ------------------------------------------------------------------
    // 登录 / 登出 / 鉴权状态
    // ------------------------------------------------------------------
    void HandleAuth(Client &c, const std::string &url_path, const std::string &method,
                    const std::map<std::string, std::string> &hdr,
                    const std::string &post_header)
    {
        // GET /api/login: 探测服务端是否启用登录
        if (method == "GET" && url_path == "/api/login") {
            SendJson(c, 200,
                     std::string("{\"ok\":true,\"authRequired\":")
                         + (admin_token.empty() ? "false" : "true") + "}");
            return;
        }
        // 任意方法 /api/logout: 会话 Cookie 立即过期
        if (url_path == "/api/logout") {
            SendJson(c, 200, "{\"ok\":true}",
                     "Set-Cookie: jxg_admin=; Path=/; Max-Age=0; "
                     "HttpOnly; SameSite=Lax\r\n");
            return;
        }
        // POST /api/login: 校验令牌,成功则下发 30 天 HttpOnly 会话 Cookie
        if (method == "POST" && url_path == "/api/login") {
            // 请求体未按 Content-Length 收全时等待(http_done 不置位,
            // 后续数据到达会重新进入 HandleHttp)
            auto clit = hdr.find("content-length");
            if (clit != hdr.end()) {
                unsigned long need = strtoull(clit->second.c_str(), nullptr, 10);
                if (post_header.size() < need) return;
            }
            std::map<std::string, std::string> bq = ParseQuery(post_header);
            std::string tok = bq.count("token") ? bq.at("token") : "";
            if (AdminTokenOk(tok)) {
                std::string set_cookie =
                    "Set-Cookie: jxg_admin=" + UrlEncode(tok)
                    + "; Path=/; Max-Age=2592000; HttpOnly; SameSite=Lax\r\n";
                SendJson(c, 200, "{\"ok\":true}", set_cookie);
            } else {
                SendJson(c, 401, "{\"ok\":false,\"error\":\"invalid token\"}");
            }
            return;
        }
        SendJson(c, 400, "{\"ok\":false,\"error\":\"bad auth request\"}");
    }

    // ------------------------------------------------------------------
    // HTTP / WS 握手
    // ------------------------------------------------------------------
    void HandleHttp(Client &c)
    {
        size_t hdr_end = c.http_buf.find("\r\n\r\n");
        if (hdr_end == std::string::npos) {
            if (c.http_buf.size() > 16384) { ::close(c.fd); c.fd = -1; }
            return;
        }
        std::string headers = c.http_buf.substr(0, hdr_end);
        std::string post_header = c.http_buf.substr(hdr_end + 4);

        std::istringstream iss(headers);
        std::string method, path, version;
        iss >> method >> path >> version;
        if (path.empty() || (method != "GET" && method != "PUT" && method != "POST")) {
            SendTo(c, "HTTP/1.1 405 Method Not Allowed\r\nContent-Length: 0\r\n\r\n");
            return;
        }

        // 提取全部请求头(小写键)
        std::map<std::string, std::string> hdr;
        std::string upgrade, ws_key;
        std::string line;
        bool first = true;
        while (std::getline(iss, line)) {
            if (first) { first = false; continue; }  // 请求行已消费
            auto trim = [](std::string s) {
                while (!s.empty() && (s.back() == '\r' || s.back() == ' ')) s.pop_back();
                return s;
            };
            size_t colon = line.find(':');
            if (colon == std::string::npos) continue;
            std::string k = line.substr(0, colon);
            std::string v = trim(line.substr(colon + 1));
            while (!v.empty() && (v.front() == ' ' || v.front() == '\t')) v.erase(v.begin());
            std::transform(k.begin(), k.end(), k.begin(),
                           [](unsigned char ch) { return char(std::tolower(ch)); });
            hdr[k] = v;
            if (k == "upgrade") {
                std::string lv = v;
                std::transform(lv.begin(), lv.end(), lv.begin(),
                               [](unsigned char ch) { return char(std::tolower(ch)); });
                upgrade = lv;
            }
            if (k == "sec-websocket-key") ws_key = v;
        }

        // 拆分路径与查询
        size_t qpos = path.find('?');
        std::string url_path = qpos == std::string::npos ? path : path.substr(0, qpos);
        std::string query   = qpos == std::string::npos ? "" : path.substr(qpos + 1);
        std::map<std::string, std::string> qpar = ParseQuery(query);

        // WebSocket 升级（路径 /ws）
        if (url_path == "/ws") {
            if (upgrade != "websocket" || ws_key.empty()) {
                SendTo(c, "HTTP/1.1 400 Bad Request\r\nContent-Length: 0\r\n\r\n");
                return;
            }
            // 鉴权: query token / X-Admin-Token 头 / Cookie;未通过拒绝升级
            std::string tok = RequestToken(qpar, hdr);
            if (!AdminTokenOk(tok)) {
                SendJson(c, 401, "{\"ok\":false,\"error\":\"unauthorized\"}");
                return;
            }
            std::string resp =
                "HTTP/1.1 101 Switching Protocols\r\n"
                "Upgrade: websocket\r\n"
                "Connection: Upgrade\r\n"
                "Sec-WebSocket-Accept: " + WsAcceptKey(ws_key) + "\r\n\r\n";
            c.http_buf.clear();
            c.http_done = true;
            c.ws = true;
            c.admin_ok = true;
            SendTo(c, resp);
            c.last_recv = NowSec();
            std::cout << "[web_bridge] ws client connected, fd=" << c.fd
                      << ", clients=" << clients.size()
                      << (admin_token.empty() ? " (admin: open)" : " (admin: token ok)")
                      << std::endl;
            return;
        }

        // 登录 / 登出 / 鉴权状态
        if (url_path == "/api/login" || url_path == "/api/logout") {
            HandleAuth(c, url_path, method, hdr, post_header);
            return;
        }
        // 版本信息:进程名/版本/构建号/构建时间/工作目录(便于区分 vN 部署)
        if (method == "GET" && url_path == "/api/version") {
            char cwd[1024] = {0};
            if (!getcwd(cwd, sizeof(cwd) - 1)) cwd[0] = '\0';
            char comm[64] = {0};
            {
                std::ifstream pf("/proc/self/comm");
                if (pf) pf.getline(comm, sizeof(comm) - 1);
            }
            std::string j = "{\"ok\":true,\"proc\":\"" + JsonEscape(comm)
                          + "\",\"version\":\"" APP_VERSION
                          + "\",\"build\":\"" BUILD_NUMBER
                          + "\",\"built\":\"" BUILD_TIMESTAMP
                          + "\",\"cwd\":\"" + JsonEscape(cwd) + "\"}";
            SendJson(c, 200, j);
            return;
        }
        // WebFTP / 系统管理 HTTP API
        if (url_path.rfind("/api/fs/", 0) == 0) {
            HandleFsApi(c, method, qpar, hdr, post_header);
            return;
        }
        // 模型管理 HTTP API
        if (url_path.rfind("/api/model/", 0) == 0) {
            HandleModelApi(c, method, qpar, hdr, post_header);
            return;
        }
        if (method != "GET") {
            SendJson(c, 405, "{\"ok\":false,\"error\":\"method not allowed\"}");
            return;
        }

        ServeStatic(c, url_path, qpos != std::string::npos, qpar, hdr);
    }

    void ServeStatic(Client &c, const std::string &url_path, bool has_query,
                     const std::map<std::string, std::string> &qpar,
                     const std::map<std::string, std::string> &hdr)
    {
        std::string dec = UrlDecode(url_path);
        if (dec.empty() || dec.back() == '/') dec += "index.html";

        // 登录拦截:服务端配置了管理令牌时,除登录页自身资源外,
        // 未登录请求一律 302 到登录页(Cookie 随导航自动携带)
        if (!admin_token.empty()) {
            const bool public_asset = (dec == "/login.html" || dec == "/css/login.css");
            if (!public_asset && !AdminTokenOk(RequestToken(qpar, hdr))) {
                SendTo(c, "HTTP/1.1 302 Found\r\nLocation: /login.html\r\n"
                          "Content-Length: 0\r\nCache-Control: no-store\r\n"
                          "Connection: close\r\n\r\n");
                c.http_done = true;
                c.close_after_flush = true;
                return;
            }
        }

        std::string root_real = web_root;
        {
            char buf[4096];
            if (::realpath(web_root.c_str(), buf)) root_real = buf;
        }
        std::string full = root_real + dec;
        char resolved[4096];
        if (!::realpath(full.c_str(), resolved) ||
            std::string(resolved).rfind(root_real, 0) != 0)
        {
            SendTo(c, "HTTP/1.1 404 Not Found\r\nContent-Type: text/plain\r\n"
                      "Content-Length: 9\r\n\r\nNot Found");
            return;
        }

        int fd = ::open(resolved, O_RDONLY);
        struct stat st{};
        if (fd < 0 || ::fstat(fd, &st) != 0 || !S_ISREG(st.st_mode)) {
            if (fd >= 0) ::close(fd);
            SendTo(c, "HTTP/1.1 404 Not Found\r\nContent-Type: text/plain\r\n"
                      "Content-Length: 9\r\n\r\nNot Found");
            return;
        }

        std::string cache = has_query
            ? "Cache-Control: public, max-age=31536000, immutable\r\n"
            : "Cache-Control: no-cache\r\n";
        std::ostringstream resp_hdr;
        resp_hdr << "HTTP/1.1 200 OK\r\n"
            << "Content-Type: " << MimeType(resolved) << "\r\n"
            << "Content-Length: " << st.st_size << "\r\n"
            << cache << "Connection: close\r\n\r\n";
        c.http_buf.clear();
        c.http_done = true;
        SendTo(c, resp_hdr.str());
        c.ws = false;
        c.file_fd = fd;
        c.file_off = 0;
        c.file_size = st.st_size;
        c.close_after_flush = true;
    }

    // ------------------------------------------------------------------
    // WS 帧解析（增量）
    // ------------------------------------------------------------------
    void FeedWs(Client &c)
    {
        size_t pos = 0;
        while (true) {
            if (pos + 2 > c.in_buf.size()) break;
            uint8_t b0 = uint8_t(c.in_buf[pos]);
            uint8_t b1 = uint8_t(c.in_buf[pos + 1]);
            bool fin = b0 & 0x80;
            uint8_t opcode = b0 & 0x0F;
            bool masked = b1 & 0x80;
            uint64_t len = b1 & 0x7F;
            size_t header = 2;
            if (len == 126) {
                if (pos + 4 > c.in_buf.size()) break;
                len = (uint64_t(uint8_t(c.in_buf[pos + 2]) << 8) | uint8_t(c.in_buf[pos + 3]));
                header = 4;
            } else if (len == 127) {
                if (pos + 10 > c.in_buf.size()) break;
                len = 0;
                for (int i = 0; i < 8; ++i)
                    len = (len << 8) | uint8_t(c.in_buf[pos + 2 + i]);
                header = 10;
            }
            size_t mask_off = masked ? 4 : 0;
            if (pos + header + mask_off + len > c.in_buf.size()) break;

            std::string payload;
            payload.reserve(size_t(len));
            const uint8_t *p = reinterpret_cast<const uint8_t *>(c.in_buf.data()) + pos + header;
            if (masked) {
                const uint8_t *mk = p;
                p += 4;
                payload.resize(size_t(len));
                for (uint64_t i = 0; i < len; ++i)
                    payload[size_t(i)] = char(p[i] ^ mk[i % 4]);
            } else {
                payload.assign(reinterpret_cast<const char *>(p), size_t(len));
            }
            pos += header + mask_off + size_t(len);

            switch (opcode) {
                case 0x0:
                    c.frag_buf += payload;
                    if (fin) { HandleWsMessage(c, c.frag_buf); c.frag_buf.clear(); c.in_frag = false; }
                    break;
                case 0x1:
                case 0x2:
                    if (fin) HandleWsMessage(c, payload);
                    else { c.frag_buf = payload; c.in_frag = true; }
                    break;
                case 0x8: {
                    std::string cl;
                    cl += char(0x88); cl += char(0);
                    SendTo(c, cl);
                    ::close(c.fd);
                    c.fd = -1;
                    return;
                }
                case 0x9: {
                    std::string pong;
                    pong += char(0x8A);
                    size_t n = payload.size();
                    if (n < 126) pong += char(n);
                    else { pong += char(126); pong += char((n >> 8) & 0xFF); pong += char(n & 0xFF); }
                    pong += payload;
                    SendTo(c, pong);
                    break;
                }
                case 0xA:
                    c.ping_outstanding = false;
                    break;
                default:
                    break;
            }
        }
        c.in_buf.erase(0, pos);
    }

    // ------------------------------------------------------------------
    // 网络主循环
    // ------------------------------------------------------------------
    void NetLoop()
    {
        while (running.load()) {
            pfds.clear();
            std::vector<int> pty_owner_fds;   // 与 pfds 平行: 非 -1 表示该项是某客户端的 PTY
            if (listen_fd >= 0) { pfds.push_back({listen_fd, POLLIN, 0}); pty_owner_fds.push_back(-1); }
            std::vector<int> ids;
            {
                std::lock_guard<std::mutex> lock(bcast_mutex);
                for (auto &kv : clients) {
                    Client &cc = kv.second;
                    short ev = POLLIN | ((!cc.out_buf.empty() || cc.file_fd >= 0) ? POLLOUT : 0);
                    pfds.push_back({cc.fd, ev, 0});
                    pty_owner_fds.push_back(-1);
                    ids.push_back(cc.fd);
                    if (cc.pty_fd >= 0) {
                        pfds.push_back({cc.pty_fd, POLLIN, 0});
                        pty_owner_fds.push_back(cc.fd);
                    }
                }
            }
            int rc = ::poll(pfds.data(), nfds_t(pfds.size()), 100);
            if (rc < 0) {
                if (errno == EINTR) continue;
                break;
            }

            double now = NowSec();

            // accept
            for (size_t i = 0; i < pfds.size(); ++i) {
                if (pty_owner_fds[i] != -1) continue;
                if (pfds[i].fd == listen_fd && (pfds[i].revents & POLLIN)) {
                    while (true) {
                        int nfd = ::accept4(listen_fd, nullptr, nullptr, SOCK_NONBLOCK);
                        if (nfd < 0) break;
                        SetNonBlock(nfd);
                        std::lock_guard<std::mutex> lock(bcast_mutex);
                        Client nc;
                        nc.fd = nfd;
                        nc.last_recv = now;
                        clients[nfd] = std::move(nc);
                    }
                }
            }

            // client socket io
            {
                std::lock_guard<std::mutex> lock(bcast_mutex);
                for (int fd : ids) {
                    auto it = clients.find(fd);
                    if (it == clients.end()) continue;
                    Client &c = it->second;
                    if (c.fd < 0) continue;

                    char buf[16384];
                    while (true) {
                        ssize_t n = ::recv(c.fd, buf, sizeof(buf), 0);
                        if (n > 0) {
                            if (c.ws) c.in_buf.append(buf, size_t(n));
                            else if (c.up_fd >= 0) UploadAppend(c, buf, size_t(n));
                            else c.http_buf.append(buf, size_t(n));
                            c.last_recv = now;
                            c.ping_outstanding = false;
                        } else if (n == 0) {
                            ::close(c.fd); c.fd = -1;
                            break;
                        } else {
                            if (errno == EAGAIN || errno == EWOULDBLOCK) break;
                            ::close(c.fd); c.fd = -1;
                            break;
                        }
                        if (!c.ws && c.up_fd < 0 && c.http_buf.size() > 16384) { ::close(c.fd); c.fd = -1; break; }
                        if (c.ws && c.in_buf.size() > kMaxFramePayload * 2) { ::close(c.fd); c.fd = -1; break; }
                    }
                    if (c.fd < 0) continue;

                    if (c.ws) {
                        FeedWs(c);
                    } else if (!c.http_done) {
                        HandleHttp(c);
                    }

                    // flush out
                    while (c.fd >= 0 && !c.out_buf.empty()) {
                        ssize_t n = ::send(c.fd, c.out_buf.data(), c.out_buf.size(), MSG_NOSIGNAL);
                        if (n > 0) c.out_buf.erase(0, size_t(n));
                        else if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) break;
                        else { ::close(c.fd); c.fd = -1; break; }
                    }
                    // 静态文件/下载流式发送
                    while (c.fd >= 0 && c.file_fd >= 0 && c.out_buf.empty() &&
                           c.file_off < c.file_size) {
                        ssize_t n = ::sendfile(c.fd, c.file_fd, &c.file_off,
                                               size_t(c.file_size - c.file_off));
                        if (n > 0) continue;
                        if (n == 0) break;
                        if (errno == EAGAIN || errno == EWOULDBLOCK) break;
                        ::close(c.file_fd); c.file_fd = -1;
                        ::close(c.fd); c.fd = -1;
                        break;
                    }
                    if (c.fd >= 0 && c.file_fd >= 0 && c.file_off >= c.file_size) {
                        ::close(c.file_fd);
                        c.file_fd = -1;
                    }
                    if (c.fd >= 0 && c.out_buf.empty() && c.file_fd < 0 &&
                        c.up_fd < 0 && c.close_after_flush) {
                        ::close(c.fd);
                        c.fd = -1;
                    }
                }

                // PTY 主设备 → ssh_data 帧
                for (size_t i = 0; i < pfds.size(); ++i) {
                    int owner = pty_owner_fds[i];
                    if (owner < 0 || !(pfds[i].revents & (POLLIN | POLLHUP | POLLERR))) continue;
                    auto it = clients.find(owner);
                    if (it == clients.end() || it->second.pty_fd < 0) continue;
                    Client &c = it->second;
                    char pbuf[8192];
                    bool hangup = false;
                    while (true) {
                        ssize_t n = ::read(c.pty_fd, pbuf, sizeof(pbuf));
                        if (n > 0) {
                            std::string chunk(pbuf, size_t(n));
                            SendTo(c, WsEncodeText("{\"type\":\"ssh_data\",\"data\":\"" + JsonEscape(chunk) + "\"}"));
                        } else if (n == 0) {
                            hangup = true; break;
                        } else {
                            if (errno == EAGAIN || errno == EWOULDBLOCK) break;
                            if (errno == EIO || errno == EINVAL) hangup = true;
                            break;
                        }
                    }
                    if (hangup || pfds[i].revents & (POLLHUP | POLLERR)) {
                        if (c.pty_fd >= 0)
                            SendTo(c, WsEncodeText("{\"type\":\"ssh_exit\",\"data\":\"session closed\"}"));
                        PtyClose(c);
                    }
                }

                // 清理已关闭
                for (auto it = clients.begin(); it != clients.end();) {
                    if (it->second.fd < 0) {
                        PtyClose(it->second);
                        if (it->second.log_on) { it->second.log_on = false; StopLogStream(); }
                        if (it->second.up_fd >= 0) {
                            ::close(it->second.up_fd);
                            if (!it->second.up_tmp.empty()) ::unlink(it->second.up_tmp.c_str());
                        }
                        if (it->second.file_fd >= 0) ::close(it->second.file_fd);
                        it = clients.erase(it);
                    } else ++it;
                }
                // 保活: 20s 无入站流量发 WS ping(浏览器自动回 pong)。
                // 空闲杀连接阈值 180s:模型热加载/RL 重进时模型 IO+内存抖动会
                // 让本进程短暂停顿,pong 读取延迟导致 idle 虚高,75s 曾误杀
                // 正常连接(切换模型后 WS 断开)。ping 仍 20s 一发,浏览器
                // 恢复后 pong 到达即续命;真死连接最多 3 分钟内回收。
                for (auto &kv : clients) {
                    Client &c = kv.second;
                    if (!c.ws) continue;
                    const double idle = now - c.last_recv;
                    if (idle > 180.0) {
                        std::cout << "[web_bridge] client " << c.fd
                                  << " idle " << (int)idle << "s, closing" << std::endl;
                        ::close(c.fd); c.fd = -1; continue;
                    }
                    if (idle > 20.0 && (!c.ping_outstanding || now - c.last_ping > 20.0)) {
                        // 真正的 WebSocket 协议层 ping(0x89),浏览器内核收到后会
                        // 自动回 pong(RFC6455 §5.5.2),无需前端 JS 参与;
                        // pong 字节到达即刷新 last_recv。切勿用 0x82(空 binary 帧)
                        // —— 浏览器不会应答,空闲连接仍会被 75s 超时误杀。
                        std::string ping;
                        ping += char(0x89); ping += char(0);
                        if (c.out_buf.size() + ping.size() <= kMaxOutBuf) {
                            c.out_buf += ping;
                            c.ping_outstanding = true;
                            c.last_ping = now;
                        }
                    }
                }
                for (auto it = clients.begin(); it != clients.end();) {
                    if (it->second.fd < 0) it = clients.erase(it);
                    else ++it;
                }
            }
        }
    }
};

// ============================================================================
// 公开 API
// ============================================================================
WebBridge::WebBridge() : impl_(new Impl) {}
WebBridge::~WebBridge() { stop(); }

bool WebBridge::start(int port, const std::string &web_root,
                      CmdVelCb on_cmd_vel, CommandCb on_command, BoolCb on_imu_stream,
                      StringCb on_notify_ack, VoidCb on_check_stand)
{
    impl_->port = port;
    impl_->web_root = web_root;
    impl_->on_cmd_vel = std::move(on_cmd_vel);
    impl_->on_command = std::move(on_command);
    impl_->on_imu_stream = std::move(on_imu_stream);
    impl_->on_notify_ack = std::move(on_notify_ack);
    impl_->on_check_stand = std::move(on_check_stand);

    if (const char *tok = std::getenv("WEB_ADMIN_TOKEN")) impl_->admin_token = tok;
    if (const char *unit = std::getenv("JXG_LOG_UNIT")) impl_->log_unit = unit;

    ::signal(SIGPIPE, SIG_IGN);

    if (!impl_->StartListen()) {
        std::cerr << "[web_bridge] listen on 0.0.0.0:" << port << " failed: "
                  << std::strerror(errno) << std::endl;
        return false;
    }

    impl_->running.store(true);
    impl_->net_thread = std::thread([this] { impl_->NetLoop(); });
    impl_->dispatch_thread = std::thread([this] { impl_->DispatchLoop(); });
    std::cout << "[web_bridge] serving web_root=" << web_root
              << " + websocket /ws on 0.0.0.0:" << port << std::endl;
    if (impl_->admin_token.empty())
        std::cout << "[web_bridge] WARNING: WEB_ADMIN_TOKEN 未设置 —— WebSSH/WebFTP/电源操作对局域网完全开放" << std::endl;
    else
        std::cout << "[web_bridge] admin token auth enabled" << std::endl;
    return true;
}

void WebBridge::stop()
{
    if (!impl_->running.exchange(false)) {
        if (impl_->listen_fd >= 0) { ::close(impl_->listen_fd); impl_->listen_fd = -1; }
        return;
    }
    if (impl_->listen_fd >= 0) { ::close(impl_->listen_fd); impl_->listen_fd = -1; }
    // 停止日志进程并等待读取线程退出(LogReader 会 waitpid 子进程)
    {
        std::lock_guard<std::mutex> lk(impl_->log_mtx);
        if (impl_->log_pid > 0) ::kill(impl_->log_pid, SIGTERM);
    }
    if (impl_->log_thread.joinable()) impl_->log_thread.join();
    {
        std::lock_guard<std::mutex> lock(impl_->bcast_mutex);
        for (auto &kv : impl_->clients) {
            impl_->PtyClose(kv.second);
            ::close(kv.first);
        }
        impl_->clients.clear();
    }
    impl_->q_cv.notify_all();
    if (impl_->net_thread.joinable()) impl_->net_thread.join();
    if (impl_->dispatch_thread.joinable()) impl_->dispatch_thread.join();
}

void WebBridge::publishMotorState(const std::vector<float> &data)
{ impl_->PublishFloatArray("motor_state", data); }

void WebBridge::publishImuState(const std::vector<float> &data)
{ impl_->PublishFloatArray("imu_state", data); }

void WebBridge::publishPose2D(const std::vector<float> &data)
{ impl_->PublishFloatArray("pose2d", data); }

void WebBridge::publishFeedback(const std::string &text)
{ impl_->PublishString("feedback", text); }

void WebBridge::publishNotify(const std::string &json)
{
    std::string out = "{\"type\":\"notify\",\"data\":";
    out += '"' + JsonEscape(json) + "\"}";
    impl_->Broadcast(out);
}

void WebBridge::publishCheckStandResult(const std::string &json)
{
    std::string out = "{\"type\":\"check_stand_result\",\"data\":";
    out += '"' + JsonEscape(json) + "\"}";
    impl_->Broadcast(out);
}

void WebBridge::publishPosture(const std::string &json)
{ impl_->Broadcast("{\"type\":\"posture\",\"data\":" + json + "}"); }

void WebBridge::publishCheckStandingResult(const std::string &json)
{
    std::string out = "{\"type\":\"check_standing_result\",\"data\":";
    out += '"' + JsonEscape(json) + "\"}";
    impl_->Broadcast(out);
}

void WebBridge::broadcastFrame(const std::string &frame)
{ impl_->Broadcast(frame); }
