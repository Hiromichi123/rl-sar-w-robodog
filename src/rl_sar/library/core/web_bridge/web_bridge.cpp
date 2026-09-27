#include "web_bridge.hpp"

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <signal.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/sendfile.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cctype>
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

// ============================================================================
// 最小 JSON 工具（与 rl_real_JXG.cpp 中 JsonEscape 同风格，无第三方库）
// ============================================================================
static std::string JsonEscape(const std::string &s)
{
    std::string out;
    out.reserve(s.size() + 8);
    for (char c : s) {
        switch (c) {
            case '\\': out += "\\\\"; break;
            case '"':  out += "\\\""; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default:   out += c; break;
        }
    }
    return out;
}

// 从 JSON 文本中提取 "key":"value" 的字符串 value（处理 \\" \\\\ 最小转义）
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
            if (n == 'u' && p + 5 < json.size()) {
                // 最小 UTF-16 处理：仅透传原转义（场景名一般不含），退化为跳过
                out += '?'; p += 6; continue;
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
    uint64_t len = 0;
    uint8_t buf[64] = {};
    size_t buf_len = 0;

    static uint32_t rol(uint32_t v, int s) { return (v << s) | (v >> (32 - s)); }

    void block(const uint8_t *p)
    {
        uint32_t w[80];
        for (int i = 0; i < 16; ++i)
            w[i] = (uint32_t(p[i * 4]) << 24) | (uint32_t(p[i * 4 + 1]) << 16) |
                   (uint32_t(p[i * 4 + 2]) << 8) | uint32_t(p[i * 4 + 3]);
        for (int i = 16; i < 80; ++i) w[i] = rol(w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16], 1);
        uint32_t a = h[0], b = h[1], c = h[2], d = h[3], e = h[4];
        for (int i = 0; i < 80; ++i) {
            uint32_t f, k;
            if (i < 20)      { f = (b & c) | ((~b) & d);          k = 0x5A827999; }
            else if (i < 40) { f = b ^ c ^ d;                     k = 0x6ED9EBA1; }
            else if (i < 60) { f = (b & c) | (b & d) | (c & d);   k = 0x8F1BBCDC; }
            else             { f = b ^ c ^ d;                     k = 0xCA62C1D6; }
            uint32_t t = rol(a, 5) + f + e + k + w[i];
            e = d; d = c; c = rol(b, 30); b = a; a = t;
        }
        h[0] += a; h[1] += b; h[2] += c; h[3] += d; h[4] += e;
    }

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
        out += tbl[(v >> 18) & 63]; out += tbl[(v >> 12) & 63]; out += tbl[(v >> 6) & 63];
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
constexpr size_t kMaxOutBuf = 256u * 1024;           // 慢客户端写缓冲上限

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
    };
    size_t dot = path.rfind('.');
    if (dot == std::string::npos) return "application/octet-stream";
    auto it = mime.find(path.substr(dot));
    return it != mime.end() ? it->second : "application/octet-stream";
}

// 解析后的入站消息
struct Inbound
{
    enum Kind { CmdVelMsg, CommandMsg, ImuStreamMsg, NotifyAckMsg, CheckStandMsg, BriefingMsg } kind;
    CmdVel vel;
    std::string text;
    bool flag = false;
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
    StringCb on_command;
    BoolCb on_imu_stream;
    StringCb on_notify_ack;
    VoidCb on_check_stand;
    StringCb on_briefing;

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
        std::string in_buf;           // 未解析的原始输入
        std::string out_buf;          // 待发送
        // HTTP 请求缓冲
        std::string http_buf;
        bool http_done = false;
        bool close_after_flush = false;   // 静态响应发完即关
        // 静态文件流式发送(sendfile),避免大文件整体入队撑爆 out_buf
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
    };
    std::map<int, Client> clients;

    // ---- 分发线程（等价 ROS 单线程 executor）----
    std::thread dispatch_thread;
    std::mutex q_mutex;
    std::condition_variable q_cv;
    std::deque<Inbound> in_queue;
    static constexpr size_t kQueueCap = 16;

    // ---- 广播锁 ----
    std::mutex bcast_mutex;

    double NowSec() const
    {
        return std::chrono::duration<double>(
                   std::chrono::steady_clock::now().time_since_epoch()).count();
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

    void CloseClient(int fd)
    {
        auto it = clients.find(fd);
        if (it == clients.end()) return;
        ::close(fd);
        clients.erase(it);
    }

    // 发送（append 到 out_buf；慢客户端超限断开）
    void SendTo(Client &c, const std::string &data)
    {
        if (c.out_buf.size() + data.size() > kMaxOutBuf) {
            std::cout << "[web_bridge] client " << c.fd << " too slow, dropping" << std::endl;
            ::close(c.fd);
            // 延迟 erase 由调用方统一处理：这里直接标记
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

    // ------------------------------------------------------------------
    // 入站 JSON → Inbound → 队列
    // ------------------------------------------------------------------
    void HandleWsMessage(const std::string &msg)
    {
        std::string type;
        if (!JsonGetString(msg, "type", type)) return;

        Inbound in;
        if (type == "cmd_vel") {
            in.kind = Inbound::CmdVelMsg;
            JsonGetNumber(msg, "vx", in.vel.x);
            JsonGetNumber(msg, "vy", in.vel.y);
            JsonGetNumber(msg, "yaw", in.vel.yaw);
        } else if (type == "command") {
            in.kind = Inbound::CommandMsg;
            if (!JsonGetString(msg, "data", in.text)) return;
        } else if (type == "imu_stream") {
            in.kind = Inbound::ImuStreamMsg;
            if (!JsonGetBool(msg, "data", in.flag)) return;
        } else if (type == "notify_ack") {
            in.kind = Inbound::NotifyAckMsg;
            if (!JsonGetString(msg, "data", in.text)) return;
        } else if (type == "check_stand") {
            in.kind = Inbound::CheckStandMsg;
        } else if (type == "briefing") {
            in.kind = Inbound::BriefingMsg;
            if (!JsonGetString(msg, "data", in.text)) return;
        } else {
            return;  // 未知类型忽略
        }

        {
            std::lock_guard<std::mutex> lock(q_mutex);
            if (in_queue.size() >= kQueueCap) in_queue.pop_front();  // 满则丢旧
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
                case Inbound::CmdVelMsg:    if (on_cmd_vel) on_cmd_vel(in.vel); break;
                case Inbound::CommandMsg:   if (on_command) on_command(in.text); break;
                case Inbound::ImuStreamMsg: if (on_imu_stream) on_imu_stream(in.flag); break;
                case Inbound::NotifyAckMsg: if (on_notify_ack) on_notify_ack(in.text); break;
                case Inbound::CheckStandMsg:if (on_check_stand) on_check_stand(); break;
                case Inbound::BriefingMsg:  if (on_briefing) on_briefing(in.text); break;
            }
        }
    }

    // ------------------------------------------------------------------
    // HTTP / WS 握手
    // ------------------------------------------------------------------
    void HandleHttp(Client &c)
    {
        // 找到请求头结束
        size_t hdr_end = c.http_buf.find("\r\n\r\n");
        if (hdr_end == std::string::npos) {
            if (c.http_buf.size() > 8192) { ::close(c.fd); c.fd = -1; }
            return;
        }
        std::string headers = c.http_buf.substr(0, hdr_end);

        // 请求行
        std::istringstream iss(headers);
        std::string method, path, version;
        iss >> method >> path >> version;
        if (method != "GET" || path.empty()) {
            SendTo(c, "HTTP/1.1 405 Method Not Allowed\r\nContent-Length: 0\r\n\r\n");
            return;
        }

        // 提取请求头
        std::string upgrade, ws_key;
        std::string line;
        while (std::getline(iss, line)) {
            auto lower = [](std::string s) {
                std::transform(s.begin(), s.end(), s.begin(),
                               [](unsigned char ch) { return char(std::tolower(ch)); });
                return s;
            };
            size_t colon = line.find(':');
            if (colon == std::string::npos) continue;
            std::string k = lower(line.substr(0, colon));
            std::string v = line.substr(colon + 1);
            while (!v.empty() && (v.front() == ' ' || v.front() == '\t')) v.erase(v.begin());
            while (!v.empty() && (v.back() == '\r' || v.back() == ' ')) v.pop_back();
            if (k == "upgrade") upgrade = lower(v);
            if (k == "sec-websocket-key") ws_key = v;
        }

        // WebSocket 升级（路径 /ws）
        size_t qpos = path.find('?');
        std::string url_path = qpos == std::string::npos ? path : path.substr(0, qpos);
        if (url_path == "/ws") {
            if (upgrade != "websocket" || ws_key.empty()) {
                SendTo(c, "HTTP/1.1 400 Bad Request\r\nContent-Length: 0\r\n\r\n");
                return;
            }
            std::string resp =
                "HTTP/1.1 101 Switching Protocols\r\n"
                "Upgrade: websocket\r\n"
                "Connection: Upgrade\r\n"
                "Sec-WebSocket-Accept: " + WsAcceptKey(ws_key) + "\r\n\r\n";
            c.http_buf.clear();
            c.http_done = true;
            SendTo(c, resp);
            c.ws = true;
            c.last_recv = NowSec();
            std::cout << "[web_bridge] ws client connected, fd=" << c.fd
                      << ", clients=" << clients.size() << std::endl;
            return;
        }

        ServeStatic(c, url_path, qpos != std::string::npos);
    }

    void ServeStatic(Client &c, const std::string &url_path, bool has_query)
    {
        // URL 解码
        std::string dec;
        dec.reserve(url_path.size());
        for (size_t i = 0; i < url_path.size(); ++i) {
            if (url_path[i] == '%' && i + 2 < url_path.size()) {
                auto hexv = [](char ch) -> int {
                    if (ch >= '0' && ch <= '9') return ch - '0';
                    if (ch >= 'a' && ch <= 'f') return ch - 'a' + 10;
                    if (ch >= 'A' && ch <= 'F') return ch - 'A' + 10;
                    return -1;
                };
                int hi = hexv(url_path[i + 1]), lo = hexv(url_path[i + 2]);
                if (hi >= 0 && lo >= 0) { dec += char((hi << 4) | lo); i += 2; continue; }
            }
            dec += url_path[i];
        }

        if (dec.empty() || dec.back() == '/') dec += "index.html";

        // 路径穿越防护
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

        // 流式发送:打开文件后交给 NetLoop 用 sendfile 推送(大文件不入 out_buf)
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
        std::ostringstream hdr;
        hdr << "HTTP/1.1 200 OK\r\n"
            << "Content-Type: " << MimeType(resolved) << "\r\n"
            << "Content-Length: " << st.st_size << "\r\n"
            << cache << "Connection: close\r\n\r\n";
        c.http_buf.clear();
        c.http_done = true;
        SendTo(c, hdr.str());
        c.ws = false;
        // 文件体由 NetLoop 在 POLLOUT 时 sendfile,全部发完再关连接
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
                case 0x0:  // continuation
                    c.frag_buf += payload;
                    if (fin) {
                        HandleWsMessage(c.frag_buf);
                        c.frag_buf.clear();
                        c.in_frag = false;
                    }
                    break;
                case 0x1:  // text
                case 0x2:  // binary(当文本尝试解析)
                    if (fin) HandleWsMessage(payload);
                    else { c.frag_buf = payload; c.in_frag = true; }
                    break;
                case 0x8:  // close
                {
                    // 服务器端 close 帧无需 mask，发完即关
                    std::string cl;
                    cl += char(0x88); cl += char(0);
                    SendTo(c, cl);
                    ::close(c.fd);
                    c.fd = -1;
                    return;
                }
                case 0x9:  // ping → pong
                {
                    std::string pong;
                    pong += char(0x8A);
                    size_t n = payload.size();
                    if (n < 126) pong += char(n);
                    else { pong += char(126); pong += char((n >> 8) & 0xFF); pong += char(n & 0xFF); }
                    pong += payload;
                    SendTo(c, pong);
                    break;
                }
                case 0xA:  // pong
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
            if (listen_fd >= 0) pfds.push_back({listen_fd, POLLIN, 0});
            std::vector<int> ids;
            {
                std::lock_guard<std::mutex> lock(bcast_mutex);
                for (auto &kv : clients) {
                    short ev = POLLIN | ((!kv.second.out_buf.empty() || kv.second.file_fd >= 0) ? POLLOUT : 0);
                    pfds.push_back({kv.second.fd, ev, 0});
                    ids.push_back(kv.second.fd);
                }
            }
            int rc = ::poll(pfds.data(), nfds_t(pfds.size()), 100);
            if (rc < 0) {
                if (errno == EINTR) continue;
                break;
            }

            double now = NowSec();

            // accept
            for (auto &p : pfds) {
                if (p.fd == listen_fd && (p.revents & POLLIN)) {
                    while (true) {
                        int nfd = ::accept4(listen_fd, nullptr, nullptr, SOCK_NONBLOCK);
                        if (nfd < 0) break;
                        SetNonBlock(nfd);
                        std::lock_guard<std::mutex> lock(bcast_mutex);
                        Client c;
                        c.fd = nfd;
                        c.last_recv = now;
                        clients[nfd] = std::move(c);
                    }
                }
            }

            // client io
            {
                std::lock_guard<std::mutex> lock(bcast_mutex);
                for (int fd : ids) {
                    auto it = clients.find(fd);
                    if (it == clients.end()) continue;
                    Client &c = it->second;
                    if (c.fd < 0) continue;  // 已标记关闭

                    char buf[16384];
                    // read
                    while (true) {
                        ssize_t n = ::recv(c.fd, buf, sizeof(buf), 0);
                        if (n > 0) {
                            if (c.ws) c.in_buf.append(buf, size_t(n));
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
                        if (!c.ws && c.http_buf.size() > 8192) { ::close(c.fd); c.fd = -1; break; }
                        if (c.ws && c.in_buf.size() > kMaxFramePayload * 2) { ::close(c.fd); c.fd = -1; break; }
                    }
                    if (c.fd < 0) continue;

                    // 处理
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
                    // 流式发送静态文件体(sendfile);响应头已先进入 out_buf
                    while (c.fd >= 0 && c.file_fd >= 0 && c.out_buf.empty() &&
                           c.file_off < c.file_size) {
                        ssize_t n = ::sendfile(c.fd, c.file_fd, &c.file_off,
                                               size_t(c.file_size - c.file_off));
                        if (n > 0) continue;
                        if (n == 0) break;  // 理论上不会发生
                        if (errno == EAGAIN || errno == EWOULDBLOCK) break;  // 等下次 POLLOUT
                        ::close(c.file_fd); c.file_fd = -1;
                        ::close(c.fd); c.fd = -1;
                        break;
                    }
                    if (c.fd >= 0 && c.file_fd >= 0 && c.file_off >= c.file_size) {
                        ::close(c.file_fd);
                        c.file_fd = -1;
                    }
                    if (c.fd >= 0 && c.out_buf.empty() && c.file_fd < 0 && c.close_after_flush) {
                        ::close(c.fd);
                        c.fd = -1;
                    }
                }
                // 清理已关闭(同时关闭可能残留的文件 fd)
                for (auto it = clients.begin(); it != clients.end();) {
                    if (it->second.fd < 0) {
                        if (it->second.file_fd >= 0) { ::close(it->second.file_fd); it->second.file_fd = -1; }
                        it = clients.erase(it);
                    } else ++it;
                }
                // 心跳：空闲 30s 发 ping；60s 无响应断开
                for (auto &kv : clients) {
                    Client &c = kv.second;
                    if (!c.ws) continue;
                    if (now - c.last_recv > 60.0) { ::close(c.fd); c.fd = -1; continue; }
                    if (now - c.last_recv > 30.0 && !c.ping_outstanding) {
                        std::string ping;
                        ping += char(0x82); ping += char(0);  // ping, 空 payload
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
WebBridge::~WebBridge() { stop(); delete impl_; }

bool WebBridge::start(int port, const std::string &web_root,
                      CmdVelCb on_cmd_vel, StringCb on_command, BoolCb on_imu_stream,
                      StringCb on_notify_ack, VoidCb on_check_stand, StringCb on_briefing)
{
    impl_->port = port;
    impl_->web_root = web_root;
    impl_->on_cmd_vel = std::move(on_cmd_vel);
    impl_->on_command = std::move(on_command);
    impl_->on_imu_stream = std::move(on_imu_stream);
    impl_->on_notify_ack = std::move(on_notify_ack);
    impl_->on_check_stand = std::move(on_check_stand);
    impl_->on_briefing = std::move(on_briefing);

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
    return true;
}

void WebBridge::stop()
{
    if (!impl_->running.exchange(false)) {
        if (impl_->listen_fd >= 0) { ::close(impl_->listen_fd); impl_->listen_fd = -1; }
        return;
    }
    if (impl_->listen_fd >= 0) { ::close(impl_->listen_fd); impl_->listen_fd = -1; }
    {
        std::lock_guard<std::mutex> lock(impl_->bcast_mutex);
        for (auto &kv : impl_->clients) ::close(kv.first);
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
    // json 已是合法 JSON 字符串内容：作为字符串值需要转义
    out += '"' + JsonEscape(json) + "\"}";
    impl_->Broadcast(out);
}

void WebBridge::publishCheckStandResult(const std::string &json)
{
    std::string out = "{\"type\":\"check_stand_result\",\"data\":";
    out += '"' + JsonEscape(json) + "\"}";
    impl_->Broadcast(out);
}

void WebBridge::publishBriefingStatus(const std::string &text)
{ impl_->PublishString("briefing_status", text); }

size_t WebBridge::clientCount() const
{
    std::lock_guard<std::mutex> lock(impl_->bcast_mutex);
    size_t n = 0;
    for (auto &kv : impl_->clients) n += (kv.second.ws && kv.second.fd >= 0) ? 1 : 0;
    return n;
}
