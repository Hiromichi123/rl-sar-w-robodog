// ============================================================================
// bms_ble.cpp —— 电池 BMS 蓝牙直读实现(BlueZ D-Bus + sd-bus)
// 详见 bms_ble.hpp 顶部说明。
// ============================================================================
#include "bms_ble.hpp"

#include <poll.h>
#include <systemd/sd-bus.h>

#include <atomic>
#include <chrono>
#include <cstring>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace bms_ble
{
namespace
{
// ── 目标设备与协议常量 ─────────────────────────────────────────────────────
constexpr const char *kTargetName = "HCYPZY26HYDA922100";
constexpr const char *kTargetAddr = "F5:CB:00:17:36:38";  // Static Random,可能变化
constexpr const char *kAdapterPath = "/org/bluez/hci0";
constexpr const char *kWriteUuid = "00002760-08c2-11e1-9073-0e8ac72e0001";
constexpr const char *kNotifyUuid = "00002760-08c2-11e1-9073-0e8ac72e0002";

constexpr int kPollMs = 2000;          // 轮询周期
constexpr int kNotifyTimeoutMs = 600;  // 等待 notify 响应超时(协议要求 500ms)
constexpr int kReconnectMs = 3000;     // 连接失败重试间隔
constexpr int kDiscoverMs = 9000;      // 扫描定位超时

// ── CRC16-CCITT / XMODEM(多项式 0x1021,初值 0x0000,低字节在前) ────────────
unsigned short crc16(const unsigned char *d, size_t n)
{
    unsigned short crc = 0x0000;
    for (size_t i = 0; i < n; ++i) {
        crc ^= static_cast<unsigned short>(d[i] << 8);
        for (int b = 0; b < 8; ++b)
            crc = (crc & 0x8000) ? static_cast<unsigned short>((crc << 1) ^ 0x1021)
                                 : static_cast<unsigned short>(crc << 1);
    }
    return crc;
}

// 构造读寄存器请求: [0x16,0x03,start_hi,start_lo,count_hi,count_lo,crc_lo,crc_hi]
std::vector<unsigned char> BuildReadFrame(unsigned short start, unsigned short count)
{
    std::vector<unsigned char> f(6);
    f[0] = 0x16;  // 从机地址
    f[1] = 0x03;  // 读保持寄存器
    f[2] = static_cast<unsigned char>(start >> 8);
    f[3] = static_cast<unsigned char>(start & 0xFF);
    f[4] = static_cast<unsigned char>(count >> 8);
    f[5] = static_cast<unsigned char>(count & 0xFF);
    unsigned short c = crc16(f.data(), f.size());
    f.push_back(static_cast<unsigned char>(c & 0xFF));         // 低字节在前
    f.push_back(static_cast<unsigned char>((c >> 8) & 0xFF));
    return f;
}

// ── 解析后的电池数据(缓存供前端查询) ──────────────────────────────────────
struct Data
{
    bool online = false;
    std::string msg = "未启动";
    long long ts = 0;  // epoch ms
    int cells = 0;
    double voltage = 0;    // V
    double current = 0;    // A(正=充电)
    double remain_ah = 0;  // Ah
    double fcc_ah = 0;     // Ah
    double dc_ah = 0;      // Ah
    int soc = -1, soh = -1, cycles = -1;
    int alarm = 0, pack = 0;
    double mos = 0, t1 = 0, t2 = 0, t3 = 0;  // °C
    std::vector<int> cell_mv;                // mV
    std::string fw;
};

// ── 全局状态 ───────────────────────────────────────────────────────────────
std::atomic<bool> g_run{false};
std::atomic<bool> g_enabled{true};  // 后台常驻轮询(默认开启;可用 bms off 临时省电关闭)
std::thread g_thread;
std::mutex g_mu;
Data g_data;
Publish g_pub;

long long NowMs()
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

std::string JsonEsc(const std::string &s)
{
    std::string o;
    for (char ch : s) {
        if (ch == '"' || ch == '\\') { o += '\\'; o += ch; }
        else if (ch == '\n') o += "\\n";
        else o += ch;
    }
    return o;
}

void Pub(const std::string &frame)
{
    if (g_pub) g_pub(frame);
}

void PublishStatus(bool online, const std::string &msg)
{
    {
        std::lock_guard<std::mutex> lk(g_mu);
        g_data.online = online;
        g_data.msg = msg;
    }
    std::ostringstream os;
    os << "{\"type\":\"bms_status\",\"data\":{\"online\":" << (online ? "true" : "false")
       << ",\"msg\":\"" << JsonEsc(msg) << "\"}}";
    Pub(os.str());
}

void PublishData()
{
    Data d;
    {
        std::lock_guard<std::mutex> lk(g_mu);
        d = g_data;
    }
    if (!d.online && d.ts == 0) {  // 尚无任何数据:仅发状态
        std::ostringstream os;
        os << "{\"type\":\"bms_status\",\"data\":{\"online\":false,\"msg\":\""
           << JsonEsc(d.msg) << "\"}}";
        Pub(os.str());
        return;
    }
    std::ostringstream os;
    os << "{\"type\":\"bms_data\",\"data\":{"
       << "\"online\":" << (d.online ? "true" : "false")
       << ",\"msg\":\"" << JsonEsc(d.msg) << "\""
       << ",\"ts\":" << d.ts
       << ",\"soc\":" << d.soc << ",\"soh\":" << d.soh
       << ",\"voltage\":" << d.voltage
       << ",\"current\":" << d.current
       << ",\"remain_ah\":" << d.remain_ah
       << ",\"fcc_ah\":" << d.fcc_ah
       << ",\"dc_ah\":" << d.dc_ah
       << ",\"cycles\":" << d.cycles
       << ",\"mos\":" << d.mos << ",\"t1\":" << d.t1 << ",\"t2\":" << d.t2
       << ",\"t3\":" << d.t3
       << ",\"alarm\":" << d.alarm << ",\"pack\":" << d.pack
       << ",\"cells\":" << d.cells
       << ",\"fw\":\"" << JsonEsc(d.fw) << "\""
       << ",\"cell_mv\":[";
    for (size_t i = 0; i < d.cell_mv.size(); ++i) {
        if (i) os << ",";
        os << d.cell_mv[i];
    }
    os << "]}}";
    Pub(os.str());
}

// ── sd-bus BlueZ 客户端 ────────────────────────────────────────────────────
struct Ble
{
    sd_bus *bus = nullptr;
    sd_bus_slot *match_slot = nullptr;
    std::string devpath;
    std::string write_char, notify_char;
    bool notify_started = false;
    bool is_connected = false;
    std::string err_msg;  // 最近一次连接失败原因(BlueZ 错误文本)
    std::vector<unsigned char> last_val;
    int val_seq = 0;

    bool Open()
    {
        if (bus) return true;
        int r = sd_bus_default_system(&bus);
        if (r < 0) { bus = nullptr; return false; }
        sd_bus_set_method_call_timeout(bus, 30000000ull);  // 30s(连接 BMS 可能较慢)
        return true;
    }

    void Close()
    {
        if (match_slot) { sd_bus_slot_unref(match_slot); match_slot = nullptr; }
        if (bus) { sd_bus_flush_close_unref(bus); bus = nullptr; }
        notify_started = false;
        is_connected = false;
        write_char.clear();
        notify_char.clear();
    }

    // 读取字符串属性
    bool GetStr(const char *path, const char *iface, const char *prop, std::string &out)
    {
        sd_bus_error err = SD_BUS_ERROR_NULL;
        sd_bus_message *m = nullptr;
        int r = sd_bus_call_method(bus, "org.bluez", path, "org.freedesktop.DBus.Properties",
                                   "Get", &err, &m, "ss", iface, prop);
        if (r >= 0) {
            sd_bus_message_enter_container(m, 'v', "s");
            const char *v = nullptr;
            if (sd_bus_message_read(m, "s", &v) >= 0 && v) out = v;
            sd_bus_message_exit_container(m);
            r = 0;
        }
        if (m) sd_bus_message_unref(m);
        sd_bus_error_free(&err);
        return r >= 0;
    }

    bool GetBool(const char *path, const char *iface, const char *prop, bool &out)
    {
        sd_bus_error err = SD_BUS_ERROR_NULL;
        sd_bus_message *m = nullptr;
        int r = sd_bus_call_method(bus, "org.bluez", path, "org.freedesktop.DBus.Properties",
                                   "Get", &err, &m, "ss", iface, prop);
        if (r >= 0) {
            sd_bus_message_enter_container(m, 'v', "b");
            int b = 0;
            if (sd_bus_message_read(m, "b", &b) >= 0) out = (b != 0);
            sd_bus_message_exit_container(m);
            r = 0;
        }
        if (m) sd_bus_message_unref(m);
        sd_bus_error_free(&err);
        return r >= 0;
    }

    bool GetStrArray(const char *path, const char *iface, const char *prop,
                     std::vector<std::string> &out)
    {
        sd_bus_error err = SD_BUS_ERROR_NULL;
        sd_bus_message *m = nullptr;
        int r = sd_bus_call_method(bus, "org.bluez", path, "org.freedesktop.DBus.Properties",
                                   "Get", &err, &m, "ss", iface, prop);
        if (r >= 0) {
            sd_bus_message_enter_container(m, 'v', "as");
            sd_bus_message_enter_container(m, 'a', "s");
            const char *s = nullptr;
            while (sd_bus_message_read(m, "s", &s) > 0 && s) out.push_back(s);
            sd_bus_message_exit_container(m);
            sd_bus_message_exit_container(m);
            r = 0;
        }
        if (m) sd_bus_message_unref(m);
        sd_bus_error_free(&err);
        return r >= 0;
    }

    // 读取特征值(部分设备 0002 可读;用于通知失效时的兜底)
    bool ReadValue(const std::string &path, std::vector<unsigned char> &out)
    {
        sd_bus_message *m = nullptr;
        if (sd_bus_message_new_method_call(bus, &m, "org.bluez", path.c_str(),
                                           "org.bluez.GattCharacteristic1", "ReadValue") < 0)
            return false;
        sd_bus_message_open_container(m, 'a', "{sv}");
        sd_bus_message_close_container(m);
        sd_bus_error err = SD_BUS_ERROR_NULL;
        sd_bus_message *reply = nullptr;
        int r = sd_bus_call(bus, m, 3000000ull, &err, &reply);
        if (r >= 0 && reply) {
            const void *p = nullptr;
            size_t n = 0;
            if (sd_bus_message_read_array(reply, 'y', &p, &n) >= 0 && p)
                out.assign(static_cast<const unsigned char *>(p),
                           static_cast<const unsigned char *>(p) + n);
        }
        if (reply) sd_bus_message_unref(reply);
        sd_bus_message_unref(m);
        sd_bus_error_free(&err);
        return !out.empty();
    }

    bool DeviceExists()
    {
        std::string a;
        return !devpath.empty() &&
               GetStr(devpath.c_str(), "org.bluez.Device1", "Address", a);
    }

    // 重新解析设备对象路径:优先按名称匹配,其次按已知地址
    // (BMS 使用 Static Random 地址,可能变化;缓存路径会因设备重广播而失效)
    bool ResolveDevicePath()
    {
        std::string p = FindObjectPath("org.bluez.Device1", "Name", kTargetName, true);
        if (!p.empty()) { devpath = p; return true; }
        std::string a = kTargetAddr;
        for (char &c : a) if (c == ':') c = '_';
        std::string ap = std::string(kAdapterPath) + "/dev_" + a;
        std::string tmp;
        if (GetStr(ap.c_str(), "org.bluez.Device1", "Address", tmp)) { devpath = ap; return true; }
        devpath.clear();
        return false;
    }

    // 遍历 ObjectManager,返回首个满足 iface.prop == want(或前缀匹配)的对象路径
    std::string FindObjectPath(const char *iface, const char *prop, const std::string &want,
                              bool prefix)
    {
        sd_bus_error err = SD_BUS_ERROR_NULL;
        sd_bus_message *m = nullptr;
        int r = sd_bus_call_method(bus, "org.bluez", "/", "org.freedesktop.DBus.ObjectManager",
                                   "GetManagedObjects", &err, &m, nullptr);
        if (r < 0) { if (m) sd_bus_message_unref(m); sd_bus_error_free(&err); return ""; }

        std::string found;
        r = sd_bus_message_enter_container(m, 'a', "{oa{sa{sv}}}");
        while (r > 0 && sd_bus_message_enter_container(m, 'e', "oa{sa{sv}}") > 0) {
            const char *opath = nullptr;
            sd_bus_message_read(m, "o", &opath);
            std::string path = opath ? opath : "";
            sd_bus_message_enter_container(m, 'a', "{sa{sv}}");
            while (sd_bus_message_enter_container(m, 'e', "sa{sv}") > 0) {
                const char *ifn = nullptr;
                sd_bus_message_read(m, "s", &ifn);
                bool hit = ifn && !std::strcmp(ifn, iface);
                std::string val;
                sd_bus_message_enter_container(m, 'a', "{sv}");
                while (sd_bus_message_enter_container(m, 'e', "sv") > 0) {
                    const char *key = nullptr;
                    sd_bus_message_read(m, "s", &key);
                    if (hit && key && !std::strcmp(key, prop)) {
                        sd_bus_message_enter_container(m, 'v', "s");
                        const char *v = nullptr;
                        if (sd_bus_message_read(m, "s", &v) >= 0 && v) val = v;
                        sd_bus_message_exit_container(m);
                    } else {
                        sd_bus_message_skip(m, "v");
                    }
                    sd_bus_message_exit_container(m);  // 退出 dict entry
                }
                sd_bus_message_exit_container(m);  // 退出 a{sv}
                sd_bus_message_exit_container(m);  // 退出 sa{sv} entry
                if (hit && !val.empty()) {
                    bool match = prefix ? (val.rfind(want, 0) == 0) : (val == want);
                    if (match) { found = path; break; }
                }
            }
            sd_bus_message_exit_container(m);  // 退出 a{sa{sv}}
            sd_bus_message_exit_container(m);  // 退出 oa{sa{sv}} entry
            if (!found.empty()) break;
        }
        sd_bus_message_exit_container(m);
        sd_bus_message_unref(m);
        sd_bus_error_free(&err);
        return found;
    }

    // 启动扫描定位设备(设备不在缓存中时)
    bool Discover()
    {
        devpath.clear();  // 丢弃可能失效的缓存路径,结果以本次扫描为准
        // 清理可能残留的 discovery,避免 InProgress
        sd_bus_error e = SD_BUS_ERROR_NULL;
        sd_bus_message *mm = nullptr;
        sd_bus_call_method(bus, "org.bluez", kAdapterPath, "org.bluez.Adapter1",
                           "StopDiscovery", &e, &mm, nullptr);
        if (mm) sd_bus_message_unref(mm);
        sd_bus_error_free(&e);

        e = SD_BUS_ERROR_NULL;
        mm = nullptr;
        int r = sd_bus_call_method(bus, "org.bluez", kAdapterPath, "org.bluez.Adapter1",
                                   "StartDiscovery", &e, &mm, nullptr);
        if (mm) sd_bus_message_unref(mm);
        sd_bus_error_free(&e);
        if (r < 0) return false;

        bool ok = false;
        long long t0 = NowMs();
        while (g_run.load() && g_enabled.load() && NowMs() - t0 < kDiscoverMs) {
            std::string p = FindObjectPath("org.bluez.Device1", "Name", kTargetName, true);
            if (!p.empty()) { devpath = p; ok = true; break; }
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        // 停止扫描(未连接前释放射频)
        sd_bus_error e2 = SD_BUS_ERROR_NULL;
        sd_bus_message *m2 = nullptr;
        sd_bus_call_method(bus, "org.bluez", kAdapterPath, "org.bluez.Adapter1",
                           "StopDiscovery", &e2, &m2, nullptr);
        if (m2) sd_bus_message_unref(m2);
        sd_bus_error_free(&e2);
        return ok;
    }

    bool Connect()
    {
        sd_bus_error err = SD_BUS_ERROR_NULL;
        sd_bus_message *m = nullptr;
        int r = sd_bus_call_method(bus, "org.bluez", devpath.c_str(), "org.bluez.Device1",
                                   "Connect", &err, &m, nullptr);
        if (r < 0 && err.message) err_msg = err.message;
        else if (r >= 0) err_msg.clear();
        if (m) sd_bus_message_unref(m);
        sd_bus_error_free(&err);
        // 等待 Connected=true(最长 ~6s;退出时提前中断)
        long long t0 = NowMs();
        while (NowMs() - t0 < 6000 && g_run.load()) {
            bool c = false;
            if (GetBool(devpath.c_str(), "org.bluez.Device1", "Connected", c) && c) {
                is_connected = true;
                return true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(300));
        }
        return false;
    }

    void Disconnect()
    {
        if (bus && !devpath.empty() && is_connected) {
            sd_bus_error err = SD_BUS_ERROR_NULL;
            sd_bus_message *m = nullptr;
            sd_bus_call_method(bus, "org.bluez", devpath.c_str(), "org.bluez.Device1",
                               "Disconnect", &err, &m, nullptr);
            if (m) sd_bus_message_unref(m);
            sd_bus_error_free(&err);
        }
        if (match_slot) { sd_bus_slot_unref(match_slot); match_slot = nullptr; }
        notify_started = false;
        is_connected = false;
        write_char.clear();
        notify_char.clear();
    }

    static int OnPropsChanged(sd_bus_message *m, void *ud, sd_bus_error *)
    {
        Ble *self = static_cast<Ble *>(ud);
        const char *iface = nullptr;
        sd_bus_message_read(m, "s", &iface);
        if (!sd_bus_message_enter_container(m, 'a', "{sv}")) return 0;
        while (sd_bus_message_enter_container(m, 'e', "sv") > 0) {
            const char *key = nullptr;
            sd_bus_message_read(m, "s", &key);
            if (key && !std::strcmp(key, "Value")) {
                sd_bus_message_enter_container(m, 'v', "ay");
                const void *p = nullptr;
                size_t n = 0;
                if (sd_bus_message_read_array(m, 'y', &p, &n) >= 0 && p) {
                    self->last_val.assign(static_cast<const unsigned char *>(p),
                                          static_cast<const unsigned char *>(p) + n);
                    self->val_seq++;
                }
                sd_bus_message_exit_container(m);
            } else {
                sd_bus_message_skip(m, "v");
            }
            sd_bus_message_exit_container(m);
        }
        sd_bus_message_exit_container(m);
        return 0;
    }

    bool ResolveChars()
    {
        write_char = FindObjectPath("org.bluez.GattCharacteristic1", "UUID", kWriteUuid, false);
        notify_char = FindObjectPath("org.bluez.GattCharacteristic1", "UUID", kNotifyUuid, false);
        // 诊断:打印两个特征的 Flags(确认 Notify/Write 能力)
        std::vector<std::string> wf, nf;
        if (!write_char.empty()) GetStrArray(write_char.c_str(), "org.bluez.GattCharacteristic1",
                                             "Flags", wf);
        if (!notify_char.empty()) GetStrArray(notify_char.c_str(), "org.bluez.GattCharacteristic1",
                                              "Flags", nf);
        auto join = [](const std::vector<std::string> &v) {
            std::string s;
            for (size_t i = 0; i < v.size(); ++i) s += (i ? "," : "") + v[i];
            return s;
        };
        std::cout << "[bms_ble] write_char=" << write_char << " flags=[" << join(wf)
                  << "] notify_char=" << notify_char << " flags=[" << join(nf) << "]" << std::endl;
        return !write_char.empty() && !notify_char.empty();
    }

    bool StartNotify()
    {
        if (notify_started) return true;
        sd_bus_error err = SD_BUS_ERROR_NULL;
        sd_bus_message *m = nullptr;
        int r = sd_bus_call_method(bus, "org.bluez", notify_char.c_str(),
                                   "org.bluez.GattCharacteristic1", "StartNotify", &err, &m,
                                   nullptr);
        if (m) sd_bus_message_unref(m);
        sd_bus_error_free(&err);
        if (r < 0) return false;
        std::string match = std::string("type='signal',sender='org.bluez',"
                                        "interface='org.freedesktop.DBus.Properties',"
                                        "member='PropertiesChanged',path='") +
                            notify_char + "'";
        r = sd_bus_add_match(bus, &match_slot, match.c_str(), OnPropsChanged, this);
        if (r < 0) return false;
        notify_started = true;
        return true;
    }

    bool Write(const std::vector<unsigned char> &v, bool with_response)
    {
        sd_bus_message *m = nullptr;
        if (sd_bus_message_new_method_call(bus, &m, "org.bluez", write_char.c_str(),
                                           "org.bluez.GattCharacteristic1",
                                           "WriteValue") < 0)
            return false;
        sd_bus_message_append_array(m, 'y', v.data(), v.size());
        sd_bus_message_open_container(m, 'a', "{sv}");
        sd_bus_message_open_container(m, 'e', "sv");
        sd_bus_message_append(m, "s", "Type");
        sd_bus_message_open_container(m, 'v', "s");
        sd_bus_message_append(m, "s", with_response ? "request" : "command");
        sd_bus_message_close_container(m);
        sd_bus_message_close_container(m);
        sd_bus_message_close_container(m);
        sd_bus_error err = SD_BUS_ERROR_NULL;
        sd_bus_message *reply = nullptr;
        int r = sd_bus_call(bus, m, 3000000ull, &err, &reply);
        if (reply) sd_bus_message_unref(reply);
        sd_bus_message_unref(m);
        sd_bus_error_free(&err);
        return r >= 0;
    }

    // 处理事件直到收到新的 notify 值或超时
    bool WaitValue(int timeout_ms, std::vector<unsigned char> &out)
    {
        int base = val_seq;
        long long t0 = NowMs();
        for (;;) {
            int r = sd_bus_process(bus, nullptr);
            if (r < 0) return false;
            if (val_seq != base) { out = last_val; return true; }
            long long el = NowMs() - t0;
            if (el >= timeout_ms) return false;
            if (r == 0) {
                struct pollfd pfd;
                pfd.fd = sd_bus_get_fd(bus);
                pfd.events = static_cast<short>(sd_bus_get_events(bus));
                pfd.revents = 0;
                int t = static_cast<int>(timeout_ms - el);
                if (t > 50) t = 50;
                if (t < 1) t = 1;
                ::poll(&pfd, 1, t);
            }
        }
    }
};

// ── 帧解析 → Data ──────────────────────────────────────────────────────────
// 响应: [addr,func,start_hi,start_lo,bytecount,data...,crc_lo,crc_hi]
// 数据自索引 5 起,寄存器偏移 = 寄存器号 * 2(大端)
bool ParseFrame(const std::vector<unsigned char> &f, Data &d)
{
    const size_t n = f.size();
    if (n < 7) return false;
    unsigned short got = static_cast<unsigned short>(f[n - 2] | (f[n - 1] << 8));
    if (crc16(f.data(), n - 2) != got) return false;
    if (f[1] & 0x80) return false;  // 异常码
    size_t bc = f[4];
    if (n < 5 + bc) return false;

    const unsigned char *p = f.data() + 5;
    auto rd16 = [&](size_t off) -> unsigned int {
        return (static_cast<unsigned int>(p[off]) << 8) | p[off + 1];
    };
    auto rd16s = [&](size_t off) -> int {
        unsigned int v = rd16(off);
        return (v & 0x8000) ? static_cast<int>(v) - 0x10000 : static_cast<int>(v);
    };
    auto rd32 = [&](size_t off) -> unsigned int {
        return (static_cast<unsigned int>(p[off]) << 24) |
               (static_cast<unsigned int>(p[off + 1]) << 16) |
               (static_cast<unsigned int>(p[off + 2]) << 8) | p[off + 3];
    };
    auto rd32s = [&](size_t off) -> int { return static_cast<int>(rd32(off)); };
    auto has = [&](size_t off, size_t len) -> bool { return off + len <= bc; };

    if (!has(0x40, 2)) return false;  // 至少要有总压

    d.mos = rd16s(0x00) / 100.0;
    d.t1 = rd16s(0x02) / 100.0;
    d.t2 = rd16s(0x04) / 100.0;
    d.t3 = rd16s(0x06) / 100.0;
    d.voltage = rd32(0x08) / 1000.0;       // mV → V
    d.current = rd32s(0x0C) / 1000.0;      // mA → A
    d.remain_ah = rd32(0x10) / 1000.0;     // mAh → Ah
    if (has(0x1C, 2)) d.alarm = static_cast<int>(rd16(0x1C));
    if (has(0x1E, 2)) d.pack = static_cast<int>(rd16(0x1E));
    if (has(0x26, 2)) {
        unsigned int fv = rd16(0x26);
        d.fw = std::to_string((fv >> 8) & 0xFF) + "." + std::to_string(fv & 0xFF);
    }
    if (has(0x28, 2)) {
        unsigned int v = rd16(0x28);
        d.soh = static_cast<int>((v >> 8) & 0xFF);
        d.soc = static_cast<int>(v & 0xFF);
    }
    if (has(0x30, 2)) d.cells = static_cast<int>((rd16(0x30) >> 8) & 0xFF);
    if (has(0x34, 2)) d.cycles = static_cast<int>(rd16(0x34));
    if (has(0x36, 2)) d.fcc_ah = rd16(0x36) / 100.0;  // 寄存器单位 0.01Ah
    if (has(0x38, 2)) d.dc_ah = rd16(0x38) / 100.0;

    d.cell_mv.clear();
    int ncells = d.cells > 0 ? d.cells : 0;
    if (ncells > 32) ncells = 32;
    for (int i = 0; i < ncells; ++i) {
        size_t off = 0x40 + static_cast<size_t>(i) * 2;
        if (!has(off, 2)) break;
        d.cell_mv.push_back(static_cast<int>(rd16(off)));
    }
    return true;
}

// ── 轮询线程 ───────────────────────────────────────────────────────────────
void ThreadLoop()
{
    Ble ble;
    auto now_ms = [] {
        return static_cast<long long>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch())
                .count());
    };
    std::string last_status;
    auto pub_status = [&](bool online, const std::string &m) {
        if (m == last_status) return;  // 去重:失败时避免每轮刷屏
        last_status = m;
        std::cout << "[bms_ble] " << m << std::endl;
        PublishStatus(online, m);
    };
    bool force_discover = true;  // 强制先扫描定位,避免复用失效的设备路径
    bool was_enabled = g_enabled.load();
    while (g_run.load()) {
        const bool enabled = g_enabled.load();
        if (enabled && !was_enabled) force_discover = true;  // off→on:重新扫描(留时间给 BMS 恢复广播)
        was_enabled = enabled;
        if (!enabled) {
            if (ble.bus) {
                // 必须显式 Disconnect 断开 BLE 链路,否则 BMS 仍认为被占用,不会重新广播
                ble.Disconnect();
                ble.Close();
                pub_status(false, "已停止(bms off)");
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(300));
            continue;
        }
        if (!ble.Open()) {
            pub_status(false, "无法连接系统 D-Bus(BlueZ)");
            std::this_thread::sleep_for(std::chrono::milliseconds(2000));
            continue;
        }
        if (ble.devpath.empty()) ble.ResolveDevicePath();

        // 建立连接
        if (!ble.is_connected) {
            // 每次都重新解析设备路径(Static Random 地址可能变化,缓存路径会失效)
            if (force_discover || !ble.ResolveDevicePath()) {
                pub_status(false, "搜索 BMS 设备…");
                ble.Discover();
                force_discover = false;
            }
            if (ble.devpath.empty() || !ble.DeviceExists()) {
                pub_status(false, "未发现 BMS 设备");
                std::this_thread::sleep_for(std::chrono::milliseconds(kReconnectMs));
                continue;
            }
            pub_status(false, "正在连接 BMS…");
            bool gatt_ok = false;
            if (ble.Connect()) {
                // GATT 服务解析可能滞后于连接,重试等待特征出现
                for (int i = 0; i < 6 && g_run.load(); ++i) {
                    if (ble.ResolveChars()) { gatt_ok = ble.StartNotify(); break; }
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                }
            }
            if (!gatt_ok) {
                std::string why;
                if (ble.is_connected) why = "GATT 特征未就绪(稍后重试)";
                else if (!ble.err_msg.empty()) why = "连接失败: " + ble.err_msg;
                else why = "连接失败(蓝牙链路超时?)";
                pub_status(false, why);
                ble.Disconnect();
                force_discover = true;  // 下次先重新扫描再连,避免再次干等 Connect 超时
                std::this_thread::sleep_for(std::chrono::milliseconds(kReconnectMs));
                continue;
            }
            last_status.clear();
            PublishStatus(true, "已连接");
        }

        // 轮询一帧
        std::vector<unsigned char> req = BuildReadFrame(0x0000, 0x0040);
        std::vector<unsigned char> resp;
        bool got = false;
        if (!ble.Write(req, false)) {
            pub_status(false, "写入失败,正在重连…");
            ble.Disconnect();
            force_discover = true;
            std::this_thread::sleep_for(std::chrono::milliseconds(kReconnectMs));
            continue;
        }
        got = ble.WaitValue(kNotifyTimeoutMs, resp);
        if (!got) {
            // 通知未到达:兜底直接读特征值(部分设备 notify 特征亦支持 read)
            got = ble.ReadValue(ble.notify_char, resp) || ble.ReadValue(ble.write_char, resp);
        }
        if (got) {
            Data d;
            if (ParseFrame(resp, d)) {
                d.online = true;
                d.msg = "OK";
                d.ts = now_ms();
                {
                    std::lock_guard<std::mutex> lk(g_mu);
                    g_data = d;
                }
                last_status.clear();
                PublishData();
            } else {
                std::ostringstream hs;
                for (size_t i = 0; i < resp.size() && i < 24; ++i)
                    hs << std::hex << (int)resp[i] << " ";
                std::cout << "[bms_ble] 响应解析失败 len=" << resp.size() << " head=" << hs.str()
                          << std::endl;
                pub_status(true, "已连接,响应解析失败");
            }
        } else {
            pub_status(true, "已连接,响应超时");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(kPollMs));
    }
    ble.Disconnect();  // 进程退出时同样断开链路,避免残留占用导致 BMS 不广播
    ble.Close();
}
}  // namespace

void Start(Publish publish)
{
    if (g_run.load()) return;
    g_pub = std::move(publish);
    g_run.store(true);
    g_thread = std::thread(ThreadLoop);
}

void Stop()
{
    if (!g_run.load()) return;
    g_run.store(false);
    if (g_thread.joinable()) g_thread.join();
    g_pub = nullptr;
}

Snapshot GetSnapshot()
{
    Snapshot s;
    std::lock_guard<std::mutex> lk(g_mu);
    s.online  = g_data.online;
    s.soc     = g_data.soc;
    s.voltage = g_data.voltage;
    s.current = g_data.current;
    s.alarm   = g_data.alarm;
    s.ts      = g_data.ts;
    return s;
}

bool HandleCommand(const std::string &text, const Publish &publish)
{
    if (text != "bms" && text.rfind("bms ", 0) != 0) return false;
    std::string op = text.size() > 4 ? text.substr(4) : "";
    // 去首尾空白
    size_t a = op.find_first_not_of(" \t\r\n");
    size_t b = op.find_last_not_of(" \t\r\n");
    op = (a == std::string::npos) ? "" : op.substr(a, b - a + 1);

    auto reply = [&](const std::string &frame) {
        if (publish) publish(frame);
        else Pub(frame);
    };

    if (op == "on" || op == "open") {
        g_enabled.store(true);
        std::ostringstream os;
        os << "{\"type\":\"bms_status\",\"data\":{\"online\":false,\"msg\":\"已开启,正在连接…\"}}";
        reply(os.str());
        return true;
    }
    if (op == "off" || op == "close") {
        g_enabled.store(false);
        std::ostringstream os;
        os << "{\"type\":\"bms_status\",\"data\":{\"online\":false,\"msg\":\"已停止\"}}";
        reply(os.str());
        return true;
    }
    if (op == "info" || op == "" || op == "get") {
        Data d;
        {
            std::lock_guard<std::mutex> lk(g_mu);
            d = g_data;
        }
        if (d.ts == 0) {
            std::ostringstream os;
            os << "{\"type\":\"bms_status\",\"data\":{\"online\":" << (d.online ? "true" : "false")
               << ",\"msg\":\"" << JsonEsc(d.msg) << "\"}}";
            reply(os.str());
        } else {
            PublishData();
        }
        return true;
    }
    std::ostringstream os;
    os << "{\"type\":\"bms_status\",\"data\":{\"online\":false,\"msg\":\"未知 bms 子命令: "
       << JsonEsc(op) << "\"}}";
    reply(os.str());
    return true;
}
}  // namespace bms_ble
