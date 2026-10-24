#ifndef CAN_SETUP_H
#define CAN_SETUP_H

// ============================================================================
// CanSetup.h — C++ 版 SocketCAN 接口配置（替代 scripts/can_setup.sh）
//
// 功能与 can_setup.sh 完全对齐：
//   阶段1: 按 USB 序列号前缀识别 candleLight/gs_usb 通道 → 目标接口名
//   阶段2: 两阶段改名（避免命名冲突），收敛为 can0~can4
//   阶段3: 配置 bitrate / restart-ms / txqueuelen，接口 UP
//
// 依赖：linux/netlink.h、linux/rtnetlink.h、linux/can/netlink.h
// 权限：需要 root（服务以 root 运行，天然满足）
//
// 序列号尾段 4348570220313938 为五路共用，前 8 位区分通道：
//   004B001F=can0(FL)  004B003C=can1(FR)  004C0026=can2(RL)
//   004C0029=can3(RR)  004B0041=can4(备用)
// ============================================================================

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cerrno>
#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <algorithm>
#include <unistd.h>
#include <dirent.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <net/if.h>
#include <linux/netlink.h>
#include <linux/rtnetlink.h>
#include <linux/can/netlink.h>

class CanSetup
{
public:
  // 通道序列号前缀 → 目标接口名（函数内局部 static，避免头文件多重定义）
  static const std::map<std::string, std::string>& GetSer2Name()
  {
    static const std::map<std::string, std::string> m = {
      {"004B001F", "can0"},  // FL
      {"004B003C", "can1"},  // FR
      {"004C0026", "can2"},  // RL
      {"004C0029", "can3"},  // RR
      {"004B0041", "can4"},  // 备用
    };
    return m;
  }
  // 固定顺序（FL FR RL RR 备用）
  static const std::vector<std::string>& GetOrder()
  {
    static const std::vector<std::string> v = {
      "004B001F", "004B003C", "004C0026", "004C0029", "004B0041"
    };
    return v;
  }

  // 执行完整配置；返回成功配置的接口数
  // 调用前确保已 root；失败仅打印日志，不 exit（由调用方决定策略）
  static int Run(uint32_t bitrate = 1000000)
  {
    std::cout << "[CanSetup] 开始 C++ SocketCAN 接口配置..." << std::endl;

    // 阶段1: 识别现有 can* 接口的序列号 → 当前名/目标名
    std::map<std::string, IfaceInfo> ifaces = scanCanInterfaces();

    // 兜底：未识别到五路盒序列号，但 can0~can3 已存在（udev/旧配置已命名正确）
    if (ifaces.empty()) {
      bool has_any = false;
      for (int i = 0; i <= 4; ++i) {
        std::string name = "can" + std::to_string(i);
        if (ifaceExists(name)) has_any = true;
      }
      if (has_any) {
        std::cout << "[CanSetup] 未匹配到预设序列号，检测到已有 can* 接口，跳过改名直接配置" << std::endl;
      } else {
        std::cerr << "[CanSetup] 未发现任何 can* 接口" << std::endl;
        return 0;
      }
    } else {
      // 阶段2: 改名（两阶段腾挪，避免冲突）
      renameInterfaces(ifaces);
    }

    // 阶段3: 配置参数并 UP（目标接口或现有 can0~can3）
    int ok_count = 0;
    for (const auto& key : GetOrder()) {
      auto it = GetSer2Name().find(key);
      if (it == GetSer2Name().end()) continue;
      const std::string& target = it->second;
      if (!ifaceExists(target)) {
        std::cerr << "[CanSetup] 接口 " << target << " 不存在，跳过" << std::endl;
        continue;
      }
      if (configureInterface(target, bitrate)) {
        ok_count++;
      }
    }

    printStatus();
    std::cout << "[CanSetup] 配置完成: " << ok_count << "/" << GetOrder().size() << " 路就绪" << std::endl;
    return ok_count;
  }

private:
  struct IfaceInfo {
    std::string cur_name;    // 当前接口名
    std::string serial;      // USB 序列号
    std::string target_name; // 目标名（空串表示非五路盒通道）
    std::string port;        // USB 物理端口（日志用）
  };

  // ── 阶段1: 扫描 /sys/class/net，识别 can* 接口的 USB 序列号 ──
  static std::map<std::string, IfaceInfo> scanCanInterfaces()
  {
    std::map<std::string, IfaceInfo> result;
    DIR* dir = opendir("/sys/class/net");
    if (!dir) {
      std::cerr << "[CanSetup] 无法打开 /sys/class/net: " << strerror(errno) << std::endl;
      return result;
    }

    struct dirent* ent;
    while ((ent = readdir(dir)) != nullptr) {
      std::string name = ent->d_name;
      if (name.compare(0, 3, "can") != 0) continue;

      std::string serial = readSerial(name);
      if (serial.empty()) continue;

      std::string key = serial.substr(0, 8);
      auto it = GetSer2Name().find(key);
      if (it == GetSer2Name().end()) continue;  // 非五路盒通道

      IfaceInfo info;
      info.cur_name = name;
      info.serial = serial;
      info.target_name = it->second;
      info.port = readPort(name);
      result[key] = info;

      std::cout << "[CanSetup] 识别 " << name << " -> " << info.target_name
                << " (序列号 " << key << ", 端口 " << info.port << ")" << std::endl;
    }
    closedir(dir);
    return result;
  }

  // ── 阶段2: 两阶段改名 ──
  static void renameInterfaces(std::map<std::string, IfaceInfo>& ifaces)
  {
    // 先把需要改名的接口改为中转名
    for (auto& [key, info] : ifaces) {
      if (info.cur_name == info.target_name) {
        std::cout << "[CanSetup] " << info.cur_name << " 命名正确" << std::endl;
        continue;
      }
      setIfaceDown(info.cur_name);
      std::string mid = "mig_" + key;
      if (renameIface(info.cur_name, mid)) {
        std::cout << "[CanSetup] " << info.cur_name << " -> " << mid << " (中转)" << std::endl;
        info.cur_name = mid;
      } else {
        std::cerr << "[CanSetup] " << info.cur_name << " 中转改名失败" << std::endl;
      }
    }

    // 再从中转名改为最终名
    for (auto& [key, info] : ifaces) {
      if (info.cur_name == info.target_name) continue;
      if (renameIface(info.cur_name, info.target_name)) {
        std::cout << "[CanSetup] " << info.cur_name << " -> " << info.target_name << std::endl;
      } else {
        std::cerr << "[CanSetup] " << info.cur_name << " 改名失败" << std::endl;
      }
    }
  }

  // ── 阶段3: 配置单个接口（bitrate / restart-ms / txqueuelen / UP）──
  static bool configureInterface(const std::string& name, uint32_t bitrate)
  {
    setIfaceDown(name);

    int fd = socket(PF_NETLINK, SOCK_RAW, NETLINK_ROUTE);
    if (fd < 0) {
      std::cerr << "[CanSetup] netlink socket 创建失败: " << strerror(errno) << std::endl;
      return false;
    }

    // 获取接口索引
    ifreq ifr;
    std::memset(&ifr, 0, sizeof(ifr));
    std::strncpy(ifr.ifr_name, name.c_str(), IFNAMSIZ - 1);
    if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
      std::cerr << "[CanSetup] " << name << " 获取索引失败: " << strerror(errno) << std::endl;
      close(fd);
      return false;
    }
    int ifindex = ifr.ifr_ifindex;

    // 构造 netlink 请求：RTM_NEWLINK + CAN 属性（嵌套在 IFLA_LINKINFO 中）
    struct {
      struct nlmsghdr  n;
      struct ifinfomsg i;
      char             buf[1024];
    } req;
    std::memset(&req, 0, sizeof(req));

    req.n.nlmsg_len = NLMSG_LENGTH(sizeof(struct ifinfomsg));
    req.n.nlmsg_flags = NLM_F_REQUEST | NLM_F_ACK;
    req.n.nlmsg_type = RTM_NEWLINK;
    req.i.ifi_family = AF_UNSPEC;
    req.i.ifi_index = ifindex;

    // IFLA_LINKINFO 嵌套
    struct rtattr* linkinfo = startNest(&req.n, IFLA_LINKINFO);

    // IFLA_INFO_KIND = "can"
    const char* kind = "can";
    addAttr(&req.n, IFLA_INFO_KIND, kind, strlen(kind) + 1);

    // IFLA_INFO_DATA 嵌套
    struct rtattr* infodata = startNest(&req.n, IFLA_INFO_DATA);

    // IFLA_CAN_BITTIMING
    struct can_bittiming bt;
    std::memset(&bt, 0, sizeof(bt));
    bt.bitrate = bitrate;
    bt.sample_point = 750;  // 0.750 = 75.0%
    addAttr(&req.n, IFLA_CAN_BITTIMING, &bt, sizeof(bt));

    // IFLA_CAN_RESTART_MS
    uint32_t restart_ms = 100;
    addAttr(&req.n, IFLA_CAN_RESTART_MS, &restart_ms, sizeof(restart_ms));

    // 闭合嵌套
    endNest(&req.n, infodata);
    endNest(&req.n, linkinfo);

    // 发送并等待 ACK
    if (!sendNetlink(fd, &req.n)) {
      std::cerr << "[CanSetup] " << name << " 配置 bitrate 失败" << std::endl;
      close(fd);
      return false;
    }

    // txqueuelen（通过 ioctl 更直接）
    ifr.ifr_qlen = 100;
    if (ioctl(fd, SIOCSIFTXQLEN, &ifr) < 0) {
      std::cerr << "[CanSetup] " << name << " 设置 txqueuelen 失败: " << strerror(errno) << std::endl;
      // 非致命，继续
    }

    close(fd);

    // UP
    if (!setIfaceUp(name)) {
      std::cerr << "[CanSetup] " << name << " UP 失败" << std::endl;
      return false;
    }

    std::cout << "[CanSetup] " << name << " 配置完成: bitrate=" << bitrate
              << " restart-ms=100 txqueuelen=100 UP" << std::endl;
    return true;
  }

  // ── 工具函数 ──

  // 从 /sys/class/net/<iface>/device 向上找 serial 属性
  static std::string readSerial(const std::string& iface)
  {
    char path[512];
    std::snprintf(path, sizeof(path), "/sys/class/net/%s/device", iface.c_str());

    char resolved[512];
    if (!realpath(path, resolved)) return "";

    std::string dir = resolved;
    while (dir != "/" && !dir.empty()) {
      std::string serial_path = dir + "/serial";
      int fd = open(serial_path.c_str(), O_RDONLY);
      if (fd >= 0) {
        char buf[128] = {0};
        ssize_t n = read(fd, buf, sizeof(buf) - 1);
        close(fd);
        if (n > 0) {
          // 去掉尾部换行/空格
          while (n > 0 && (buf[n-1] == '\n' || buf[n-1] == ' ')) n--;
          return std::string(buf, n);
        }
      }
      // 向上一级
      size_t pos = dir.find_last_of('/');
      if (pos == std::string::npos || pos == 0) break;
      dir = dir.substr(0, pos);
    }
    return "";
  }

  // 读 USB 物理端口（用于日志）
  static std::string readPort(const std::string& iface)
  {
    char path[512];
    std::snprintf(path, sizeof(path), "/sys/class/net/%s/device", iface.c_str());
    char resolved[512];
    if (!realpath(path, resolved)) return "";
    std::string dir = resolved;
    size_t pos = dir.find_last_of('/');
    if (pos == std::string::npos) return "";
    return dir.substr(pos + 1);
  }

  // 检查接口是否存在
  static bool ifaceExists(const std::string& name)
  {
    return if_nametoindex(name.c_str()) != 0;
  }

  // 设置接口 UP
  static bool setIfaceUp(const std::string& name)
  {
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) return false;
    ifreq ifr;
    std::memset(&ifr, 0, sizeof(ifr));
    std::strncpy(ifr.ifr_name, name.c_str(), IFNAMSIZ - 1);
    ifr.ifr_flags = IFF_UP;
    bool ok = (ioctl(fd, SIOCSIFFLAGS, &ifr) == 0);
    close(fd);
    return ok;
  }

  // 设置接口 DOWN
  static bool setIfaceDown(const std::string& name)
  {
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) return false;
    ifreq ifr;
    std::memset(&ifr, 0, sizeof(ifr));
    std::strncpy(ifr.ifr_name, name.c_str(), IFNAMSIZ - 1);
    if (ioctl(fd, SIOCGIFFLAGS, &ifr) < 0) { close(fd); return false; }
    ifr.ifr_flags &= ~IFF_UP;
    bool ok = (ioctl(fd, SIOCSIFFLAGS, &ifr) == 0);
    close(fd);
    return ok;
  }

  // 接口改名（netlink RTM_NEWLINK + IFLA_IFNAME）
  static bool renameIface(const std::string& old_name, const std::string& new_name)
  {
    int fd = socket(PF_NETLINK, SOCK_RAW, NETLINK_ROUTE);
    if (fd < 0) return false;

    // 获取旧接口索引
    ifreq ifr;
    std::memset(&ifr, 0, sizeof(ifr));
    std::strncpy(ifr.ifr_name, old_name.c_str(), IFNAMSIZ - 1);
    if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0) { close(fd); return false; }

    struct {
      struct nlmsghdr  n;
      struct ifinfomsg i;
      char             buf[256];
    } req;
    std::memset(&req, 0, sizeof(req));

    req.n.nlmsg_len = NLMSG_LENGTH(sizeof(struct ifinfomsg));
    req.n.nlmsg_flags = NLM_F_REQUEST | NLM_F_ACK;
    req.n.nlmsg_type = RTM_NEWLINK;
    req.i.ifi_family = AF_UNSPEC;
    req.i.ifi_index = ifr.ifr_ifindex;

    addAttr(&req.n, IFLA_IFNAME, new_name.c_str(), new_name.length() + 1);

    bool ok = sendNetlink(fd, &req.n);
    close(fd);
    return ok;
  }

  // 添加 netlink 属性
  static struct rtattr* addAttr(struct nlmsghdr* n, int type, const void* data, size_t alen)
  {
    size_t len = RTA_LENGTH(alen);
    size_t newlen = NLMSG_ALIGN(n->nlmsg_len) + RTA_ALIGN(len);
    struct rtattr* rta = reinterpret_cast<struct rtattr*>(
        reinterpret_cast<char*>(n) + NLMSG_ALIGN(n->nlmsg_len));
    rta->rta_type = type;
    rta->rta_len = len;
    if (alen > 0 && data) std::memcpy(RTA_DATA(rta), data, alen);
    n->nlmsg_len = newlen;
    return rta;
  }

  // 开始嵌套属性（返回嵌套起始指针，供 endNest 闭合）
  static struct rtattr* startNest(struct nlmsghdr* n, int type)
  {
    struct rtattr* nest = addAttr(n, type, nullptr, 0);
    return nest;
  }

  // 闭合嵌套属性
  static void endNest(struct nlmsghdr* n, struct rtattr* nest)
  {
    nest->rta_len = reinterpret_cast<char*>(n) + n->nlmsg_len -
                    reinterpret_cast<char*>(nest);
  }

  // 发送 netlink 消息并等待 ACK
  static bool sendNetlink(int fd, struct nlmsghdr* n)
  {
    struct sockaddr_nl nladdr;
    std::memset(&nladdr, 0, sizeof(nladdr));
    nladdr.nl_family = AF_NETLINK;

    struct iovec iov = { n, n->nlmsg_len };
    struct msghdr msg = { &nladdr, sizeof(nladdr), &iov, 1, nullptr, 0, 0 };

    if (sendmsg(fd, &msg, 0) < 0) return false;

    // 等待 ACK
    char buf[4096];
    ssize_t len = recv(fd, buf, sizeof(buf), 0);
    if (len < 0) return false;

    for (struct nlmsghdr* nh = reinterpret_cast<struct nlmsghdr*>(buf);
         NLMSG_OK(nh, len); nh = NLMSG_NEXT(nh, len)) {
      if (nh->nlmsg_type == NLMSG_ERROR) {
        struct nlmsgerr* err = reinterpret_cast<struct nlmsgerr*>(NLMSG_DATA(nh));
        return (err->error == 0);
      }
    }
    return false;
  }

  // 打印当前 can* 接口状态
  static void printStatus()
  {
    std::cout << "[CanSetup] ---- 接口状态 ----" << std::endl;
    for (const auto& key : GetOrder()) {
      auto it = GetSer2Name().find(key);
      if (it == GetSer2Name().end()) continue;
      const std::string& name = it->second;
      if (!ifaceExists(name)) continue;

      int fd = socket(AF_INET, SOCK_DGRAM, 0);
      if (fd < 0) continue;
      ifreq ifr;
      std::memset(&ifr, 0, sizeof(ifr));
      std::strncpy(ifr.ifr_name, name.c_str(), IFNAMSIZ - 1);
      if (ioctl(fd, SIOCGIFFLAGS, &ifr) == 0) {
        bool up = (ifr.ifr_flags & IFF_UP) != 0;
        std::cout << "[CanSetup]   " << name << ": " << (up ? "UP" : "DOWN") << std::endl;
      }
      close(fd);
    }
  }
};

#endif // CAN_SETUP_H
