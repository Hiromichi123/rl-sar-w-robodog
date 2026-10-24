#ifndef SOCKET_CAN_PORT_H
#define SOCKET_CAN_PORT_H

#include <string>
#include <cstring>
#include <cerrno>
#include <iostream>
#include <memory>
#include <atomic>
#include <chrono>

#include <unistd.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <net/if.h>
#include <linux/can.h>

// SocketCAN 传输端口（candleLight/gs_usb 适配器 → canX 网络接口）
// 与 SerialPort.h 对应：usbtocan 变体走串口 HDSC 0x55AA 帧，本变体直接收发 can_frame，
// MIT 载荷 8 字节格式与串口变体完全一致，仅传输封装不同。
// 防护沿用旧 socketcan 变体经验：
//   - SO_SNDTIMEO=2ms：TX 队列满时不无限阻塞腿线程；
//   - 错误日志限频 ≥1s：避免故障时刷屏拖垮日志。
class SocketCANPort
{
public:
  using SharedPtr = std::shared_ptr<SocketCANPort>;

  // 打开失败不 exit(-1)，isOpen() 返回 false，由 motor_manager::init() 跳过该设备
  explicit SocketCANPort(const std::string& iface, int rcv_timeout_ms = 20)
    : iface_(iface), stopped_(false), last_err_log_ms_(0)
  {
    fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (fd_ < 0) {
      std::cerr << "[SocketCANPort] socket(PF_CAN) 创建失败: " << strerror(errno) << std::endl;
      return;
    }

    // 发送超时 2ms（下游无 ACK/总线故障时防止阻塞）
    timeval snd_tv;
    snd_tv.tv_sec = 0;
    snd_tv.tv_usec = 2000;
    setsockopt(fd_, SOL_SOCKET, SO_SNDTIMEO, &snd_tv, sizeof(snd_tv));

    // 接收超时：rx 线程可周期性检查 stop 标志退出
    timeval rcv_tv;
    rcv_tv.tv_sec = rcv_timeout_ms / 1000;
    rcv_tv.tv_usec = (rcv_timeout_ms % 1000) * 1000;
    setsockopt(fd_, SOL_SOCKET, SO_RCVTIMEO, &rcv_tv, sizeof(rcv_tv));

    ifreq ifr;
    std::memset(&ifr, 0, sizeof(ifr));
    std::strncpy(ifr.ifr_name, iface_.c_str(), IFNAMSIZ - 1);
    if (ioctl(fd_, SIOCGIFINDEX, &ifr) < 0) {
      std::cerr << "[SocketCANPort] 接口 " << iface_ << " 不存在: " << strerror(errno)
                << "（请检查适配器连接，并运行 scripts/can_setup.sh 与 udev 规则）" << std::endl;
      close(fd_);
      fd_ = -1;
      return;
    }

    sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
      std::cerr << "[SocketCANPort] bind " << iface_ << " 失败: " << strerror(errno) << std::endl;
      close(fd_);
      fd_ = -1;
      return;
    }

    std::cout << "[SocketCANPort] " << iface_ << " 打开成功 (fd=" << fd_ << ")" << std::endl;
  }

  ~SocketCANPort()
  {
    if (fd_ >= 0) close(fd_);
  }

  void stop() { stopped_.store(true); }
  bool isOpen() const { return fd_ >= 0; }
  bool isStopped() const { return stopped_.load(); }

  // 发送一帧标准数据帧；返回 sizeof(can_frame) 表示成功，<0 失败
  ssize_t sendCan(uint32_t can_id, const uint8_t data[8])
  {
    if (fd_ < 0) return -1;
    can_frame frame;
    std::memset(&frame, 0, sizeof(frame));
    frame.can_id = can_id;
    frame.can_dlc = 8;
    std::memcpy(frame.data, data, 8);

    ssize_t ret = ::write(fd_, &frame, sizeof(frame));
    if (ret < 0) {
      log_send_error(errno);
    }
    return ret;
  }

  // 接收一帧；成功返回 sizeof(can_frame) 并填充 can_id/data，超时或已停止返回 0
  ssize_t recvCan(uint32_t& can_id, uint8_t data[8])
  {
    if (fd_ < 0 || stopped_.load()) return 0;
    can_frame frame;
    ssize_t ret = ::read(fd_, &frame, sizeof(frame));
    if (ret == static_cast<ssize_t>(sizeof(frame))) {
      can_id = frame.can_id & CAN_EFF_MASK;  // 去掉 EFF/RTR/ERR 标志位
      std::memcpy(data, frame.data, 8);
      return ret;
    }
    return 0;  // 超时（原始 CAN 套接字读写按帧原子，正常不会部分读）
  }

private:
  void log_send_error(int err)
  {
    int64_t now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    if (now_ms - last_err_log_ms_ < 1000) return;  // 限频 ≥1s
    last_err_log_ms_ = now_ms;
    if (err == ENOBUFS)
      std::cerr << "[SocketCANPort] " << iface_ << " 发送失败 ENOBUFS: TX 队列满（下游无 ACK/总线故障）" << std::endl;
    else if (err == ENXIO)
      std::cerr << "[SocketCANPort] " << iface_ << " 发送失败 ENXIO: 接口已 down" << std::endl;
    else
      std::cerr << "[SocketCANPort] " << iface_ << " 发送失败: " << strerror(err) << std::endl;
  }

  int fd_ = -1;
  std::string iface_;
  std::atomic<bool> stopped_;
  int64_t last_err_log_ms_;
};

#endif // SOCKET_CAN_PORT_H
