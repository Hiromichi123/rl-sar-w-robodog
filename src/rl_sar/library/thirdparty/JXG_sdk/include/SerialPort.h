#ifndef SERIAL_PORT_H
#define SERIAL_PORT_H

#include <termios.h>
#include <sys/select.h>
#include <errno.h>
#include <string>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/serial.h>
#include <unistd.h>
#include <iostream>
#include <memory>
#include <chrono>
#include <queue>
#include <atomic>

inline void print_data(const uint8_t* data, uint8_t len)
{
  for (int i = 0; i < len; i++)
  {
    printf("%02x ", data[i]);
  }
  printf("\n");
}

class SerialPort
{
public:
  using SharedPtr = std::shared_ptr<SerialPort>;

  SerialPort(std::string port, speed_t baudrate, int timeout_ms = 2)
    : stopped_(false)
  {
    set_timeout(timeout_ms);
    Init(port, baudrate);
  }

  ~SerialPort()
  {
    close(fd_);
  }

  // 停止接收操作
  void stop()
  {
    stopped_.store(true);
  }

  // 判断串口是否打开
  bool isOpen() const {
      return fd_ >= 0;
  }

  // 判断是否已停止
  bool isStopped() const {
      return stopped_.load();
  }
ssize_t send(const uint8_t* data, size_t len)
{
    if (fd_ < 0) return -1;
    ssize_t ret = ::write(fd_, data, len);
    if (ret < 0) {
        perror("[SerialPort] send 失败"); // 打印系统错误原因（如权限、设备断开）
    } else if (ret != (ssize_t)len) {
        std::cerr << "[SerialPort] 发送不完整，预期 " << len << "，实际 " << ret << std::endl;
    }
    return ret;
}
  ssize_t recv(uint8_t* data, size_t len)
  {
    if (fd_ < 0) { usleep(1000); return 0; }
    // VMIN=1/VTIME=1 已在 Init 设置:read 阻塞到至少 1 字节或 100ms 超时。
    // 有数据时立即返回(延迟<1ms),无数据时最多等 100ms(便于 stopped_ 检查退出)。
    // 相比 select(2ms) 轮询,唤醒频率从 500Hz 降到 10Hz,大幅降低 RT 调度开销。
    return ::read(fd_, data, len);
  }
ssize_t recv(uint8_t* data, uint8_t head, ssize_t len)
{
    if (!data || len <= 0) return 0;

    while (!stopped_.load())  // 检查停止标志，避免死循环
    {
        // 1️⃣ 从底层串口读取新数据（阻塞）
        ssize_t recv_len = this->recv(recv_buf.data(), len);  // 你底层的 read/recv
        if (recv_len > 0)
        {
            for (int i = 0; i < recv_len; i++)
                recv_queue.push(recv_buf[i]);
        }

        // 2️⃣ 查找帧头
        while (recv_queue.size() >= (size_t)len)
        {
            if (recv_queue.front() != head)
            {
                recv_queue.pop();  // 丢掉错误头
                continue;
            }

            // 3️⃣ 队列长度够，读取完整帧
            for (int i = 0; i < len; i++)
            {
                data[i] = recv_queue.front();
                recv_queue.pop();
            }

            // 4️⃣ 返回完整帧长度
            return len;
        }

        // 队列长度不够，继续阻塞等待下一批数据
        // 可以加一点微小延时避免 CPU 空转
        std::this_thread::sleep_for(std::chrono::microseconds(50));
    }

    return 0; // 停止标志被设置，返回0
}

  // void recv(uint8_t* data, uint8_t head, ssize_t len)
  // {
  //   // 存入队列
  //   ssize_t recv_len = this->recv(recv_buf.data(), len);
  //   for (int i = 0; i < recv_len; i++)
  //   {
  //     recv_queue.push(recv_buf[i]);
  //   }

  //   // 查找帧头
  //   while (recv_queue.size() >= len)
  //   {
  //     if(recv_queue.front() != head)
  //     {
  //       recv_queue.pop();
  //       continue;
  //     }
  //     break;
  //   }

  //   if(recv_queue.size() < len) return;

  //   // 读取数据
  //   for(int i = 0; i < len; i++)
  //   {
  //     data[i] = recv_queue.front();
  //     recv_queue.pop();
  //   }
  // }

  void set_timeout(int timeout_ms)
  {
    timeout_.tv_sec = timeout_ms / 1000;
    timeout_.tv_usec = (timeout_ms % 1000) * 1000;
  }

private:
  void Init(std::string port, speed_t baudrate)
  {
    int ret;
    // Open serial port
    fd_ = open(port.c_str(), O_RDWR | O_NOCTTY);
    if (fd_ < 0)
    {
      // 不在这里 exit():在 ROS/DDS 线程已启动后走 exit 会触发静态析构
      // 竞态导致 SEGV,掩盖真实原因。仅标记未打开,由上层(MotorManager)
      // 统一检测并干净退出。
      fprintf(stderr, "[SerialPort] 打开串口 %s 失败: %s\n", port.c_str(), strerror(errno));
      return;
    }

    // Set attributes
    struct termios option;
    memset(&option, 0, sizeof(option));
    ret = tcgetattr(fd_, &option);

    option.c_oflag = 0;
    option.c_lflag = 0;
    option.c_iflag = 0;

    cfsetispeed(&option, baudrate);
    cfsetospeed(&option, baudrate);

    option.c_cflag &= ~CSIZE;
    option.c_cflag |= CS8; // 8
    option.c_cflag &= ~PARENB; // no parity
    option.c_iflag &= ~INPCK; // no parity
    option.c_cflag &= ~CSTOPB; // 1 stop bit

    option.c_cc[VTIME] = 1;  // 阻塞读超时:1×100ms=100ms(仅无数据时生效,有数据立即返回)
    option.c_cc[VMIN] = 1;   // 至少读到 1 字节才返回;配合 VTIME 实现"有数据即唤醒,无数据最多等 100ms"
    option.c_lflag |= CBAUDEX;

    ret = tcflush(fd_, TCIFLUSH);
    ret = tcsetattr(fd_, TCSANOW, &option);
  }

  int fd_ = -1;
  fd_set rSet_;
  timeval timeout_;
  std::atomic<bool> stopped_;  // 停止标志

  std::queue<uint8_t> recv_queue;
  std::array<uint8_t, 1024> recv_buf;
};

#endif // SERIAL_PORT_H