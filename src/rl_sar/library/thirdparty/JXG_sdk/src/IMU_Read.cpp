#include "IMU_Read.h"
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>

// ====================== 全局变量 ======================
int         fd_serial = -1;
DM_IMU_Data dm_imu_data;

// 接收缓冲区
static uint8_t rx_buf[512];
static int    rx_buf_len = 0;

// ====================== CRC16 CCITT (0x1021, init=0xFFFF) ======================
static const uint16_t CRC16_TABLE[256] = {
    0x0000,0x1021,0x2042,0x3063,0x4084,0x50A5,0x60C6,0x70E7,
    0x8108,0x9129,0xA14A,0xB16B,0xC18C,0xD1AD,0xE1CE,0xF1EF,
    0x1231,0x0210,0x3273,0x2252,0x52B5,0x4294,0x72F7,0x62D6,
    0x9339,0x8318,0xB37B,0xA35A,0xD3BD,0xC39C,0xF3FF,0xE3DE,
    0x2462,0x3443,0x0420,0x1401,0x64E6,0x74C7,0x44A4,0x5485,
    0xA56A,0xB54B,0x8528,0x9509,0xE5EE,0xF5CF,0xC5AC,0xD58D,
    0x3653,0x2672,0x1611,0x0630,0x76D7,0x66F6,0x5695,0x46B4,
    0xB75B,0xA77A,0x9719,0x8738,0xF7DF,0xE7FE,0xD79D,0xC7BC,
    0x48C4,0x58E5,0x6886,0x78A7,0x0840,0x1861,0x2802,0x3823,
    0xC9CC,0xD9ED,0xE98E,0xF9AF,0x8948,0x9969,0xA90A,0xB92B,
    0x5AF5,0x4AD4,0x7AB7,0x6A96,0x1A71,0x0A50,0x3A33,0x2A12,
    0xDBFD,0xCBDC,0xFBBF,0xEB9E,0x9B79,0x8B58,0xBB3B,0xAB1A,
    0x6CA6,0x7C87,0x4CE4,0x5CC5,0x2C22,0x3C03,0x0C60,0x1C41,
    0xEDAE,0xFD8F,0xCDEC,0xDDCD,0xAD2A,0xBD0B,0x8D68,0x9D49,
    0x7E97,0x6EB6,0x5ED5,0x4EF4,0x3E13,0x2E32,0x1E51,0x0E70,
    0xFF9F,0xEFBE,0xDFDD,0xCFFC,0xBF1B,0xAF3A,0x9F59,0x8F78,
    0x9188,0x81A9,0xB1CA,0xA1EB,0xD10C,0xC12D,0xF14E,0xE16F,
    0x1080,0x00A1,0x30C2,0x20E3,0x5004,0x4025,0x7046,0x6067,
    0x83B9,0x9398,0xA3FB,0xB3DA,0xC33D,0xD31C,0xE37F,0xF35E,
    0x02B1,0x1290,0x22F3,0x32D2,0x4235,0x5214,0x6277,0x7256,
    0xB5EA,0xA5CB,0x95A8,0x8589,0xF56E,0xE54F,0xD52C,0xC50D,
    0x34E2,0x24C3,0x14A0,0x0481,0x7466,0x6447,0x5424,0x4405,
    0xA7DB,0xB7FA,0x8799,0x97B8,0xE75F,0xF77E,0xC71D,0xD73C,
    0x26D3,0x36F2,0x0691,0x16B0,0x6657,0x7676,0x4615,0x5634,
    0xD94C,0xC96D,0xF90E,0xE92F,0x99C8,0x89E9,0xB98A,0xA9AB,
    0x5844,0x4865,0x7806,0x6827,0x18C0,0x08E1,0x3882,0x28A3,
    0xCB7D,0xDB5C,0xEB3F,0xFB1E,0x8BF9,0x9BD8,0xABBB,0xBB9A,
    0x4A75,0x5A54,0x6A37,0x7A16,0x0AF1,0x1AD0,0x2AB3,0x3A92,
    0xFD2E,0xED0F,0xDD6C,0xCD4D,0xBDAA,0xAD8B,0x9DE8,0x8DC9,
    0x7C26,0x6C07,0x5C64,0x4C45,0x3CA2,0x2C83,0x1CE0,0x0CC1,
    0xEF1F,0xFF3E,0xCF5D,0xDF7C,0xAF9B,0xBFBA,0x8FD9,0x9FF8,
    0x6E17,0x7E36,0x4E55,0x5E74,0x2E93,0x3EB2,0x0ED1,0x1EF0
};

static uint16_t dm_crc16(const uint8_t* data, int len)
{
    uint16_t crc = 0xFFFF;
    for (int i = 0; i < len; i++) {
        uint8_t idx = ((crc >> 8) ^ data[i]) & 0xFF;
        crc = ((crc << 1) ^ CRC16_TABLE[idx]) & 0xFFFF;
    }
    return crc;
}

// ====================== 串口操作 ======================
static speed_t get_baudrate(int baud) {
    switch(baud) {
        case 9600:   return B9600;
        case 19200:  return B19200;
        case 38400:  return B38400;
        case 57600:  return B57600;
        case 115200: return B115200;
        case 460800: return B460800;
        case 921600: return B921600;
        default:     return B0;
    }
}

int serialOpen(const char* device, int baudrate) {
    int fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd < 0) {
        perror("[DM-IMU] 串口打开失败");
        return -1;
    }
    struct termios options;
    tcgetattr(fd, &options);
    speed_t br = get_baudrate(baudrate);
    if (br == B0) { close(fd); return -1; }
    cfsetispeed(&options, br);
    cfsetospeed(&options, br);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag &= ~CRTSCTS;
    options.c_cflag |= CLOCAL | CREAD;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_oflag &= ~OPOST;
    options.c_cc[VTIME] = 1;
    options.c_cc[VMIN] = 0;
    tcsetattr(fd, TCSANOW, &options);
    return fd;
}

int serialDataAvail(int fd) {
    int bytes = 0;
    ioctl(fd, FIONREAD, &bytes);
    return bytes;
}

unsigned char serialGetchar(int fd) {
    unsigned char c;
    while (read(fd, &c, 1) != 1);
    return c;
}

void serialClose(int fd) {
    if (fd >= 0) close(fd);
}

// ====================== DM-IMU 操作 ======================

static constexpr float DEG2RAD = (float)(M_PI / 180.0);

// 每收到一帧 gyro 调用：零偏补偿 + 静止冻结(ZUPT)的 yaw 积分（定义在文件末尾）
void UpdateFusedYaw(float gx_raw, float gy_raw, float gz_raw);

void open_imu(void) {
    // 重试打开:DM-IMU(cdc_acm) 枚举偏慢,上电后可能晚于主程序就绪。
    // 全部重试失败后不再 exit()(旧行为会拖死整个主程序导致 Web 调试台不可用),
    // 而是 优雅降级:fd_serial 保持 <=0,read_imu 已有保护,程序继续运行(无 IMU 数据)。
    constexpr int IMU_OPEN_RETRIES = 10;   // 10 次 × 1s
    for (int i = 0; i < IMU_OPEN_RETRIES; ++i) {
        fd_serial = serialOpen(DM_IMU_PORT, DM_IMU_BAUD);
        if (fd_serial > 0) break;
        printf("[DM-IMU] USB 串口 %s 打开失败(第 %d/%d 次),1s 后重试... (ls /dev/ttyACM*)\n",
               DM_IMU_PORT, i + 1, IMU_OPEN_RETRIES);
        sleep(1);
    }
    if (fd_serial > 0) {
        printf("[DM-IMU] USB 串口 %s 开启成功\n", DM_IMU_PORT);
        // 机器人此时应处于静止状态，采样陀螺零偏（失败会自动跳过）
        calibrate_gyro_bias(2000);
    } else {
        printf("[DM-IMU] USB 串口 %s 持续打开失败,降级运行(无 IMU 数据,程序继续)\n", DM_IMU_PORT);
    }
}

// 根据 RID 确定帧长
static int frame_len_for_rid(uint8_t rid) {
    return (rid == DM_RID_QUAT) ? DM_FRAME_LEN_4F : DM_FRAME_LEN_3F;
}

// CRC 校验的数据长度（不含 CRC 本身和帧尾）
// 帧总长 = data_len + 2(CRC) + 1(0A)
static int crc_data_len_for_rid(uint8_t rid) {
    return (rid == DM_RID_QUAT) ? 20 : 16;  // 23-2-1=20 或 19-2-1=16
}

void read_imu(void) {
    if (fd_serial <= 0) return;

    int avail = serialDataAvail(fd_serial);
    if (avail <= 0) return;

    // 读入缓冲区
    int to_read = std::min(avail, (int)(sizeof(rx_buf) - rx_buf_len));
    if (to_read <= 0) {
        rx_buf_len = 0;
        return;
    }
    for (int i = 0; i < to_read; i++) {
        rx_buf[rx_buf_len++] = serialGetchar(fd_serial);
    }

    // 最小帧长（3-float = 19 字节），不足则等下一轮
    const int MIN_FRAME = DM_FRAME_LEN_3F;

    int pos = 0;
    while (pos <= rx_buf_len - MIN_FRAME) {
        // 1. 查找帧头 0x55 0xAA
        if (rx_buf[pos] != DM_FRAME_HEAD_0 || rx_buf[pos + 1] != DM_FRAME_HEAD_1) {
            pos++;
            continue;
        }

        // 2. 读取 RID（byte[3]）判断帧长
        uint8_t rid = rx_buf[pos + 3];
        int frame_len = MIN_FRAME;  // 默认 19

        if (rid == DM_RID_ACCEL || rid == DM_RID_GYRO ||
            rid == DM_RID_EULER || rid == DM_RID_QUAT) {
            frame_len = frame_len_for_rid(rid);
        } else {
            // 未知 RID，跳过帧头继续找
            pos++;
            continue;
        }

        // 3. 缓冲区够不够这一帧
        if (pos + frame_len > rx_buf_len) {
            break;  // 剩余字节不够，等下一轮
        }

        // 4. 检查帧尾
        if (rx_buf[pos + frame_len - 1] != DM_FRAME_TAIL) {
            pos++;
            continue;
        }

        // 5. CRC16 校验
        int crc_len = crc_data_len_for_rid(rid);
        uint16_t crc_calc = dm_crc16(&rx_buf[pos], crc_len);
        uint16_t crc_wire = rx_buf[pos + crc_len]
                          | (uint16_t(rx_buf[pos + crc_len + 1]) << 8);
        if (crc_calc != crc_wire) {
            // 兜底：尝试跳过帧头 2 字节再算
            uint16_t alt = dm_crc16(&rx_buf[pos + 2], crc_len - 2);
            if (alt != crc_wire) {
                pos++;
                continue;
            }
        }

        // 6. 解析数据
        if (rid == DM_RID_QUAT) {
            // 四元数帧：4×float32，帧长 23
            float w, x, y, z;
            std::memcpy(&w, &rx_buf[pos + 4],  4);
            std::memcpy(&x, &rx_buf[pos + 8],  4);
            std::memcpy(&y, &rx_buf[pos + 12], 4);
            std::memcpy(&z, &rx_buf[pos + 16], 4);

            dm_imu_data.quat[0] = w;
            dm_imu_data.quat[1] = x;
            dm_imu_data.quat[2] = y;
            dm_imu_data.quat[3] = z;
            dm_imu_data.quat_valid = true;
        } else {
            // 3-float 帧
            float f1, f2, f3;
            std::memcpy(&f1, &rx_buf[pos + 4],  4);
            std::memcpy(&f2, &rx_buf[pos + 8],  4);
            std::memcpy(&f3, &rx_buf[pos + 12], 4);

            switch (rid) {
                case DM_RID_ACCEL:
                    dm_imu_data.accel[0] = f1;
                    dm_imu_data.accel[1] = f2;
                    dm_imu_data.accel[2] = f3;
                    break;
                case DM_RID_GYRO:
                    dm_imu_data.gyro[0] = f1;
                    dm_imu_data.gyro[1] = f2;
                    dm_imu_data.gyro[2] = f3;
                    dm_imu_data.gyro_frames++;
                    UpdateFusedYaw(f1, f2, f3);
                    break;
                case DM_RID_EULER:
                    // DM-IMU 输出欧拉角单位为度，转为弧度
                    // 顺序: Roll, Pitch, Yaw
                    dm_imu_data.roll  = f1 * DEG2RAD;
                    dm_imu_data.pitch = f2 * DEG2RAD;
                    dm_imu_data.yaw   = f3 * DEG2RAD;
                    break;
            }
        }

        pos += frame_len;
    }

    // 保留残余字节
    if (pos < rx_buf_len) {
        int remaining = rx_buf_len - pos;
        std::memmove(rx_buf, &rx_buf[pos], remaining);
        rx_buf_len = remaining;
    } else {
        rx_buf_len = 0;
    }
}

void close_imu() {
    serialClose(fd_serial);
    fd_serial = -1;
}

// ====================== 数据获取接口 ======================
//
// 注意：DM-IMU 的坐标系可能和机器人安装方向不同。
// 如果姿态方向反了，在对应轴加负号即可。

// ---------------- 陀螺零偏校准 + yaw 积分 ----------------
// 静止判定阈值：|gz| 低通均值低于该值认为无航向转动（rad/s ≈ 1.1°/s）
static constexpr float ZUPT_RATE_THRESH = 0.02f;
// 零偏合理性阈值：均值超 0.05 rad/s 或方差超 3e-4 视为"启动时在动"，放弃校准
static constexpr float GYRO_BIAS_MAX_ABS = 0.05f;
static constexpr float GYRO_BIAS_MAX_VAR = 3.0e-4f;

void calibrate_gyro_bias(int duration_ms) {
    dm_imu_data.gyro_bias_valid = false;

    // 等串口数据流稳定
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    long last_cnt = dm_imu_data.gyro_frames;
    double sum[3] = {0.0, 0.0, 0.0}, sumsq[3] = {0.0, 0.0, 0.0};
    long n = 0;
    auto t0 = std::chrono::steady_clock::now();
    // read_imu 尚未被 loop_imu 驱动（open_imu 在 loop 启动前调用），此处自行驱动
    while (std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() * 1000.0 < duration_ms) {
        read_imu();
        if (dm_imu_data.gyro_frames != last_cnt) {
            last_cnt = dm_imu_data.gyro_frames;
            for (int i = 0; i < 3; i++) {
                sum[i]   += dm_imu_data.gyro[i];
                sumsq[i] += (double)dm_imu_data.gyro[i] * dm_imu_data.gyro[i];
            }
            n++;
        }
    }

    if (n < 200) {
        printf("[DM-IMU] 陀螺零偏校准失败：仅采样 %ld 帧，跳过（yaw 将无零偏补偿）\n", n);
        return;
    }

    float mean[3], var[3];
    bool ok = true;
    for (int i = 0; i < 3; i++) {
        mean[i] = (float)(sum[i] / n);
        var[i]  = (float)(sumsq[i] / n - (double)mean[i] * mean[i]);
        if (var[i] < 0) var[i] = 0;
        if (std::fabs(mean[i]) > GYRO_BIAS_MAX_ABS || var[i] > GYRO_BIAS_MAX_VAR) ok = false;
    }

    if (!ok) {
        printf("[DM-IMU] 陀螺零偏校准放弃：启动时检测到运动 (|mean|>%.3f rad/s 或 var>%.1e)，跳过零偏补偿\n",
               GYRO_BIAS_MAX_ABS, GYRO_BIAS_MAX_VAR);
        return;
    }

    for (int i = 0; i < 3; i++) dm_imu_data.gyro_bias[i] = mean[i];
    dm_imu_data.gyro_bias_valid = true;
    printf("[DM-IMU] 陀螺零偏校准完成 (n=%ld): bias=[%.5f, %.5f, %.5f] rad/s\n",
           n, mean[0], mean[1], mean[2]);
}

// 每收到一帧 gyro 调用：零偏补偿 + 静止冻结(ZUPT)的 yaw 积分
static std::chrono::steady_clock::time_point s_last_gyro_time;
static float s_gz_ema = 0.0f;  // 补偿后 gz 的低通均值，用于静止判定

void UpdateFusedYaw(float gx_raw, float gy_raw, float gz_raw) {
    (void)gx_raw; (void)gy_raw;
    auto now = std::chrono::steady_clock::now();

    if (!dm_imu_data.fused_yaw_init) {
        dm_imu_data.fused_yaw = 0.0f;
        dm_imu_data.fused_yaw_init = true;
        s_last_gyro_time = now;
        s_gz_ema = 0.0f;
        return;
    }

    double dt = std::chrono::duration<double>(now - s_last_gyro_time).count();
    s_last_gyro_time = now;
    if (dt <= 0.0 || dt > 0.05) return;  // 首帧/长间断不积分

    float gz = gz_raw - (dm_imu_data.gyro_bias_valid ? dm_imu_data.gyro_bias[2] : 0.0f);

    // 低通均值判定是否在转动（采样约 900Hz，alpha=0.005 → 时间常数约 0.22s）
    const float alpha = 0.005f;
    s_gz_ema += alpha * (gz - s_gz_ema);

    if (std::fabs(s_gz_ema) > ZUPT_RATE_THRESH) {
        dm_imu_data.fused_yaw += gz * (float)dt;
    }
    // 静止时冻结 yaw，消除残余零偏导致的持续漂移
}

Eigen::Matrix<float, 1, 3> getAngle() {
    Eigen::Matrix<float, 1, 3> angle;
    angle << dm_imu_data.roll,
             dm_imu_data.pitch,
             dm_imu_data.yaw;
    return angle;
}

Eigen::Matrix<float, 1, 3> getAngleVelocity() {
    float b0 = dm_imu_data.gyro_bias_valid ? dm_imu_data.gyro_bias[0] : 0.0f;
    float b1 = dm_imu_data.gyro_bias_valid ? dm_imu_data.gyro_bias[1] : 0.0f;
    float b2 = dm_imu_data.gyro_bias_valid ? dm_imu_data.gyro_bias[2] : 0.0f;
    Eigen::Matrix<float, 1, 3> angular_vel;
    angular_vel << dm_imu_data.gyro[0] - b0,
                   dm_imu_data.gyro[1] - b1,
                   dm_imu_data.gyro[2] - b2;
    return angular_vel;
}

float getYawFused() {
    return dm_imu_data.fused_yaw;
}

void resetFusedYaw() {
    dm_imu_data.fused_yaw = 0.0f;
    dm_imu_data.fused_yaw_init = false;
    s_gz_ema = 0.0f;
}

Eigen::Matrix<float, 1, 3> get_com_Acc() {
    Eigen::Matrix<float, 1, 3> accel;
    accel << dm_imu_data.accel[0],
             dm_imu_data.accel[1],
             dm_imu_data.accel[2];
    return accel;
}

bool getQuaternion(float q[4]) {
    if (!dm_imu_data.quat_valid) return false;
    std::memcpy(q, dm_imu_data.quat, sizeof(float) * 4);
    return true;
}
