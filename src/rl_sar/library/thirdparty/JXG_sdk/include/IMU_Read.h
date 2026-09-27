#ifndef IMU_READ_H
#define IMU_READ_H

#include <stdint.h>
#include <stdbool.h>
#include <Eigen/Core>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ====================== DM-IMU USB (CDC ACM) 协议常量 =====================
//
// USB 帧格式（手册 V1.2 第5页）：
//   [0]    0x55              帧头
//   [1]    0xAA              帧头
//   [2]    ID                机器 ID（与 CAN can_id 一致，默认 0x01）
//   [3]    数据类型           01=加速度 02=角速度 03=欧拉角 04=四元数
//   [4..]  float32 LE ×N     数据（3 个或 4 个 float）
//   [...-2] CRC16 LE         校验 (CCITT 0x1021, init=0xFFFF)
//   [末]   0x0A              帧尾
//
// 3-float 帧（RID 01/02/03）: 2+1+1+12+2+1 = 19 字节
// 4-float 帧（RID 04 四元数）: 2+1+1+16+2+1 = 23 字节

#define DM_FRAME_HEAD_0    0x55
#define DM_FRAME_HEAD_1    0xAA
#define DM_FRAME_TAIL      0x0A

#define DM_FRAME_LEN_3F    19     // 3 个 float 的帧（加速度/角速度/欧拉角）
#define DM_FRAME_LEN_4F    23     // 4 个 float 的帧（四元数）

#define DM_RID_ACCEL       0x01   // 加速度 (m/s²)      3×float = 19字节
#define DM_RID_GYRO        0x02   // 角速度 (rad/s)     3×float = 19字节
#define DM_RID_EULER       0x03   // 欧拉角 (度)        3×float = 19字节
#define DM_RID_QUAT        0x04   // 四元数 (w,x,y,z)   4×float = 23字节


#define DM_IMU_PORT        "/dev/ttyimu"
#define DM_IMU_BAUD        921600

// ====================== DM-IMU 数据缓存 ======================
typedef struct {
    // 欧拉角（弧度，从 DM-IMU 度数转换）
    float roll  = 0.0f;
    float pitch = 0.0f;
    float yaw   = 0.0f;

    // 陀螺仪 (rad/s)
    float gyro[3] = {0.0f, 0.0f, 0.0f};

    // 加速度计 (m/s²)
    float accel[3] = {0.0f, 0.0f, 0.0f};

    // 四元数 [w, x, y, z]（DM-IMU 直接输出，无需从欧拉角转换）
    float quat[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    bool  quat_valid = false;  // 是否收到过四元数帧

    // 陀螺零偏（open_imu 时静止自检测得），getAngleVelocity() 会自动扣除
    float gyro_bias[3] = {0.0f, 0.0f, 0.0f};
    bool  gyro_bias_valid = false;

    // 零偏补偿 + 静止冻结(ZUPT)的 yaw 积分值（rad，上电为 0，相对航向）
    float fused_yaw = 0.0f;
    bool  fused_yaw_init = false;
    long  gyro_frames = 0;  // 已收到的 gyro 帧计数（校准采样用）
} DM_IMU_Data;

extern int         fd_serial;
extern DM_IMU_Data dm_imu_data;

// ====================== 串口操作 ======================
int          serialOpen(const char* device, int baudrate);
int          serialDataAvail(int fd);
unsigned char serialGetchar(int fd);
void         serialClose(int fd);

// ====================== IMU 操作（接口兼容原 N100） ======================
void open_imu(void);
void read_imu(void);
void close_imu(void);

// ====================== 数据获取（接口兼容原 N100） ======================
Eigen::Matrix<float, 1, 3> getAngle();          // [roll, pitch, yaw] (rad)
Eigen::Matrix<float, 1, 3> getAngleVelocity();  // [wx, wy, wz] (rad/s)，已扣除零偏
Eigen::Matrix<float, 1, 3> get_com_Acc();       // [ax, ay, az] (m/s²)
bool getQuaternion(float q[4]);                  // [w, x, y, z]，返回是否有效

// ====================== yaw 稳定化 ======================
// 启动时静止采样陀螺零偏（约 duration_ms 毫秒，机器人必须静止）。
// 方差过大（启动时被搬动）则自动放弃，不应用零偏。
void calibrate_gyro_bias(int duration_ms = 2000);
// 零偏补偿 + 静止冻结的相对 yaw（rad，上电为 0）
float getYawFused();
void resetFusedYaw();

#endif // IMU_READ_H
