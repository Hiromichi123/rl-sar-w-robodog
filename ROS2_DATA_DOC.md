# rl_real_JXG ROS2 数据接口文档（文字版）

适用程序：`rl_real_JXG`（JXG / JXGw 机器人 RL 控制节点）
代码位置：`src/rl_sar/src/rl_real_JXG.cpp`（话题注册 L83-L100）
启用条件：编译时定义 `USE_ROS2_REAL_CONTROL`
QoS 设置：所有话题统一 `KeepLast(10) + reliable + durability_volatile`

---

## 一、总览

| 方向 | 话题名 | 消息类型 | 用途 |
|------|--------|----------|------|
| 订阅 | /rl_real/cmd_vel | geometry_msgs/Twist | 速度控制指令 |
| 订阅 | /rl_real/command | std_msgs/String | 字符串控制指令 |
| 订阅 | /rl_real/imu_stream_enable | std_msgs/Bool | IMU 数据流开关 |
| 发布 | /rl_real/feedback | std_msgs/String | 指令应答 / 状态反馈 |
| 发布 | /rl_real/motor_state | std_msgs/Float32MultiArray | 电机遥测（9 字段/电机） |
| 发布 | /rl_real/imu_state | std_msgs/Float32MultiArray | IMU 遥测（19 字段） |
| 发布 | /rl_real/pose2d | std_msgs/Float32MultiArray | 里程计位姿（7 字段） |

---

## 二、订阅话题（控制输入）

### 2.1 /rl_real/cmd_vel（geometry_msgs/Twist）

速度指令，映射到 RL 控制量：

- linear.x  -> control.x   （前进/后退速度）
- linear.y  -> control.y   （左右平移速度）
- angular.z -> control.yaw （偏航角速度）

注意：手柄接管（gamepad_override_active）时收到的 cmd_vel 被忽略，
并通过 /rl_real/feedback 返回 "cmd_vel ignored: gamepad override active"。

### 2.2 /rl_real/command（std_msgs/String）

字符串指令协议，大小写不敏感，多余空白自动去除：

| 指令 | 别名 | 动作 |
|------|------|------|
| getup | stand, up | 起身（等价键盘 Num0） |
| getdown | down, lie, sit | 趴下（等价键盘 Num9，速度清零） |
| locomotion | rl, walk, move | RL 行走模式（等价键盘 Num1） |
| passive | stop, safe_stop | 急停（等价键盘 P，速度清零） |
| vel_stop | hold | 仅速度清零，保持当前姿态 |
| nav_on | - | 打开导航模式 |
| nav_off | - | 关闭导航模式 |
| imu_on | - | 打开 IMU 数据流 |
| imu_off | - | 关闭 IMU 数据流 |
| zero | setzero, set_zero, set_zero_all, zero_all | 全部电机置零（发 0xFE） |
| quit | exit | 退出程序 |
| feedback | status | 查询当前 FSM 状态（手柄接管时仍放行） |

除 feedback/status 外，手柄接管期间其余指令均被忽略。

### 2.3 /rl_real/imu_stream_enable（std_msgs/Bool）

true：打开 DM-IMU 串口数据流；false：关闭。
等价于发送 command: imu_on / imu_off。

---

## 三、发布话题（遥测输出）

### 3.1 /rl_real/feedback（std_msgs/String）

文本反馈，内容示例：

- "cmd_vel accepted" / "cmd_vel ignored: gamepad override active"
- "command accepted: getup"
- "command accepted: set_zero_all"
- "status state=RLFSMStatePassive gamepad_override=false"
- "unknown command: xxx"

### 3.2 /rl_real/motor_state（Float32MultiArray）

每个电机 9 个 float，按 joint_mapping 顺序排列：

| 序号 | 字段 | 说明 |
|------|------|------|
| 0 | idx | 电机索引（joint_mapping[i]） |
| 1 | target_pos | 目标位置（rad，指令 q） |
| 2 | real_pos | 实际位置（rad，已加 lying_down_offset 趴下偏移） |
| 3 | target_vel | 目标速度（rad/s，指令 dq） |
| 4 | real_vel | 实际速度（rad/s） |
| 5 | target_tau | 目标力矩（N·m，指令 tau） |
| 6 | real_tau | 实际力矩（N·m） |
| 7 | kp | 位置环增益 |
| 8 | kd | 速度环增益 |

layout.dim 标注：
dim[0] = "motor"（数量 = num_of_dofs，stride = 电机数×9）
dim[1] = "field:idx,target_pos,real_pos,target_vel,real_vel,target_tau,real_tau,kp,kd"

### 3.3 /rl_real/imu_state（Float32MultiArray）

共 19 个 float：

| 序号 | 字段 | 说明 |
|------|------|------|
| 0-3 | quat_w/x/y/z | 四元数 |
| 4 | roll | 横滚角（rad） |
| 5 | pitch | 俯仰角（rad） |
| 6 | yaw | 航向角（rad） |
| 7-9 | acc_x/y/z | 加速度计（body 系） |
| 10-12 | gyro_x/y/z | 陀螺仪（body 系） |
| 13-15 | pos_north/east/down | 位置（NED 系） |
| 16-18 | vel_body_x/y/z | 速度（body 系） |

### 3.4 /rl_real/pose2d（Float32MultiArray）

共 7 个 float：

| 序号 | 字段 | 说明 |
|------|------|------|
| 0 | x | 里程计 X |
| 1 | y | 里程计 Y |
| 2 | z | 里程计 Z |
| 3 | yaw | 航向角 |
| 4 | vel_x | 体速度 X |
| 5 | vel_y | 体速度 Y |
| 6 | yaw_rate | 偏航角速度（取自 control.yaw） |

---

## 四、rosbridge（WebSocket 9090）JSON 示例

系统内 rosbridge-websocket 服务监听 0.0.0.0:9090，上位机可经 WebSocket 收发以上话题。

发送速度指令：

{"op":"publish","topic":"/rl_real/cmd_vel","msg":{"linear":{"x":0.5,"y":0.0,"z":0.0},"angular":{"x":0.0,"y":0.0,"z":0.3}}}

发送控制指令（置零）：

{"op":"publish","topic":"/rl_real/command","msg":{"data":"zero"}}

订阅电机遥测：

{"op":"subscribe","topic":"/rl_real/motor_state"}

---

## 五、命令行测试

```bash
# 查看电机遥测
ros2 topic echo /rl_real/motor_state

# 查看 IMU / 位姿
ros2 topic echo /rl_real/imu_state
ros2 topic echo /rl_real/pose2d

# 起身 / 趴下 / 行走 / 急停
ros2 topic pub -1 /rl_real/command std_msgs/msg/String "{data: getup}"
ros2 topic pub -1 /rl_real/command std_msgs/msg/String "{data: getdown}"
ros2 topic pub -1 /rl_real/command std_msgs/msg/String "{data: locomotion}"
ros2 topic pub -1 /rl_real/command std_msgs/msg/String "{data: passive}"

# 运行中全电机置零
ros2 topic pub -1 /rl_real/command std_msgs/msg/String "{data: zero}"

# 速度控制（1 m/s 前进，0.3 rad/s 左转）
ros2 topic pub -r 10 /rl_real/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}, angular: {z: 0.3}}"

# 查询状态
ros2 topic pub -1 /rl_real/command std_msgs/msg/String "{data: status}"
```

---

## 六、硬件通道对照

| 通道 | 设备别名 | 硬件 | udev 绑定方式 |
|------|---------|------|--------------|
| 关节电机 1-4 | /dev/ttycan1-4 | HDSC 4 路 USB 串口（2e88:4603） | USB 端口 devpath 1.3.1-1.3.4 |
| IMU | /dev/ttyimu | DM-IMU-L1（6877:4d55） | 序列号 DMIMU20250212 |

电机控制协议（JXG_sdk motor_manager.cpp）：
使能 0xFC / 失能 0xFD / 置零 0xFE；
启动流程 initAllMotors = 置零 -> 使能 -> 置零，
随后 CheckMotorInitPosition 以 1.0 rad 阈值检查零位。
