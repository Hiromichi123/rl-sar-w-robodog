# rl_real_JXG ROS2 control interface

Start the robot program with ROS2 sourced and wheel mode enabled:

```bash
source /opt/ros/jazzy/setup.bash
source /home/kickpi/rl_sar-w2_recovered/install/setup.bash
./cmake_build/bin/rl_real_JXG wheel
```

## Control topics

### `/rl_real/command`

Type: `std_msgs/msg/String`

Commands:

- `getup`, `stand`, `up`: request `RLFSMStateGetUp`.
- `getdown`, `down`, `lie`, `sit`: request `RLFSMStateGetDown` and zero velocity.
- `locomotion`, `rl`, `walk`, `move`: request `RLFSMStateRLLocomotion`.
- `vel_stop`, `hold`: zero velocity without forcing passive.
- `nav_on`, `nav_off`: toggle use of ROS `cmd_vel` as RL command source.
- `imu_on`, `imu_off`: enable or freeze IMU stream for tests.
- `zero`, `setzero`, `set_zero`, `set_zero_all`, `zero_all`: call `motor_manager.setZeroAllMotors()`.
- `status`, `feedback`: publish current status to `/rl_real/feedback`.
- `quit`, `exit`: request process shutdown.

> **`passive` / `stop` / `safe_stop` 已从 ROS 接口隐藏（2026-10-01）**：趴下一律用 `getdown`
> （受控插值）。ROS 侧发这些会被直接拒绝（feedback 回
> `command rejected: passive hidden on ROS (use getdown)`，cmd_state 立即 `REJECTED`）。
> 内置 Web 调试台（WebSocket）与手柄 P 键仍保留 passive。

Terminal examples:

```bash
ros2 topic pub --once /rl_real/command std_msgs/msg/String "{data: getup}"
ros2 topic pub --once /rl_real/command std_msgs/msg/String "{data: getdown}"
ros2 topic pub --once /rl_real/command std_msgs/msg/String "{data: set_zero_all}"
```

### `/rl_real/cmd_vel`

Type: `geometry_msgs/msg/Twist`

Only these fields are used:

- `linear.x`: forward velocity command
- `linear.y`: lateral velocity command
- `angular.z`: yaw velocity command

Example:

```bash
ros2 topic pub -r 10 /rl_real/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

### `/rl_real/imu_stream_enable`

Type: `std_msgs/msg/Bool`

- `true`: use real DM-IMU stream.
- `false`: freeze IMU output to neutral zero state for software tests.

The recovered default is enabled at startup.

## Feedback topic

### `/rl_real/feedback`

Type: `std_msgs/msg/String`

Examples:

```text
command accepted: getup
cmd_vel accepted
cmd_vel ignored: gamepad override active
status state=RLFSMStatePassive gamepad_override=false
```

Physical gamepad input has priority. When a stick or button is active, ROS velocity and mode commands are ignored except `status`/`feedback`.

## Telemetry topics

### `/rl_real/motor_state`

Type: `std_msgs/msg/Float32MultiArray`

Published at about 20 Hz. Layout:

```text
dim[0].label = motor
dim[0].size = num_dofs
dim[1].label = field:idx,target_pos,real_pos,target_vel,real_vel,target_tau,real_tau,kp,kd
dim[1].size = 9
```

Each motor row has 9 float values:

```text
[idx, target_pos, real_pos, target_vel, real_vel, target_tau, real_tau, kp, kd]
```

For the 16-motor JXGw low-level order:

```text
0 FL_hip   1 FL_thigh   2 FL_calf   3 FL_foot
4 FR_hip   5 FR_thigh   6 FR_calf   7 FR_foot
8 RL_hip   9 RL_thigh  10 RL_calf  11 RL_foot
12 RR_hip 13 RR_thigh 14 RR_calf  15 RR_foot
```

### `/rl_real/pose2d`

Type: `std_msgs/msg/Float32MultiArray`

Published at about 20 Hz. This is a lightweight cloud feedback pose estimated by integrating accepted velocity commands. Layout:

```text
dim[0].label = field:x,y,z,yaw,vel_x,vel_y,yaw_rate
data = [x, y, z, yaw, vel_x, vel_y, yaw_rate]
```

### `/rl_real/imu_state`

Type: `std_msgs/msg/Float32MultiArray`

Published at about 20 Hz. Layout:

```text
dim[0].label = field:quat_w,quat_x,quat_y,quat_z,roll,pitch,yaw,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z,pos_north,pos_east,pos_down,vel_body_x,vel_body_y,vel_body_z
```

Data order:

```text
[quat_w, quat_x, quat_y, quat_z,
 roll, pitch, yaw,
 acc_x, acc_y, acc_z,
 gyro_x, gyro_y, gyro_z,
 pos_north, pos_east, pos_down,
 vel_body_x, vel_body_y, vel_body_z]
```

## Python helper

`/home/kickpi/rl_sar-w2_recovered/rl_real_ros2_client.py` exposes executable helpers and CLI commands:

```bash
python3 rl_real_ros2_client.py stand
python3 rl_real_ros2_client.py down
python3 rl_real_ros2_client.py locomotion
python3 rl_real_ros2_client.py vel 0.2 0.0 0.0
python3 rl_real_ros2_client.py stop
python3 rl_real_ros2_client.py set_zero_all
python3 rl_real_ros2_client.py imu on
```
