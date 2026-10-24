# rl_real_JXG ROS2 Telemetry Interface

This repository keeps state switching on the Xbox gamepad. The ROS2 additions here only publish telemetry.

Build and run:

```bash
source /opt/ros/humble/setup.bash
./build.sh -m
./cmake_build/bin/rl_real_JXG wheel
```

## `/rl_real/motor_state`

Type: `std_msgs/msg/Float32MultiArray`

Published at the same cadence as the original `[MOTOR]` console log, currently once every 100 control cycles, about 5 Hz when the control loop runs at 500 Hz.

Each motor row has 9 fields:

```text
idx, target_pos, real_pos, target_vel, real_vel, target_tau, real_tau, kp, kd
```

In `wheel` mode there are 16 rows, so `len(data) == 144`.

Motor index mapping:

```text
0 FL_hip    1 FL_thigh    2 FL_calf    3 FL_foot
4 FR_hip    5 FR_thigh    6 FR_calf    7 FR_foot
8 RL_hip    9 RL_thigh   10 RL_calf   11 RL_foot
12 RR_hip  13 RR_thigh   14 RR_calf   15 RR_foot
```

## `/rl_real/imu_state`

Type: `std_msgs/msg/Float32MultiArray`

Published together with `/rl_real/motor_state`.

Fields:

```text
quat_w, quat_x, quat_y, quat_z,
roll, pitch, yaw,
acc_x, acc_y, acc_z,
gyro_x, gyro_y, gyro_z,
pos_north, pos_east, pos_down,
vel_body_x, vel_body_y, vel_body_z
```

Units:

```text
roll/pitch/yaw: rad
acc: m/s^2
gyro: rad/s
position NED: m
body velocity: m/s
```

Examples:

```bash
ros2 topic echo /rl_real/motor_state
ros2 topic echo /rl_real/imu_state
```
