# rl_nav_examples

Minimal closed-loop navigation example for `rl_real_JXG`.

This example does not perform obstacle avoidance. It accepts a cloud goal,
reads a SLAM pose topic, and publishes velocity commands to the low-level
`/rl_real` ROS2 interface.

## Topics

Input:

- `/cloud/goal_pose` (`geometry_msgs/msg/PoseStamped`): target x/y/yaw.
- `/cloud/goal_point` (`geometry_msgs/msg/PointStamped`): target x/y, keep current yaw.
- `/Odometry` (`nav_msgs/msg/Odometry`): common Point-LIO/FAST-LIO pose output.
- `/slam/pose` (`geometry_msgs/msg/PoseStamped`): alternative SLAM pose output.
- `/rl_real/pose2d` (`std_msgs/msg/Float32MultiArray`): fallback internal pose estimate.

Output:

- `/rl_real/cmd_vel` (`geometry_msgs/msg/Twist`): x/y/yaw speed.
- `/rl_real/command` (`std_msgs/msg/String`): `nav_on`, optional `locomotion`, `vel_stop`.
- `/rl_nav/status` (`std_msgs/msg/String`): simple status feedback.

## Build

```bash
cd /home/kickpi/rl_sar-w2_recovered
source /opt/ros/jazzy/setup.bash
source install/setup.bash 2>/dev/null || true
colcon build --merge-install --symlink-install --packages-select rl_nav_examples
source install/setup.bash
```

## Run

```bash
ros2 launch rl_nav_examples simple_goal_follower.launch.py
```

By default the node sends `nav_on` when a goal arrives, but it does not send
`locomotion`. To let the example request locomotion automatically:

```bash
ros2 launch rl_nav_examples simple_goal_follower.launch.py \
  send_locomotion_on_goal:=true
```

## Test Goal

```bash
ros2 topic pub --once /cloud/goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"
```
