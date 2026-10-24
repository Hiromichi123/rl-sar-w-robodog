from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rl_nav_examples',
            executable='simple_goal_follower.py',
            name='rl_simple_goal_follower',
            output='screen',
            parameters=[{
                'goal_topic': '/cloud/goal_pose',
                'goal_point_topic': '/cloud/goal_point',
                'odom_topic': '/Odometry',
                'slam_pose_topic': '/slam/pose',
                'rl_pose2d_topic': '/rl_real/pose2d',
                'control_rate_hz': 20.0,
                'goal_tolerance_m': 0.20,
                'yaw_tolerance_rad': 0.15,
                'max_vx': 0.25,
                'max_vy': 0.18,
                'max_yaw_rate': 0.45,
                'send_nav_on_on_goal': True,
                'send_locomotion_on_goal': False,
                'send_nav_off_on_reach': False,
            }],
        )
    ])
