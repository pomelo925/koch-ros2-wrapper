from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Node 1: ctrl_leader_follower
        Node(
            package='koch_wrapper',
            executable='ctrl_leader_follower',
            name='ctrl_leader_follower',
            output='screen',
            parameters=[{
                'config_file': '/ros2-ws/src/koch_wrapper/config/leader_follower.yaml'
            }]
        )
    ])