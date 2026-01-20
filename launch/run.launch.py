from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cv_object_tracker',
            executable='cv_node',
            output='screen',
            parameters=[{'use_sim_time': True}]
        )
    ])  