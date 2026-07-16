from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_gz_image',
            executable='image_bridge',
            arguments=['/leader/camera/image_raw', '/follower/camera/image_raw'],
            output='screen'
        )
    ])
