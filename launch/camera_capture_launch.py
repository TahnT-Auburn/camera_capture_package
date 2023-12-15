from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    config_path = '/home/tahnt/T3_Repos/camera_packages/ros2_ws/src/camera_capture_package/config/camera_capture_config.yaml'
    
    return LaunchDescription([
        Node(
            package="camera_capture_package",
            executable="capture",
            namespace="camera1",
            name="camera_capture",
            output="screen",
            parameters=[config_path]
        ),
    ])