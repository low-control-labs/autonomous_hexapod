import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='freenove_sensors',
            executable='hcsr04_node.py',
            name='hcsr04_node',
            output='screen',
            parameters=[{
                'trigger_pin': 27,
                'echo_pin': 22,
                'update_rate_hz': 15.0
            }]
        ),

        Node(
            package='freenove_sensors',
            executable='mpu6050_node.py',
            name='mpu6050_node',
            output='screen',
            parameters=[{
                'i2c_bus': 1,
                'i2c_address': 0x68
            }]
        ),

        Node(
            package='camera_ros',
            executable='camera_node',
            name='camera_driver',
            output='screen',
            parameters=[{
                'width': 640,
                'height': 480,
                'frame_id': 'camera',
                'camera' : 1,
            }]
        ),

        # Node(
        #     package='v4l2_camera',
        #     executable='v4l2_camera_node',
        #     name='v4l2_camera_node',
        #     output='screen',
        #     parameters=[{
        #         'video_device': '/dev/video0', 
        #         'image_size': [640, 480],
        #         'camera_frame_id': 'camera'
        #     }]
        # )
    ])