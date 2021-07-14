#
# Launch NVIDIA JetBot motor controller and camera nodes.
# This is for the original NVIDIA JetBot.
#
import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir,LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    
    motor_controller = Node(package='jetbot_ros', node_executable='motors_nvidia',
                            output='screen', emulate_tty=True)              
     
    oled_controller = Node(package='jetbot_ros', node_executable='oled_ssd1306',
                            output='screen', emulate_tty=True)  
    
    video_source = Node(package='ros_deep_learning', node_executable='video_source',
                        parameters=[
                            {"resource": "csi://0"},
                            {"width": 320},
                            {"height": 240},
                            {"framerate": 15.0},
                            {"flip": "rotate-180"},
                        ],
                        remappings=[
                            ("/video_source/raw", "/jetbot/camera/image_raw"),
                        ],
                        output='screen', emulate_tty=True)
    
    video_output = Node(package='ros_deep_learning', node_executable='video_output',
                        parameters=[
                            {"resource": "rtp://sky-x4e2.fios-router.home:1234"},
                            {"codec": "h264"},
                        ],
                        remappings=[
                            ("/video_output/image_in", "/jetbot/camera/image_raw"),
                        ],
                        output='screen', emulate_tty=True)
                        
    return LaunchDescription([
        motor_controller,
        oled_controller,
        video_source,
        video_output
    ])