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
                            output='screen',
                            emulate_tty=True)              
       
    return LaunchDescription([
        motor_controller
    ])