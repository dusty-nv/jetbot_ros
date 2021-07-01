import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir,LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    
    nav_model = Node(package='jetbot_ros', node_executable='nav_model',
                     parameters=[
                        {"model": "/workspace/src/jetbot_ros/data/models/202106282129/model_best.pth"},
                        {"visualize": True},
                     ],
                     remappings=[
                        ("/jetbot/nav_model/image_in", "/jetbot/camera/image_raw"),
                        ("/jetbot/nav_model/cmd_vel", "/jetbot/cmd_vel"),
                     ],
                     output='screen')              
                     
    return LaunchDescription([
        nav_model
    ])