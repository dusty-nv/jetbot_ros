import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir,LaunchConfiguration
from launch_ros.actions import Node

                     
def generate_launch_description():
    
    model_arg = DeclareLaunchArgument('model')
    
    nav_model = Node(package='jetbot_ros', node_executable='nav_model',
                     parameters=[
                        {"model": LaunchConfiguration('model')},
                        {"visualize": True},
                     ],
                     output='screen',
                     emulate_tty=True,
                     #arguments=[('__log_level:=debug')]
                )              
                     
    return LaunchDescription([
        model_arg,
        nav_model
    ])