import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir, LaunchConfiguration
from launch_ros.actions import Node

                     
def generate_launch_description():
    
    model_arg = DeclareLaunchArgument('model')
    
    speed_gain = DeclareLaunchArgument('speed_gain', default_value="0.15")
    steering_gain = DeclareLaunchArgument('steering_gain', default_value="0.4")
    
    nav_model = Node(package='jetbot_ros', node_executable='nav_model',
                     parameters=[
                        {"model": LaunchConfiguration('model')},
                        {"speed_gain": LaunchConfiguration('speed_gain')},
                        {"steering_gain": LaunchConfiguration('steering_gain')},
                        {"visualize": True},
                     ],
                     output='screen',
                     emulate_tty=True,
                     #arguments=[('__log_level:=debug')]
                )              
                     
    return LaunchDescription([
        model_arg,
        speed_gain,
        steering_gain,
        nav_model
    ])