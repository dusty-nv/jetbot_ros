import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir, LaunchConfiguration
from launch_ros.actions import Node

from datetime import datetime



def generate_launch_description():
    
    teleop_keyboard = IncludeLaunchDescription(PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/teleop_keyboard.launch.py']))
      
    data_root = DeclareLaunchArgument('data_root', default_value="/workspace/src/jetbot_ros/data/datasets")
    data_path = DeclareLaunchArgument('data_path', default_value=f"{datetime.now().strftime('%Y%m%d-%H%M%S')}")

    data_collection = Node(package='jetbot_ros', node_executable='data_collection',
                           parameters=[
                               {"data_path": [LaunchConfiguration('data_root'), os.path.sep, LaunchConfiguration('data_path')]},
                           ],
                           output='screen', emulate_tty=True) 
                           
    return LaunchDescription([
        teleop_keyboard,
        data_root,
        data_path,
        data_collection
    ])