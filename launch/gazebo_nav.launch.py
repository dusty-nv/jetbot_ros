import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir,LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

from ament_index_python.packages import get_package_share_directory
 
def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    world_file_name = 'dirt_path_curves.world'
    pkg_dir = get_package_share_directory('jetbot_ros')
 
    os.environ["GAZEBO_MODEL_PATH"] = os.path.join(pkg_dir, 'models')
 
    world = os.path.join(pkg_dir, 'worlds', world_file_name)
    launch_file_dir = os.path.join(pkg_dir, 'launch')
 
    gazebo = ExecuteProcess(
            cmd=['gazebo', '--verbose', world, '-s', 'libgazebo_ros_init.so', 
            '-s', 'libgazebo_ros_factory.so'],
            output='screen')
 
    spawn_entity = Node(package='jetbot_ros', node_executable='gazebo_spawn',   # FYI 'node_executable' is renamed to 'executable' in Foxy
                        arguments=['simple_diff_ros', 'jetbot', '-.3', '-2.65', '0.0'],
                        output='screen')
 
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
        gazebo,
        spawn_entity,
        nav_model
    ])