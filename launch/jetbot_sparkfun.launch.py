#
# Launch Sparkfun JetBot motor controller and camera nodes.
#

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    
    teleop_keyboard = Node(package='jetbot_ros', node_executable='teleop_keyboard',
                           prefix='lxterminal -e', #'xterm -e'
                           output='screen',
                           emulate_tty=True)
    
    motor_controller = Node(package='jetbot_ros', node_executable='motors_sparkfun',
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
    
    rtp_output = DeclareLaunchArgument('rtp_output', default_value="DUSTINF-LT1.fios-router.home:1234")
    
    video_output = Node(package='ros_deep_learning', node_executable='video_output',
                        parameters=[
                            {"resource": ["rtp://", LaunchConfiguration('rtp_output')]},
                            {"codec": "h264"},
                        ],
                        remappings=[
                            ("/video_output/image_in", "/jetbot/camera/image_raw"),
                        ],
                        output='screen', emulate_tty=True)
                        
    return LaunchDescription([
        teleop_keyboard,
        motor_controller,
        rtp_output,
        video_source,
        video_output
    ])
