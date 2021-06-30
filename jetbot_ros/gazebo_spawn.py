import os
import sys
import rclpy

from ament_index_python.packages import get_package_share_directory 
from gazebo_msgs.srv import SpawnEntity


def main():

    """ Main for spawning a robot node """
    # Get input arguments from user
    argv = sys.argv[1:]

    # Start node
    rclpy.init()

    # Create the node
    node = rclpy.create_node("entity_spawner")

    # Show progress in the terminal window
    node.get_logger().info(
        'Creating Service client to connect to `/spawn_entity`')
    client = node.create_client(SpawnEntity, "/spawn_entity")

    # Get the spawn_entity service
    node.get_logger().info("Connecting to `/spawn_entity` service...")
    if not client.service_is_ready():
        client.wait_for_service()
        node.get_logger().info("...connected!")

    # Get path to the robot
    sdf_file_path = os.path.join(
        get_package_share_directory("jetbot_ros"), "models",
        "simple_diff_ros", "model.sdf")

    # Show file path
    print(f"robot_sdf={sdf_file_path}")
    
    # Set data for request
    request = SpawnEntity.Request()
    request.name = argv[0]
    request.xml = open(sdf_file_path, 'r').read()
    request.robot_namespace = argv[1]
    request.initial_pose.position.x = float(argv[2])
    request.initial_pose.position.y = float(argv[3])
    request.initial_pose.position.z = float(argv[4])

    node.get_logger().info("Sending service request to `/spawn_entity`")
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        print('response: %r' % future.result())
    else:
        raise RuntimeError(
            'exception while calling service: %r' % future.exception())

    node.get_logger().info("Done! Shutting down node.")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
