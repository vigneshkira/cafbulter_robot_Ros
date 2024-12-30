import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package share directory path
    package_share_dir = get_package_share_directory('cafe_root')

    # Define path to the world file
    world_file_path = "world/hotel.world"

    # Check if the world file exists
    if not os.path.exists(world_file_path):
        raise FileNotFoundError(f"World file not found: {world_file_path}")

    # Launch description
    return LaunchDescription([
        # Start Gazebo with the specified world file
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_file_path, '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        )
    ])
