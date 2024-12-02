# import launch
# import launch_ros.actions
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    urdf_file_path = '/home/vignesh/cafe_robot_ws/src/cafe_root/world'
    return LaunchDescription(
        [
            Node(
                package='cafe_root',
                executable='test_node',
                output = "screen"
            ),

            Node(
                package='cafe_root',
                executable='test_sub',
                output = "screen"
            ),

            Node(
                package='cafe_root',
                executable='test_chef',
                output = "screen"
            ),
            Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': '<robot_urdf>'}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
        ),


        ]
    )