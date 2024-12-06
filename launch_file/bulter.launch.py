from launch import LaunchDescription
from launch.actions import LogInfo, TimerAction
from launch_ros.actions import Node  # Correct import for Node

def generate_launch_description():
    return LaunchDescription([
        # Log a message before starting the first node
        LogInfo(
            condition=None,
            msg="Starting Publisher Node (test_node)..."
        ),
        
        # Start the Publisher Node (test_node) first
        Node(
            package='cafe_root',
            executable='test_node',  # Name of the Publisher node executable
            name='test_node',        # Node name (optional)
            output='screen',         # Print logs to the screen
        ),

        # Log message before starting the Subscriber Node
        LogInfo(
            condition=None,
            msg="Starting Subscriber Node (test_sub)..."
        ),

        # Add a small delay to ensure the publisher node starts first
        TimerAction(
            period=1.0,  # 1-second delay before starting Subscriber Node
            actions=[
                Node(
                    package='cafe_root',
                    executable='test_sub',  # Name of the Subscriber node executable
                    name='test_sub',        # Node name (optional)
                    output='screen',        # Print logs to the screen
                )
            ]
        ),

        # Log message before starting the Kitchen Node
        LogInfo(
            condition=None,
            msg="Starting Kitchen Node (test_chef)..."
        ),

        # Add another delay to allow Subscriber Node to receive the message before the Kitchen Node starts
        TimerAction(
            period=2.0,  # 2-second delay before starting Kitchen Node
            actions=[
                Node(
                    package='cafe_root',
                    executable='test_chef',  # Name of the Kitchen node executable
                    name='test_chef',        # Node name (optional)
                    output='screen',         # Print logs to the screen
                )
            ]
        )
    ])
