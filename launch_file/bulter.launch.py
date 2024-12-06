from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Create a LaunchDescription object
    ld = LaunchDescription()

    # Define the Customer Node
    customer_node = Node(
        package="cafe_root",         # Your package name
        executable="test_sub",      # Node executable for customer
        name="customer_node",        # Name of the node
        output="screen"              # Output logs to screen
    )

    # Define the Robot Node
    robot_node = Node(
        package="cafe_root",         # Your package name
        executable="test_chef",       # Node executable for robot
        name="robot_node",           # Name of the node
        output="screen"              # Output logs to screen
    )

    # Add nodes to the LaunchDescription
    ld.add_action(customer_node)
    ld.add_action(robot_node)

    # Return the LaunchDescription
    return ld
