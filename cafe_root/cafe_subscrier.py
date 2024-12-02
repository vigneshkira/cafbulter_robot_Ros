import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class CafeRobotNode(Node):
    def __init__(self):
        super().__init__('cafe_robot_node')
        self.order_sub = self.create_subscription(String, 'customer/orders', self.handle_order, 10)
        self.kitchen_pub = self.create_publisher(String, 'kitchen/orders', 10)
        self.kitchen_sub = self.create_subscription(String, 'kitchen/ready', self.handle_food_ready, 10)
        # self.kitchen_pub_table = self.create_publisher(String, 'Kitchen/food',10)

    def handle_order(self, msg):
        self.get_logger().info(f"Received order: {msg.data}")
        self.kitchen_pub.publish(msg)  # Forward order to kitchen
        self.get_logger().info("Order sent to kitchen")

    def handle_food_ready(self, msg):
        self.get_logger().info(f"Food ready: {msg.data}")
        # Simulate delivering the food
        self.get_logger().info("Delivering food to the customer...")

def main(args=None):
    rclpy.init(args=args)
    node = CafeRobotNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
