import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class KitchenNode(Node):
    def __init__(self):
        super().__init__('kitchen_node')
        self.order_sub = self.create_subscription(String, 'kitchen/orders', self.handle_order, 10)
        self.ready_pub = self.create_publisher(String, 'kitchen/ready', 10)

    def handle_order(self, msg):
        self.get_logger().info(f"Received order in kitchen: {msg.data}")
        self.get_logger().info("Cooking food...")
        time.sleep(5)  # Simulate cooking time
        self.ready_pub.publish(msg)  # Send "food ready" confirmation
        self.get_logger().info("Food is ready!")

def main(args=None):
    rclpy.init(args=args)
    node = KitchenNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
