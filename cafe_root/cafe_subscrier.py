import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ButlerNode(Node):
    def __init__(self):
        super().__init__('butler_node')
        self.subscription = self.create_subscription(
            String,
            'customer/order',  # Subscribe to the topic where the customer sends orders
            self.process_order,
            10)
        self.publisher_ = self.create_publisher(String, 'confirmation_topic', 10)  # Publish confirmations
        self.get_logger().info('Butler Node started and listening for orders.')

    def process_order(self, msg):
        order = msg.data
        self.get_logger().info(f"Received order: {order}")

        # Simulate confirmation logic
        if "Canceled" in order:
            self.get_logger().info(f"Order is canceled: {order}")
        else:
            self.confirm_order(order)

    def confirm_order(self, order):
        # Simulate sending a confirmation
        confirmation_msg = String()
        confirmation_msg.data = f"{order} - Confirmed"
        self.publisher_.publish(confirmation_msg)
        self.get_logger().info(f"Order confirmed: {confirmation_msg.data}")

def main():
    rclpy.init()
    node = ButlerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()