import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ButlerNode(Node):
    def __init__(self):
        super().__init__('butler_node')
        self.order_sub = self.create_subscription(
            String,
            'customer/order',
            self.process_order,
            10
        )
        self.confirm_pub = self.create_publisher(
            String,
            'confirmation_topic',
            10
        )
        self.get_logger().info('Butler Node initialized and listening for orders.')

        # Subscribe to kitchen's food ready messages
        self.food_ready_sub = self.create_subscription(
            String,
            'kitchen/ready',
            self.handle_food_ready,  # Ensure this is a function, not just a reference
            10
        )

    def process_order(self, msg):
        order = msg.data
        self.get_logger().info(f"Processing order: {order}")
        
        # Confirm order to the kitchen
        confirmation_msg = String()
        confirmation_msg.data = f"{order} - Confirmed"
        self.confirm_pub.publish(confirmation_msg)
        self.get_logger().info(f"Order confirmed: {confirmation_msg.data}")

    def handle_food_ready(self, msg):
        # Handle the food ready notification from the kitchen
        food_ready_msg = msg.data
        self.get_logger().info(f"Received food ready message: {food_ready_msg}")
        
        # Acknowledge food delivery to the kitchen
        ack_msg = String()
        ack_msg.data = f"{food_ready_msg} - Delivered"
        self.confirm_pub.publish(ack_msg)
        self.get_logger().info(f"Acknowledgment sent to kitchen: {ack_msg.data}")


def main(args=None):
    rclpy.init(args=args)
    node = ButlerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Butler Node gracefully.")
    finally:
        node.destroy_node()
        rclpy.shutdown()  # Only call shutdown here

if __name__ == '__main__':
    main()
