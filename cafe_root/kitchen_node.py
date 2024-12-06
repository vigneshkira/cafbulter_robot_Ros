import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class KitchenNode(Node):
    def __init__(self):
        super().__init__('kitchen_node')
        # Subscription to receive orders from the Butler (i.e., customer order)
        self.order_sub = self.create_subscription(
            String,
            'customer/order',  # The topic from the Butler
            self.handle_order,
            10
        )

        # Publisher to notify that food is ready
        self.ready_pub = self.create_publisher(
            String,
            'kitchen/ready',  # Topic where kitchen will notify that food is ready
            10
        )

        self.get_logger().info('Kitchen Node started and waiting for orders.')

    def handle_order(self, msg):
        """
        Handles incoming orders from the Butler.
        """
        order = msg.data
        self.get_logger().info(f"Received order: {order}")
        
        # Simulate food preparation
        self.get_logger().info("Preparing the food...")
        time.sleep(5)  # Simulating cooking time (5 seconds)

        # Send a confirmation back that the food is ready
        ready_msg = String()
        ready_msg.data = f"Food for {order} is ready!"
        self.ready_pub.publish(ready_msg)
        self.get_logger().info(f"Sent food readiness message: {ready_msg.data}")


def main(args=None):
    rclpy.init(args=args)
    node = KitchenNode()

    try:
        rclpy.spin(node)  # Keep the node alive and process callbacks
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Kitchen Node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
