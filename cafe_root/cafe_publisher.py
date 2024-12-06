import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random
import time

class Customernode(Node):
    def __init__(self):
        super().__init__('test_node')
        self.publisher_node = self.create_publisher(String, 'customer/order', 10)
        self.confirmation_timer = None  # Timer for order confirmation
        self.confirmed = False  # Flag for order confirmation status
        self.order_id = None  # Store the current order ID
        self.timer = self.create_timer(5.0, self.place_order)  # Place orders every 5 seconds

    def place_order(self):
        """
        This method generates a random order and sends it to the kitchen.
        Starts a timer for order confirmation.
        """
        self.order_id = random.randint(1, 100)
        order_details = f"Order ID: {self.order_id}, Coffee for Table {random.randint(1, 5)}"
        
        msg = String()
        msg.data = order_details

        # Publish the order and log the details
        self.publisher_node.publish(msg)
        self.get_logger().info(f"Order sent: {msg.data}")

        # Start a 10-second timer for order confirmation
        if self.confirmation_timer is None:
            self.confirmation_timer = self.create_timer(10.0, self.handle_timeout)

    def handle_timeout(self):
        """
        If the order is not confirmed within 10 seconds, it is considered canceled.
        """
        if not self.confirmed:
            self.get_logger().info(f"Order {self.order_id} canceled due to timeout.")
            cancel_msg = String()
            cancel_msg.data = f"Order ID: {self.order_id} - Canceled due to timeout."
            self.publisher_node.publish(cancel_msg)
        
        # Stop the confirmation timer after handling the timeout
        self.cancel_confirmation_timer()

    def confirm_order(self):
        """
        This method simulates the confirmation of the order.
        """
        if self.order_id is not None:
            self.get_logger().info(f"Order {self.order_id} confirmed!")
            self.confirmed = True
            self.cancel_confirmation_timer()

    def cancel_confirmation_timer(self):
        """
        Cancel the order confirmation timer if it's still running.
        """
        if self.confirmation_timer is not None:
            self.confirmation_timer.cancel()
            self.confirmation_timer = None


def main(args=None):
    rclpy.init(args=args)

    # Initialize the Customer Node
    node = Customernode()

    try:
        while rclpy.ok():
            # Get user input for the order or confirmation
            order_text = input("Enter your order (e.g., 'Coffee for Table 1') or type 'confirm' to confirm the order or 'exit' to quit: ")

            if order_text.lower() == 'exit':
                break
            elif order_text.lower() == 'confirm':
                # Simulate confirmation from the customer side
                node.confirm_order()
            else:
                # Manually send a user-defined order to the system
                msg = String()
                msg.data = order_text
                node.publisher_node.publish(msg)
                node.get_logger().info(f"Custom order sent: {msg.data}")
                
            time.sleep(1)  # Small delay for user input handling

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
