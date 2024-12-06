import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading

class CustomerNode(Node):
    def __init__(self):
        super().__init__('test_node')
        self.publisher_node = self.create_publisher(String, 'customer/order', 10)

        self.stop_thread = False  # Flag to control the stopping of the thread
        self.input_thread = threading.Thread(target=self.handle_user_input)
        self.input_thread.start()

    def handle_user_input(self):
        
        while not self.stop_thread:
            try:
                print("\nPlease select a table (1-4) for your order.")
                table_number = input("Enter table number (1-4) or 'exit' to quit: ").strip().lower()

                if table_number == 'exit':
                    self.get_logger().info("Exiting Customer Node.")
                    self.stop_thread = True  # Signal to stop the thread
                    break
                elif table_number in ['1', '2', '3', '4']:  # Check if valid table number
                    order_text = input(f"Enter order for Table {table_number} (e.g., 'Coffee', 'Tea', etc.): ").strip()
                    if order_text:
                        order_msg = f"Order for Table {table_number}: {order_text}"
                        msg = String()
                        msg.data = order_msg

                        self.publisher_node.publish(msg)
                        self.get_logger().info(f"Custom order sent: {msg.data}")
                    else:
                        self.get_logger().info("Order cannot be empty. Please try again.")
                else:
                    self.get_logger().info("Invalid table number. Please select a table between 1 and 4.")
            except Exception as e:
                self.get_logger().error(f"Error in input thread: {e}")
                self.stop_thread = True

    def destroy_node(self):
       
        self.stop_thread = True
        if self.input_thread.is_alive():
            self.input_thread.join()  
        super().destroy_node()


def main(args=None):
    
    rclpy.init(args=args)
    node = CustomerNode()

    try:
        rclpy.spin(node) 
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt detected. Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()  # Shutdown ROS 2 when done


if __name__ == '__main__':
    main()
