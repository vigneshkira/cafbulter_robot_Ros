import rclpy 
from rclpy.node import Node
from std_msgs.msg import String
import random

class KitchenNode(Node):
    def __init__(self):
        super().__init__("KitchenNode")
        self.order_publish = self.create_publisher(String,'customer/orders',10)
        self.order_sub = self.create_subscription(String,'Kitchen/food',self.thanking_robot)
        
        self.timer = self.create_timer(5.0,self.place_order)


    

    def place_order(self):
        randomList=[]
        for i in range(15):
             r=random.randint(1,5)
        order_details = f"order_id:{r}"
        self.get_logger().info(f"place order:{order_details}")
        msg = String()
        msg.data = order_details
        self.order_publish.publish(msg)


   




def main(args=None):
    rclpy.init(args=args)
    node = KitchenNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
