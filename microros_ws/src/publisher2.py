import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class Publisher(Node):
    def __init__(self):
        super().__init__('Publisher_node')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel/bot_1', 10)
        timer_period = 0.5  # seconds
       # Initialze Publisher with the "/Integer" topic
        self.timer = self.create_timer(0.5,self.timer_callback)
        self.i = 0
    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 1.0
        msg.linear.y = 0.0
        msg.angular.z = 1.0
        self.publisher_.publish(msg)
        self.i += 1
       # Assign the msg variable to i
       # Publish the msg 
       # Increment the i

def main(args = None):
    rclpy.init(args=args)
    Publisher_node = Publisher()
    rclpy.spin(Publisher_node)
    Publisher_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()