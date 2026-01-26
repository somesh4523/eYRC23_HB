import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class Publisher(Node):
    def __init__(self):
        super().__init__('Publisher_node')
        self.publisher1_ = self.create_publisher(Twist, '/cmd_vel/bot_1', 10)
        self.publisher2_ = self.create_publisher(Twist, '/cmd_vel/bot_2', 10)
        self.publisher3_ = self.create_publisher(Twist, '/cmd_vel/bot_3', 10)
        timer_period = 0.5  # seconds
       # Initialze Publisher with the "/Integer" topic
        self.timer = self.create_timer(0.5,self.timer_callback)
        self.i = 0
    def timer_callback(self):
        omg1 = Twist()
        if self.i <= 25:
            omg1.linear.x = -0.0087
            omg1.linear.y = 0.0087
            omg1.angular.z = 0.00

        elif self.i >25 and self.i <= 50:
            omg1.linear.x = -0.005
            omg1.linear.y = -0.005
            omg1.angular.z = 0.01

        elif self.i >50 and self.i <= 75:
            omg1.linear.x = 0.0087
            omg1.linear.y = -0.0087
            omg1.angular.z = 0.00

        elif self.i >75 and self.i <= 100:
            omg1.linear.x = 0.005
            omg1.linear.y = 0.005
            omg1.angular.z = -0.01

        else:
            omg1.linear.x = 0.0
            omg1.linear.y = 0.0
            omg1.angular.z = 0.0

        self.publisher1_.publish(omg1)


        omg2 = Twist()
        if self.i <= 33:
            omg2.linear.x = -0.0087
            omg2.linear.y = 0.0087
            omg2.angular.z = 0.00
        elif self.i > 33 and self.i <= 66:
            omg2.linear.x = 0.00
            omg2.linear.y = -0.0087
            omg2.angular.z = 0.0087
        elif self.i > 66 and self.i <= 99:
            omg2.linear.x = 0.0087
            omg2.linear.y = 0.00
            omg2.angular.z = -0.0087
        else:
            omg2.linear.x = 0.0
            omg2.linear.y = 0.0
            omg2.angular.z = 0.0

        self.publisher2_.publish(omg2)

        omg3 = Twist()
        if self.i <= 100:
            omg3.linear.x = 0.001339
            omg3.linear.y = 0.018660
            omg3.angular.z = 0.01
        else:
            omg3.linear.x = 0.0
            omg3.linear.y = 0.0
            omg3.angular.z = 0.0

        self.publisher3_.publish(omg3)

        self.i += 1
        print(self.i)
        if self.i >100:
            self.i = 0
        
 

def main(args = None):
    rclpy.init(args=args)
    Publisher_node = Publisher()
    rclpy.spin(Publisher_node)
    Publisher_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()