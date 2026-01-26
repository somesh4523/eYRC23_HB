#!/usr/bin/python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn
from turtlesim.srv import SetPen

#######################################[[[ PUBLISHER ]]]############################################
class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.publisher1_ = self.create_publisher(Twist, 'turtle2/cmd_vel', 10)
        self.subscription = self.create_subscription(Pose, 'turtle1/pose', self.listener_callback, 10)
        self.cli = self.create_client(Spawn, '/spawn')
        self.req = Spawn.Request()

        self.subscription
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.status = 0
        self.loop = 0

    def paida(self, x, y, theta):
        print("paida kardu kya ?")
        self.req.x = x
        self.req.y = y
        self.req.theta = theta
        self.future = self.cli.call_async(self.req)
        
    # def __init__(delf):
    #     super().__init__('dinimal_publisher')
    #     delf.srv = delf.create_service(Spawn, 'turtlesim/Spawn', delf.add_two_ints_callback)

    def listener_callback(self, msg):
        self.whatever = 0
        # self.get_logger().info('position:')
        # self.get_logger().info('x: "%f"' % msg.x)
        # self.get_logger().info('y: "%f"' % msg.y)
        # self.get_logger().info('theta: "%f"' % msg.theta)
        # self.get_logger().info('Linear Velocity : "%f"' %msg.linear_velocity )

    def timer_callback(self):
        msg = Twist()
        self.i = self.i + 1
        # msg.data = 'Hello World: %d' % self.i
        if self.i < 30:
            msg.linear.x = 2.0
            msg.angular.z = 2.0
            self.publisher_.publish(msg)
        if 32> self.i > 30:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            print("spawn karo")
            self.paida(5.544445, 5.544445, 0.0)
            print("kar diya")
        if 86 > self.i > 32:
            msg.linear.x = -2.5
            msg.angular.z = 1.0
            self.publisher1_.publish(msg)

        
        


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()
    # Spawn = minimal_publisher.send_request()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    
    
    
