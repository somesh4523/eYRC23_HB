import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
import numpy as np

class Publisher(Node):
    def __init__(self):
        super().__init__('Publisher_node')
        self.left_wheel_force = self.create_publisher(Wrench, '/hb_bot_1/left_wheel_force', 10)
        self.right_wheel_force = self.create_publisher(Wrench, '/hb_bot_1/right_wheel_force', 10)
        self.rear_wheel_force = self.create_publisher(Wrench, '/hb_bot_1/rear_wheel_force', 10)

        timer_period = 0.5  # seconds
       # Initialze Publisher with the "/Integer" topic
        self.timer = self.create_timer(0.5,self.timer_callback)
        self.i = 0
    def timer_callback(self):
 
        x_vel = 0.0
        y_vel = 19.50
        theta_vel = 0.0
        d = 500/22
        rotation_matrix = np.array([[  -0.5  ,  -np.sqrt(3)/2  ,  d   ],
                                    [  -0.5  ,   np.sqrt(3)/2  ,  d   ],
                                    [     1  ,   0             ,  d   ] ] ) 
        
        [u1, u2, u3] = np.dot( rotation_matrix, [x_vel, y_vel, theta_vel]) 

        forces_left = Wrench()
        forces_right = Wrench()
        forces_rear = Wrench()

        forces_left.force.y = u1
        forces_right.force.y = u2
        forces_rear.force.y = u3

        print(u1)
        print(u2)
        print(u3)

        self.right_wheel_force.publish(forces_right)
        self.left_wheel_force.publish(forces_left)
        self.rear_wheel_force.publish(forces_rear)

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