#! /usr/bin/env python3

'''
*****************************************************************************************
*
*        		===============================================
*           		Hologlyph Bots (HB) Theme (eYRC 2023-24)
*        		===============================================
*
*  This script is to implement Task 2A of Hologlyph Bots (HB) Theme (eYRC 2023-24).
*  
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or 
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''


# Team ID:		[ Team-ID ]
# Author List:		[ Names of team members worked on this file separated by Comma: Name1, Name2, ... ]
# Filename:		feedback.py
# Functions:
#			[ Comma separated list of functions in this file ]
# Nodes:		Add your publishing and subscribing node


################### IMPORT MODULES #######################

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import numpy as np
import math
from tf_transformations import euler_from_quaternion
from my_robot_interfaces.srv import NextGoal             
from geometry_msgs.msg import Wrench
from geometry_msgs.msg import Pose2D
# You can add more if required
##############################################################


# Initialize Global variables


################# ADD UTILITY FUNCTIONS HERE #################

##############################################################


# Define the HBController class, which is a ROS node
class HBController(Node):
    def __init__(self):
        super().__init__('hb_controller')
        
        # Initialze Publisher and Subscriber
        # NOTE: You are strictly NOT-ALLOWED to use "cmd_vel" or "odom" topics in this task
	    #	Use the below given topics to generate motion for the robot.
	    #   /hb_bot_1/left_wheel_force,
	    #   /hb_bot_1/right_wheel_force,
	    #   /hb_bot_1/left_wheel_force


        self.left_wheel_force = self.create_publisher(Wrench, '/hb_bot_1/left_wheel_force', 10)
        self.right_wheel_force = self.create_publisher(Wrench, '/hb_bot_1/right_wheel_force', 10)
        self.rear_wheel_force = self.create_publisher(Wrench, '/hb_bot_1/rear_wheel_force', 10)

        self.subscription = self.create_subscription( Pose2D, '/detected_aruco', self.aruco_callback, 10)
        PI = math.pi
        self.Kp = 3.75
        self.Kw = 2.4
        
        # For maintaining control loop rate.
        self.rate = self.create_rate(100)
        self.x_goals = [ 0, 0, 0, 0, 0]
        self.y_goals = [0, 0, 0, 0, 0]
        self.theta_goals = [0, 0, 0, 0, 0]

        self.xPose = 0.0
        self.yPose = 0.0
        self.thetaPose = 0.0

        # client for the "next_goal" service
        self.cli = self.create_client(NextGoal, 'next_goal')      
        self.req = NextGoal.Request() 
        self.index = 0

    def aruco_callback(self, data):
        self.xPose = data.x
        self.yPose = data.y
        self.thetaPose = data.theta
    
    # Method to create a request to the "next_goal" service
    def send_request(self, index):
        self.req.request_goal = index
        self.future = self.cli.call_async(self.req)
        

    def inverse_kinematics(self, x_vel, y_vel, theta_vel):
        # x_vel = 10.0
        # y_vel = 0.0
        # theta_vel = 0.0

        ############ ADD YOUR CODE HERE ############

        # INSTRUCTIONS & HELP : 
        #	-> Use the target velocity you calculated for the robot in previous task, and
        #	Process it further to find what proportions of that effort should be given to 3 individuals wheels !!
        #	Publish the calculated efforts to actuate robot by applying force vectors on provided topics
        ##################################################################

        r = 0.14
        d = 0.67999
        # vel_front = theta_vel + x_vel
        # vel_right = theta_vel - x_vel*math.cos(math.pi/3) - y_vel*math.sin(math.pi/3)
        # vel_left = theta_vel - x_vel*math.cos(math.pi/3) + y_vel*math.sin(math.pi/3)

        # rotation_matrix = np.array([[  -1/3     ,  -1/3   ,  2/3    ],
        #                             [ (2/3)*(-0.86602540378)    , -(2/3)*(-0.86602540378)   ,  0      ],
        #                             [  1/(3*d)  , 1/(3*d) ,1/(3*d)  ] ] ) 
        
        # [u1, u2, u3] = np.dot( np.linalg.inv(rotation_matrix), [x_vel, y_vel, theta_vel]) 

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

        self.right_wheel_force.publish(forces_right)
        self.left_wheel_force.publish(forces_left)
        self.rear_wheel_force.publish(forces_rear)

        pass


def main(args=None):
    rclpy.init(args=args)
    
    # Create an instance of the HBController class
    hb_controller = HBController()
   
    # Send an initial request with the index from hb_controller.index
    hb_controller.send_request(hb_controller.index)
    
    # Main loop
    while rclpy.ok():

        # Check if the service call is done
        if hb_controller.future.done():
            try:
                # response from the service call
                response = hb_controller.future.result()
            except Exception as e:
                hb_controller.get_logger().infselfo(
                    'Service call failed %r' % (e,))
            else:
                #########           GOAL POSE             #########
                x_goal      = response.x_goal + 250
                y_goal      = response.y_goal + 250
                theta_goal  = response.theta_goal
                hb_controller.flag = response.end_of_list
                ####################################################
                print("x Goal " + str(x_goal), "y Goal " + str(y_goal), "theta Goal " + str(theta_goal) )
                x_err = x_goal - hb_controller.xPose
                y_err = y_goal - hb_controller.yPose 
                theta_err = theta_goal - hb_controller.thetaPose

                x_err_body = math.cos(hb_controller.thetaPose) * x_err - math.sin(hb_controller.thetaPose) * y_err
                y_err_body = math.sin(hb_controller.thetaPose) * x_err + math.cos(hb_controller.thetaPose) * y_err

                vel_x = hb_controller.Kp * x_err_body
                vel_y = -hb_controller.Kp * y_err_body
                W_z = -hb_controller.Kw * theta_err
                # Safety Check
                # make sure the velocities are within a range.
                if vel_x > 16:
                    vel_x = 16
                if vel_x < -16:
                    vel_x = -16
                if vel_y > 16:
                    vel_y = 16
                if vel_y < -16:
                    vel_y = -16
                if W_z   > 16:
                    W_z  = 16
                if W_z   < -16:                       
                    W_z  = -16

                print(vel_x, vel_y, W_z)

                hb_controller.inverse_kinematics(vel_x, vel_y, W_z)
                # Calculate Error from feedback

                # Change the frame by using Rotation Matrix (If you find it required)

                # Calculate the required velocity of bot for the next iteration(s)
                
                # Find the required force vectors for individual wheels from it.(Inverse Kinematics)

                # Apply appropriate force vectors

                # Modify the condition to Switch to Next goal (given position in pixels instead of meters)
                        
                #If Condition is up to you
                
            if hb_controller.index == 46:
                hb_controller.flag = 1
            else:
                hb_controller.flag = 0

            if abs(x_err) <= 2 and abs(y_err) <= 2 and abs(theta_err) <= 0.1:           
                print("reached")

                # forces_left = Wrench()
                # forces_right = Wrench()
                # forces_rear = Wrench()

                # forces_left.force.y = 0.0
                # forces_right.force.y = 0.0
                # forces_rear.force.y = 0.0

                # hb_controller.right_wheel_force.publish(forces_right)
                # hb_controller.left_wheel_force.publish(forces_left)
                # hb_controller.rear_wheel_force.publish(forces_rear)

                # time.sleep(1.0)
                    
                 ############     DO NOT MODIFY THIS       #########
                hb_controller.index += 1
                if hb_controller.flag == 1 :
                    hb_controller.index = 0
                hb_controller.send_request(hb_controller.index)
                ####################################################


        # Spin once to process callbacks
        rclpy.spin_once(hb_controller)
    
    # Destroy the node and shut down ROS
    hb_controller.destroy_node()
    rclpy.shutdown()

# Entry point of the script
if __name__ == '__main__':
    main()
