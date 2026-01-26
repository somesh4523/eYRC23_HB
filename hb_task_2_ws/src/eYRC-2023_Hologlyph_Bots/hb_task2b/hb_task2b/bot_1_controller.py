#! /usr/bin/env python3

'''
*******************************
*
*        		===============================================
*           		Hologlyph Bots (HB) Theme (eYRC 2023-24)
*        		===============================================
*
*  This script is to implement Task 2B of Hologlyph Bots (HB) Theme (eYRC 2023-24).
*  
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or 
*  breach of the terms of this agreement.
*
*******************************
'''


# Team ID:		[ Team-ID ]
# Author List:		[ Names of team members worked on this file separated by Comma: Name1, Name2, ... ]
# Filename:		feedback.py
# Functions:
# [ Comma separated list of functions in this file ]
# Nodes:		Add your publishing and subscribing node


################### IMPORT MODULES #######################

import rclpy
from rclpy.node import Node
import time
import math
import numpy as np
from tf_transformations import euler_from_quaternion
from my_robot_interfaces.msg import Goal

from geometry_msgs.msg import Wrench
from geometry_msgs.msg import Pose2D


class HBController(Node):
    def __init__(self):
        super().__init__('hb_controller')

        # Initialise the required variables
        self.bot_1_x = 0.0
        self.bot_1_y = 0.0
        self.bot_1_theta = 0.0

        self.xPose = 0.0
        self.yPose = 0.0
        self.thetaPose = 0.0

        self.Kp = 3.75
        self.Kw = 2.4

        # Initialze Publisher and Subscriber
        # NOTE: You are strictly NOT-ALLOWED to use "cmd_vel" or "odom" topics in this task
        # Use the below given topics to generate motion for the robot.
        #   /hb_bot_1/left_wheel_force,
        #   /hb_bot_1/right_wheel_force,
        #   /hb_bot_1/left_wheel_force

        self.left_wheel_force = self.create_publisher(
            Wrench, '/hb_bot_1/left_wheel_force', 10)
        self.right_wheel_force = self.create_publisher(
            Wrench, '/hb_bot_1/right_wheel_force', 10)
        self.rear_wheel_force = self.create_publisher(
            Wrench, '/hb_bot_1/rear_wheel_force', 10)

        # Similar to this you can create subscribers for hb_bot_2 and hb_bot_3
        self.subscription = self.create_subscription(
            Goal,
            'hb_bot_1/goal',
            self.goalCallBack,  # Callback function to handle received messages
            10  # QoS profile, here it's 10 which means a buffer size of 10 messages
        )

        self.subscription_pose = self.create_subscription(
            Pose2D, '/detected_aruco_1', self.aruco_callback, 10)

        self.subscription  # Prevent unused variable warning
        # For maintaining control loop rate.
        self.rate = self.create_rate(100)
        self.index = 0

    def inverse_kinematics(self, x_vel, y_vel, theta_vel):
        ############ ADD YOUR CODE HERE ############

        # INSTRUCTIONS & HELP :
        # -> Use the target velocity you calculated for the robot in previous task, and
        # Process it further to find what proportions of that effort should be given to 3 individuals wheels !!
        # Publish the calculated efforts to actuate robot by applying force vectors on provided topics
        ##################################################################

        d = 0.67999

        rotation_matrix = np.array([[-0.5,  -np.sqrt(3)/2,  d],
                                    [-0.5,   np.sqrt(3)/2,  d],
                                    [1,   0,  d]])

        [u1, u2, u3] = np.dot(rotation_matrix, [x_vel, y_vel, theta_vel]) 

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

    def goalCallBack(self, msg):
        self.bot_1_x = msg.x
        self.bot_1_y = msg.y
        self.bot_1_theta = np.radians(msg.theta)

    def aruco_callback(self, data):
        self.xPose = data.x
        self.yPose = data.y
        self.thetaPose = data.theta


def main(args=None):
    rclpy.init(args=args)

    hb_controller = HBController()
    x_goal1 = 0.0
    y_Goal1 = 0.0
    theta1 = 0.0

    while x_goal1 == 0.0:
        rclpy.spin_once(hb_controller)
        x_goal1 = hb_controller.bot_1_x
        y_Goal1 = hb_controller.bot_1_y
        theta1 = hb_controller.bot_1_theta


    if x_goal1 != 0.0:
        # Main loop
        while rclpy.ok():

            x_goal = x_goal1[hb_controller.index]
            y_goal = y_Goal1[hb_controller.index]
            theta_goal = theta1

            ####################################################
            # print("x Goal " + str(x_goal), "y Goal " + str(y_goal), "theta Goal " + str(theta_goal))
            x_err = x_goal - hb_controller.xPose
            y_err = y_goal - hb_controller.yPose
            theta_err = theta_goal - hb_controller.thetaPose
            # print("x_err = " + str(x_err), y_err, theta_err)

            x_err_body = math.cos(hb_controller.thetaPose) * x_err - math.sin(hb_controller.thetaPose) * y_err
            y_err_body = math.sin(hb_controller.thetaPose) * x_err + math.cos(hb_controller.thetaPose) * y_err

            vel_x = hb_controller.Kp * x_err_body
            vel_y = -hb_controller.Kp * y_err_body
            W_z = -hb_controller.Kw * theta_err
            # Safety Check
            # make sure the velocities are within a range.
            if vel_x > 13:
                vel_x = 13
            if vel_x < -13:
                vel_x = -13
            if vel_y > 13:
                vel_y = 13
            if vel_y < -13:
                vel_y = -13
            if W_z > 13:
                W_z = 13
            if W_z < -13:
                W_z = -13

            hb_controller.inverse_kinematics(vel_x, vel_y, W_z)

            if abs(x_err) <= 2 and abs(y_err) <= 2 and abs(theta_err) <= 0.1 and hb_controller.index < 100:
                print("reached")
                hb_controller.index += 1

            elif hb_controller.index == 100 :
                rclpy.get_logger().info("Shape completed !!")
                break
            


        # Spin once to process callbacks
            rclpy.spin_once(hb_controller)

    else:
        rclpy.spin_once(hb_controller)

    # Destroy the node and shut down ROS
    hb_controller.destroy_node()
    rclpy.shutdown()


# Entry point of the script
if __name__ == '__main__':
    main()
