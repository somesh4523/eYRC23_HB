from cv_bridge import CvBridge
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import Image

import rclpy
from rclpy.node import Node
import cv2
import math
import cv2.aruco as aruco
import numpy as np

class ArUcoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        self.cv_bridge = CvBridge()
        self.publisher1 = self.create_publisher(Pose2D, '/pen1_pose', 10)
        self.publisher2 = self.create_publisher(Pose2D, '/pen2_pose', 10)
        self.publisher3 = self.create_publisher(Pose2D, '/pen3_pose', 10)
        self.subscription = self.create_subscription( Image, '/image_rect_color', self.image_callback, 10)

    def image_callback(self, msg):
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv_image = cv_image[45:375, 147:424]
        cv_image = cv2.resize(cv_image, (500, 500))

        gray_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        arucoDict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        parameters =  aruco.DetectorParameters()
        corners, ids, _ = aruco.detectMarkers(gray_img, arucoDict, parameters=parameters)
        # print(ids)

        if ids is not None:  # Make sure at least one marker was found
            for (marker_corner, marker_id) in zip(corners, ids):
                if marker_id[0] == 0:   # Check for the robot's marker ID
                    corners = marker_corner.reshape((4, 2))
                    top_left, top_right, bottom_right, bottom_left = corners
                
                    center_x = (top_left[0] + bottom_right[0]) / 2.0
                    center_y = (top_right[1] + bottom_left[1]) / 2.0

                    center_xp = (top_left[0] + bottom_right[0]) / 2.0 + 10
                    center_yp = (top_right[1] + bottom_left[1]) / 2.0 + 17

                    dx = (top_right[0] + bottom_right[0])/2.0 - center_x
                    dy = (top_right[1] + bottom_right[1])/2.0 - center_y
                    
                    angle_rad = np.arctan2(dy, dx)
                
                    print("Aruco id = 0 " + str(center_xp), center_yp, angle_rad)
                    print(str(corners))

                    # Publish the results
                    data = Pose2D()
                    data.x = center_xp
                    data.y = center_yp
                    data.theta = float(angle_rad)
                    
                    self.publisher1.publish(data)

                if marker_id[0] == 1:   # Check for the robot's marker ID
                    corners = marker_corner.reshape((4, 2))
                    top_left, top_right, bottom_right, bottom_left = corners
                
                    center_x = (top_left[0] + bottom_right[0]) / 2.0
                    center_y = (top_right[1] + bottom_left[1]) / 2.0

                    # adding pen offset
                    center_xp = (top_left[0] + bottom_right[0]) / 2.0 + 10
                    center_yp = (top_right[1] + bottom_left[1]) / 2.0 + 17

                    dx = (top_right[0] + bottom_right[0])/2.0 - center_x
                    dy = (top_right[1] + bottom_right[1])/2.0 - center_y
                
                    angle_rad = np.arctan2(dy, dx)
                
                    print("Aruco id = 1 " + str(center_xp), center_yp, angle_rad)
                    print(str(corners))


                    # Publish the results
                    data = Pose2D()
                    data.x = center_xp
                    data.y = center_yp
                    data.theta = float(angle_rad)
                    
                    self.publisher2.publish(data)

                if marker_id[0] == 2:   # Check for the robot's marker ID
                    corners = marker_corner.reshape((4, 2))
                    top_left, top_right, bottom_right, bottom_left = corners
                
                    center_x = (top_left[0] + bottom_right[0]) / 2.0
                    center_y = (top_right[1] + bottom_left[1]) / 2.0


                    center_xp = (top_left[0] + bottom_right[0]) / 2.0 + 10
                    center_yp = (top_right[1] + bottom_left[1]) / 2.0 + 17

                    dx = (top_right[0] + bottom_right[0])/2.0 - center_x
                    dy = (top_right[1] + bottom_right[1])/2.0 - center_y
                    
                    angle_rad = np.arctan2(dy, dx)
                
                    print("Aruco id = 2 " + str(center_xp), center_yp, angle_rad)
                    print(str(corners))

                    # Publish the results
                    data = Pose2D()
                    data.x = center_xp
                    data.y = center_yp
                    data.theta = float(angle_rad)
                    
                    self.publisher3.publish(data)

        cv2.imshow("Image", cv_image)
        cv2.waitKey(2)

def main(args=None):
    rclpy.init(args=args)
    aruco_detector = ArUcoDetector()
    rclpy.spin(aruco_detector)
    aruco_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()