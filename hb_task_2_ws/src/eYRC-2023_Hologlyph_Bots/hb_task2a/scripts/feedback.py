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
        super().__init__('ar_uco_detector')
        self.cv_bridge = CvBridge()
        self.publisher_ = self.create_publisher(Pose2D, '/detected_aruco', 10)
        self.subscription = self.create_subscription( Image, '/camera/image_raw', self.image_callback, 10)

    def image_callback(self, msg):
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        arucoDict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        parameters =  aruco.DetectorParameters()
        corners, ids, _ = aruco.detectMarkers(gray_img, arucoDict, parameters=parameters)

        if ids is not None:  # Make sure at least one marker was found
            for (marker_corner, marker_id) in zip(corners, ids):
                if marker_id[0] == 1:   # Check for the robot's marker ID
                    corners = marker_corner.reshape((4, 2))
                    top_left, top_right, bottom_right, bottom_left = corners
                    # print(f"Top left corner of marker {marker_id[0]}: {top_left}")
                    # print(f"Top right corner of marker {marker_id[0]}: {top_right}")
                    # print(f"Bottom right corner of marker {marker_id[0]}: {bottom_right}")
                    # print(f"Bottom left corner of marker {marker_id[0]}: {bottom_left}")

                    # Compute the center of the marker
                    # center_x = sum([coord[0][0] for coord in marker_corner]) / 4
                    # center_y = sum([coord[0][1] for coord in marker_corner]) / 4

                    center_x = (top_left[0] + bottom_right[0]) / 2.0
                    center_y = (top_right[1] + bottom_left[1]) / 2.0

                    # print(center_x ,center_y)

                    # Compute orientation of the marker (Assuming 0 when facing up)
                    # dx = marker_corner[0][0][0] - marker_corner[0][1][0]
                    # dy = marker_corner[0][0][1] - marker_corner[0][1][1]

                    # dx = top_right[0] - top_left[0]
                    # dy = top_right[1] - top_left[1]
                    dx = (top_right[0] + bottom_right[0])/2.0 - center_x
                    dy = (top_right[1] + bottom_right[1])/2.0 - center_y
                    
                
                    angle_rad = np.arctan2(dy, dx)
                    # angle_rad = 0.0

                    print(center_x, center_y, angle_rad)

                    # Publish the results
                    data = Pose2D()
                    data.x = center_x
                    data.y = center_y
                    data.theta = float(angle_rad)
                    
                    self.publisher_.publish(data)

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