import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import cv_bridge

class camera_node(Node):
    def __init__(self):
        super().__init__("Camera_output_node")
        self.subscription = self.create_subscription(
            Image,
            '/image_rect_color',
            self.image_callback,
            10)
        self.cv_bridge = cv_bridge.CvBridge()
        self.publisher = self.create_publisher(Image, '/output_image', 2)  # Create the publisher

    
    def image_callback(self,msg):
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # cv_image = cv2.resize(cv_image, (500, 500))
            # x, y, w, h = 184, 42, 300, 300
            # cv_image = cv_image[y:y+h, x:x+w]
            cv_image = cv_image[40:375, 145:426]
            cv_image = cv2.resize(cv_image, (500, 500))
            image_message = self.cv_bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')

            self.publisher.publish(image_message)



        except Exception as e:
            self.get_logger().error('Error converting ROS Image to OpenCV image: %s' % str(e))
            return
        
        
        # cv2.imshow(region_of_interest)
        cv2.imshow('output video',cv_image)
     
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)

    output = camera_node()

    rclpy.spin(output)

    output.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
