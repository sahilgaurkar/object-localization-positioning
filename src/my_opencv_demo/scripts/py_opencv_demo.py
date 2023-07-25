#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class PythonOpenCV(Node):
    def __init__(self):
        super().__init__("python_opencv")
        # Initilize local variables
        # self.current_frame = 0.0
        # Create a Subscriber
        self.create_subscription(Image, "/camera/image_raw", self.subscribe_image, 10)
        # Create a Publisher
        self.pub_ = self.create_publisher(Image, "py_opencv", 10)

        # OpenCV ROS2 bridge
        self.bridge = CvBridge()

    def subscribe_image(self, msg):  # Self and the msg we will receive
        # Display the message on the console
        self.get_logger().info("Receiving video frame")

        # Convert ROS Image message to OpenCV image
        self.current_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        (rows, cols, channels) = self.current_frame.shape
        if cols > 60 and rows > 60:
            cv2.circle(self.current_frame, (50, 50), 10, 255)
        # Display image
        cv2.imshow("camera", self.current_frame)
        cv2.waitKey(1)

        # Convert and Publish
        self.pub_.publish(self.bridge.cv2_to_imgmsg(self.current_frame, "bgr8"))


def main(args=None):
    rclpy.init(args=args)
    node = PythonOpenCV()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
