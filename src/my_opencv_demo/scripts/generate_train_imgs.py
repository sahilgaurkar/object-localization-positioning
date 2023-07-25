#!/usr/bin/env python3

import rclpy
import os
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
        self.create_subscription(Image, "/rgb/image_raw", self.subscribe_image, 10)
        # Create a Publisher
        self.pub_ = self.create_publisher(Image, "py_opencv", 10)
        self.index_ = 0

        # OpenCV ROS2 bridge
        self.bridge = CvBridge()

    def subscribe_image(self, msg):  # Self and the msg we will receive
        # Display the message on the console
        self.get_logger().info("Receiving video frame")

        # Convert ROS Image message to OpenCV image
        self.current_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        #Resize Images
        dim = (256, 256)
        resized = cv2.resize(self.current_frame, dim, interpolation = cv2.INTER_AREA)

        # Display image
        cv2.imshow("camera", resized)

        path = '/home/sahil/result/'
        cv2.imwrite(os.path.join(path, f'img_{self.index_}.png'), resized)


        print(os.path.join(path, f'img_{self.index_}.png'))
        self.index_ += 1
        cv2.waitKey(5)


        # Convert and Publish
        self.pub_.publish(self.bridge.cv2_to_imgmsg(self.current_frame, "bgr8"))


def main(args=None):
    rclpy.init(args=args)
    node = PythonOpenCV()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
