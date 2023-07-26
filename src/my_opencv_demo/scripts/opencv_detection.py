#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class PythonOpenCV(Node):
    def __init__(self):
        super().__init__("python_opencv")
        # Initilize local variables
        # self.current_frame = 0.0
        # Create a Subscriber
        self.create_subscription(Image, "/rgb/image_raw", self.subscribe_image, 10)
        # Create a Publisher
        self.pub_ = self.create_publisher(Image, "py_opencv", 10)

        # OpenCV ROS2 bridge
        self.bridge = CvBridge()

    def subscribe_image(self, msg):  # Self and the msg we will receive
        # Display the message on the console
        self.get_logger().info("Receiving video frame")

        # Convert ROS Image message to OpenCV image
        self.current_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        img = self.current_frame
        img_hsv=cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # lower red mask (0-10)
        lower_red = np.array([0,50,50])
        upper_red = np.array([10,255,255])
        mask0 = cv2.inRange(img_hsv, lower_red, upper_red)

        # upper mask (170-180)
        lower_red = np.array([170,50,50])
        upper_red = np.array([180,255,255])
        mask1 = cv2.inRange(img_hsv, lower_red, upper_red)

        # join my masks
        mask = mask0+mask1

        # set my output img to zero everywhere except my mask
        r_img = img.copy()
        r_img[np.where(mask==0)] = 0


        lower_green = np.array([40, 10, 10])
        upper_green = np.array([70, 255, 255])

        mask = cv2.inRange(img_hsv, lower_green, upper_green)

        # set my output img to zero everywhere except my mask
        g_img = img.copy()
        g_img[np.where(mask==0)] = 0


        lower_blue = np.array([100, 10, 10])
        upper_blue = np.array([140, 255, 255])

        mask = cv2.inRange(img_hsv, lower_blue, upper_blue)

        # set my output img to zero everywhere except my mask
        b_img = img.copy()
        b_img[np.where(mask==0)] = 0



        lower_yellow = np.array([20, 180, 180])
        upper_yellow = np.array([30, 255, 255])

        mask = cv2.inRange(img_hsv, lower_yellow, upper_yellow)

        # set my output img to zero everywhere except my mask
        y_img = img.copy()
        y_img[np.where(mask==0)] = 0

        


        # Display image
        cv2.imshow("camera", y_img, edges)
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
