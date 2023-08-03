#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import PoseStamped, TransformStamped
import math
from tf2_ros.buffer import Buffer


def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci * ck
    cs = ci * sk
    sc = si * ck
    ss = si * sk

    q = np.empty((4,))
    q[0] = cj * sc - sj * cs
    q[1] = cj * ss + sj * cc
    q[2] = cj * cs - sj * sc
    q[3] = cj * cc + sj * ss

    return q


class PythonOpenCV(Node):
    def __init__(self):
        super().__init__("python_opencv")

        self.current_frame = None
        self.current_point = None

        # Create Subscriber
        self.create_subscription(PointCloud2, "/depth/points", self.subscribe_depth, 10)
        self.create_subscription(Image, "/rgb/image_raw", self.subscribe_image, 10)

        # Create a Publisher
        self.pub_ = self.create_publisher(Image, "py_opencv", 10)

        # OpenCV ROS2 bridge
        self.bridge = CvBridge()

        # Declare and acquire `target_frame` parameter
        # self.target_frame = self.declare_parameter(
        #   'world', 'camera_rgb_optical_frame').get_parameter_value().string_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)


    def custom_transform(self, pt3D):
        from_frame_rel = "world"
        to_frame_rel = "camera_rgb_optical_frame"

        transform = self.tf_buffer.lookup_transform(
            to_frame_rel, from_frame_rel, rclpy.time.Time()
        )

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "camera_rgb_frame"
        t.child_frame_id = "red"
        t.transform.translation.x = float(pt3D[0])
        t.transform.translation.y = -float(pt3D[2])
        t.transform.translation.z = -float(pt3D[1])

        q = quaternion_from_euler(90, 0, 0)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)

    def get_depth(self, width, height, data):
        # This function retrieves a 3D from a 2D image coordinate
        if (height >= data.height) or (width >= data.width):
            return -1

        data_out = pc2.read_points(
            data, field_names=None, skip_nans=False, uvs=[width, height]
        )
        int_data = data_out[1]
        return int_data

    def block_detection(self):
        img = self.current_frame
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # lower red mask (0-10)
        lower_red = np.array([0, 50, 50])
        upper_red = np.array([10, 255, 255])
        mask0 = cv2.inRange(img_hsv, lower_red, upper_red)

        # upper mask (170-180)
        lower_red = np.array([170, 50, 50])
        upper_red = np.array([180, 255, 255])
        mask1 = cv2.inRange(img_hsv, lower_red, upper_red)

        # join my masks
        mask = mask0 + mask1

        # set my output img to zero everywhere except my mask
        r_img = img.copy()
        r_img[np.where(mask == 0)] = 0

        lower_green = np.array([40, 10, 10])
        upper_green = np.array([70, 255, 255])

        mask = cv2.inRange(img_hsv, lower_green, upper_green)

        # set my output img to zero everywhere except my mask
        g_img = img.copy()
        g_img[np.where(mask == 0)] = 0

        lower_blue = np.array([100, 10, 10])
        upper_blue = np.array([140, 255, 255])

        mask = cv2.inRange(img_hsv, lower_blue, upper_blue)

        # set my output img to zero everywhere except my mask
        b_img = img.copy()
        b_img[np.where(mask == 0)] = 0

        lower_yellow = np.array([20, 180, 180])
        upper_yellow = np.array([30, 255, 255])

        mask = cv2.inRange(img_hsv, lower_yellow, upper_yellow)

        # set my output img to zero everywhere except my mask
        y_img = img.copy()
        y_img[np.where(mask == 0)] = 0

        edges = cv2.Canny(r_img, 5, 50)
        cnts, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        outCnt = []
        for c in cnts:
            perimeter = cv2.arcLength(c, True)
            # print ("Perimeter: " + str(perimeter))
            if perimeter >= 30 and perimeter <= 300:
                outCnt.append(c)

        self.centroid = []
        self.angle = []
        self.out_img = img.copy()
        out_ellipse = []
        for c in outCnt:
            (x, y), (MA, ma), angle = cv2.fitEllipse(c)
            self.centroid.append((x, y))

            out_ellipse.append(cv2.fitEllipse(c))

            # Printing and displaying results
            cv2.drawContours(self.out_img, [c], -1, (0, 255, 0), 1)

            # print ("Centroid: " + str((x,y)))
            # print ("Angle: " + str(angle)) # With respect to y in the image.

        # cv2.imshow("camera", self.out_img)
        # cv2.waitKey(1)

    def detector(self):
        pt2D = self.centroid[0]
        theta = self.angle
        # print ("Centroid: " + str(int(pt2D[0])) + " " + str(int(pt2D[1])))

        pt3D = self.get_depth(int(pt2D[0]), int(pt2D[1]), self.current_point)

        print("Block centroid: " + str(pt2D))
        print("Block centroid 3D (wrt to Kinect): " + str(pt3D))
        print("Block angle: " + str(theta))  # With respect to y in the image

        self.custom_transform(pt3D)

    def subscribe_depth(self, msg):
        # Callback for the point cloud topic
        self.current_point = msg

    def subscribe_image(self, msg):  # Self and the msg we will receive
        # Display the message on the console
        # self.get_logger().info("Receiving video frame")
        # Convert ROS Image message to OpenCV image
        self.current_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        self.block_detection()

        if self.current_frame is not None and self.current_point is not None:
            self.detector()


def main(args=None):
    rclpy.init(args=args)
    node = PythonOpenCV()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
