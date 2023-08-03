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
from message_filters import TimeSynchronizer, Subscriber, ApproximateTimeSynchronizer
from rclpy.qos import qos_profile_sensor_data
import math


class PythonOpenCV(Node):
    def __init__(self):
        super().__init__("python_opencv")

        self.img_raw = None
        self.pointcloud = None

        # Time Synchronous Subscription to Image and PointCloud
        queue_size = 10
        self.ts = TimeSynchronizer(
            [
                Subscriber(self, Image, "/rgb/image_raw"),
                Subscriber(self, PointCloud2, "/depth/points"),
            ],
            queue_size,
        )

        self.ts.registerCallback(self.callback)

        # Create a Publisher
        self.pub_ = self.create_publisher(Image, "detection_image", 10)

        # OpenCV ROS2 bridge
        self.bridge = CvBridge()


        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)





    def callback(self, image_msg, pointcloud_msg):
        assert image_msg.header.stamp == pointcloud_msg.header.stamp

        # self.get_logger().info("Received image and pointcloud")

        self.block_detection(image_msg)
        self.detector(pointcloud_msg)

    def block_detection(self, image_msg):
        # This method will detect block of color and their centroids

        # Convert ROS Image message to OpenCV image
        self.img_raw = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        self.img_hsv = cv2.cvtColor(self.img_raw, cv2.COLOR_BGR2HSV)

        mask = (
            self.get_redMask()
            + self.get_greenMask()
            + self.get_blueMask()
            + self.get_yellowMask()
        )

        # set my output img to zero everywhere except my mask
        img_masked = self.img_raw.copy()
        img_masked[np.where(mask == 0)] = 0

        edges = cv2.Canny(img_masked, 5, 50)
        cnts, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # print (cnts)
        outCnt = []
        for c in cnts:
            perimeter = cv2.arcLength(c, True)
            # print ("Perimeter: " + str(perimeter))
            if perimeter >= 30 and perimeter <= 300:
                outCnt.append(c)

        self.centroids = []
        self.angles = []
        self.out_img = self.img_raw.copy()
        out_ellipse = []
        for c in outCnt:
            try:
                (x, y), (MA, ma), theta = cv2.fitEllipse(c)
                self.centroids.append((x, y))
                self.angles.append(theta)
                out_ellipse.append(cv2.fitEllipse(c))
                # Printing and displaying results
                cv2.drawContours(self.out_img, [c], -1, (0, 255, 0), 1)
                # print (f"Centroid: {(x,y)}")
                # print (f"Angle: {theta}")
            except:
                print("Something went wrong")
                continue

        for each_ellipse in out_ellipse:
            cv2.ellipse(self.out_img, each_ellipse, (255, 0, 0), 2)

        # cv2.imshow("camera", self.out_img)
        # cv2.waitKey(1)

    def detector(self, pointcloud_msg):
        print (f"Number of blocks detected: {len(self.centroids)}")

        pt2D = self.centroids[0]
        theta = self.angles[0]

        # print(pt2D)

        pt3D = self.get_depth(pt2D, pointcloud_msg)

        print (f"Block centroid: {pt2D}")
        print (f"Block centroid 3D (wrt to Kinect): {pt3D}")
        print (f"Block angle: {theta}")

        self.transformToWorld(pt3D)

    def transformToWorld(self, pt3D):
        from_frame_rel = "world"
        to_frame_rel = "camera_depth_frame"

        transform = self.tf_buffer.lookup_transform(
            to_frame_rel, from_frame_rel, rclpy.time.Time()
        )

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "camera_depth_frame"
        t.child_frame_id = "red"
        t.transform.translation.x = float(pt3D[0])
        t.transform.translation.y = float(pt3D[1])
        t.transform.translation.z = float(pt3D[2])

        q = self.quaternion_from_euler(math.radians(0), math.radians(-90), math.radians(0))
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)

    def quaternion_from_euler(self, roll, pitch, yaw):
        """
        Converts euler roll, pitch, yaw to quaternion (w in last place)
        quat = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = [0] * 4
        q[0] = cy * cp * cr + sy * sp * sr
        q[1] = cy * cp * sr - sy * sp * cr
        q[2] = sy * cp * sr + cy * sp * cr
        q[3] = sy * cp * cr - cy * sp * sr

        return q

    def get_depth(self, pt2D, data):
        # This function retrieves a 3D from a 2D image coordinate
        if (pt2D[0] >= data.height) or (pt2D[1] >= data.width):
            return -1

        data_out = pc2.read_points(data, field_names=None, skip_nans=False, uvs=[int(pt2D[0]), int(pt2D[1])])
        return data_out[1]

    def get_redMask(self):
        # lower red mask (0-10)
        lower = np.array([0, 50, 50])
        upper = np.array([10, 255, 255])
        mask0 = cv2.inRange(self.img_hsv, lower, upper)

        # upper mask (170-180)
        lower = np.array([170, 50, 50])
        upper = np.array([180, 255, 255])
        mask1 = cv2.inRange(self.img_hsv, lower, upper)

        # join my masks
        return mask0 + mask1

    def get_greenMask(self):
        lower = np.array([40, 10, 10])
        upper = np.array([70, 255, 255])
        mask = cv2.inRange(self.img_hsv, lower, upper)

        return mask

    def get_blueMask(self):
        lower = np.array([100, 10, 10])
        upper = np.array([140, 255, 255])
        mask = cv2.inRange(self.img_hsv, lower, upper)

        return mask

    def get_yellowMask(self):
        lower = np.array([20, 180, 180])
        upper = np.array([30, 255, 255])
        mask = cv2.inRange(self.img_hsv, lower, upper)

        return mask


def main(args=None):
    rclpy.init(args=args)
    node = PythonOpenCV()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
