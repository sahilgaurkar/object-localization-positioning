#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from message_filters import TimeSynchronizer, Subscriber
import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from tf2_ros import TransformBroadcaster
from math import atan2, cos, sin, sqrt, pi
import numpy as np
from geometry_msgs.msg import PoseStamped, TransformStamped, Point, Quaternion
from opencv_interfaces.srv import CaptureSource
from opencv_interfaces.msg import ObjectList
from tf_transformations import euler_from_quaternion, quaternion_from_euler


class PythonOpenCV(Node):
    def __init__(self):
        super().__init__("python_opencv")

        self.img_raw = None
        self.pointcloud = None
        self.capture_done = False

        # Time Synchronous Subscription to Image and PointCloud
        queue_size = 10
        self.ts = TimeSynchronizer(
            [
                Subscriber(self, Image, "/rgb_1/image_raw"),
                Subscriber(self, PointCloud2, "/depth_1/points"),
            ],
            queue_size,
        )

        self.ts_destination = TimeSynchronizer(
            [
                Subscriber(self, Image, "/rgb/image_raw"),
                Subscriber(self, PointCloud2, "/depth/points"),
            ],
            queue_size,
        )

        # self.ts.registerCallback(self.block_detection)

        # Create a Publisher
        self.pub_ = self.create_publisher(Image, "detection_image", 10)

        # OpenCV ROS2 bridge
        self.bridge = CvBridge()

        self.tf_broadcaster = TransformBroadcaster(self)

        # Create Service Server
        self.srv = self.create_service(
            CaptureSource, "get_source_block_pose", self.get_block_pose
        )

        # Create Service Server
        self.srv = self.create_service(
            CaptureSource, "get_destination_block_pose", self.get_destination_block_pose
        )

    def get_block_pose(self, request, response):
        self.get_logger().info("Incoming request")

        # Get block pose for each service call
        self.servicecall = True
        self.ts.registerCallback(self.block_detection)

        response.capture_sucessfull = True
        return response

    def get_destination_block_pose(self, request, response):
        self.get_logger().info("Incoming request")

        # Get block pose for each service call
        self.servicecall_1 = True
        self.ts_destination.registerCallback(self.block_detection_1)

        response.capture_sucessfull = True
        return response

    def get_3Dpt(self, label, centroid, angle, pointcloud_msg):
        pt3D = self.get_depth(centroid, pointcloud_msg)

        # print(f'{label}:')
        # print (f"Block centroid: {centroid}")
        # print (f"Block centroid 3D (wrt to Kinect): {pt3D}")
        # print (f"Block angle: {angle}")

        return pt3D

    def transformToWorldSource(self, pt3D, theta, child_frame):
        parent_frame = "camera_rgb_frame_1"

        try:
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = parent_frame
            t.child_frame_id = child_frame
            t.transform.translation.x = float(pt3D[2])
            t.transform.translation.y = float(pt3D[1])
            t.transform.translation.z = float(pt3D[0]) - 0.11

            q = self.quaternion_from_euler(0, 0, (pi / 2) - theta)

            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
        except:
            print(f"Unable to Publish transform: {child_frame}")
        finally:
            self.tf_broadcaster.sendTransform(t)

    def transformToWorldDestination(self, pt3D, theta, child_frame):
        parent_frame = "camera_rgb_frame"

        try:
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = parent_frame
            t.child_frame_id = child_frame
            t.transform.translation.x = float(pt3D[2])
            t.transform.translation.y = float(pt3D[1])
            t.transform.translation.z = float(pt3D[0]) - 0.11

            

            q = quaternion_from_euler( 0 , (pi/2), (pi + pi/2 + theta), 'rxyz')

            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
        except:
            print(f"Unable to Publish transform: {child_frame}")
        finally:
            self.tf_broadcaster.sendTransform(t)

    # def quaternion_from_euler(self, roll, pitch, yaw):
    #     """
    #     Converts euler roll, pitch, yaw to quaternion (w in last place)
    #     quat = [x, y, z, w]
    #     Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    #     """
    #     cy = cos(yaw * 0.5)
    #     sy = sin(yaw * 0.5)
    #     cp = cos(pitch * 0.5)
    #     sp = sin(pitch * 0.5)
    #     cr = cos(roll * 0.5)
    #     sr = sin(roll * 0.5)

    #     q = [0] * 4
    #     q[0] = cy * cp * cr + sy * sp * sr
    #     q[1] = cy * cp * sr - sy * sp * cr
    #     q[2] = sy * cp * sr + cy * sp * cr
    #     q[3] = sy * cp * cr - cy * sp * sr

    #     return q

    def get_depth(self, pt2D, data):
        # This function retrieves a 3D from a 2D image coordinate
        if (pt2D[1] >= data.height) or (pt2D[0] >= data.width):
            return -1

        data_out = pc2.read_points(
            data, field_names=None, skip_nans=False, uvs=(int(pt2D[0]), int(pt2D[1]))
        )
        # print(data_out)
        # for point in pc2.read_points(data, field_names=None, skip_nans=True, uvs=(int(pt2D[0]), int(pt2D[1]))):
        #     print("x: {}, y: {}, z: {}".format(*point))

        return (data_out[1][1], data_out[0][1], data_out[0][0])

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
        return [mask0 + mask1, "red"]

    def get_greenMask(self):
        lower = np.array([40, 10, 10])
        upper = np.array([70, 255, 255])
        mask = cv2.inRange(self.img_hsv, lower, upper)

        return [mask, "green"]

    def get_blueMask(self):
        lower = np.array([100, 10, 10])
        upper = np.array([140, 255, 255])
        mask = cv2.inRange(self.img_hsv, lower, upper)

        return [mask, "blue"]

    def get_yellowMask(self):
        lower = np.array([20, 180, 180])
        upper = np.array([30, 255, 255])
        mask = cv2.inRange(self.img_hsv, lower, upper)

        return [mask, "yellow"]

    def drawAxis(self, img, p_, q_, color, scale):
        p = list(p_)
        q = list(q_)

        ## [visualization1]
        angle = atan2(p[1] - q[1], p[0] - q[0])  # angle in radians
        hypotenuse = sqrt((p[1] - q[1]) * (p[1] - q[1]) + (p[0] - q[0]) * (p[0] - q[0]))

        # Here we lengthen the arrow by a factor of scale
        q[0] = p[0] - scale * hypotenuse * cos(angle)
        q[1] = p[1] - scale * hypotenuse * sin(angle)
        cv2.line(
            img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), color, 3, cv2.LINE_AA
        )

        # create the arrow hooks
        p[0] = q[0] + 9 * cos(angle + pi / 4)
        p[1] = q[1] + 9 * sin(angle + pi / 4)
        cv2.line(
            img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), color, 3, cv2.LINE_AA
        )

        p[0] = q[0] + 9 * cos(angle - pi / 4)
        p[1] = q[1] + 9 * sin(angle - pi / 4)
        cv2.line(
            img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), color, 3, cv2.LINE_AA
        )
        ## [visualization1]

    def getOrientation(self, pts, img):
        ## [pca]
        # Construct a buffer used by the pca analysis
        sz = len(pts)
        data_pts = np.empty((sz, 2), dtype=np.float64)
        for i in range(data_pts.shape[0]):
            data_pts[i, 0] = pts[i, 0, 0]
            data_pts[i, 1] = pts[i, 0, 1]

        # Perform PCA analysis
        mean = np.empty((0))
        mean, eigenvectors, eigenvalues = cv2.PCACompute2(data_pts, mean)

        # Store the center of the object
        cntr = (int(mean[0, 0]), int(mean[0, 1]))
        ## [pca]

        ## [visualization]
        # Draw the principal components
        cv2.circle(img, cntr, 3, (255, 0, 255), 2)
        p1 = (
            cntr[0] + 0.02 * eigenvectors[0, 0] * eigenvalues[0, 0],
            cntr[1] + 0.02 * eigenvectors[0, 1] * eigenvalues[0, 0],
        )
        p2 = (
            cntr[0] - 0.02 * eigenvectors[1, 0] * eigenvalues[1, 0],
            cntr[1] - 0.02 * eigenvectors[1, 1] * eigenvalues[1, 0],
        )
        self.drawAxis(img, cntr, p1, (255, 255, 0), 1)
        self.drawAxis(img, cntr, p2, (0, 0, 255), 5)

        angle = atan2(eigenvectors[0, 1], eigenvectors[0, 0])  # orientation in radians
        ## [visualization]

        # Label with the rotation angle
        label = "  Rotation Angle: " + str(-int(np.rad2deg(angle)) - 90) + " degrees"
        textbox = cv2.rectangle(
            img,
            (cntr[0], cntr[1] - 25),
            (cntr[0] + 250, cntr[1] + 10),
            (255, 255, 255),
            -1,
        )
        cv2.putText(
            img,
            label,
            (cntr[0], cntr[1]),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 0, 0),
            1,
            cv2.LINE_AA,
        )

        return angle

    def get_contour(self, img_masked):
        edges = cv2.Canny(img_masked, 400, 650)
        cnts, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        # print (cnts)
        contour = []
        for c in cnts:
            perimeter = cv2.arcLength(c, True)
            # print ("Perimeter: " + str(perimeter))
            if perimeter >= 90 and perimeter <= 350:
                contour.append(c)
                break

        return contour

    def get_pose(self, contour):
        for c in contour:
            (x, y), (MA, ma), temp = cv2.fitEllipse(c)
            angle = self.getOrientation(c, self.img_raw.copy())

        return (x, y), angle

    def block_detection(self, image_msg, pointcloud_msg):
        assert image_msg.header.stamp == pointcloud_msg.header.stamp

        if self.servicecall:
            self.get_logger().info("Processing request")

            # Convert ROS Image message to OpenCV image
            self.img_raw = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
            self.img_hsv = cv2.cvtColor(self.img_raw, cv2.COLOR_BGR2HSV)

            masks = []
            masks.append(self.get_redMask())
            masks.append(self.get_greenMask())
            masks.append(self.get_blueMask())
            masks.append(self.get_yellowMask())

            self.objects = []
            for mask in masks:
                color_mask = mask[0]
                label = mask[1]

                # set my output img to zero everywhere except my mask
                img_masked = self.img_raw.copy()
                img_masked[np.where(color_mask == 0)] = 0

                contour = self.get_contour(img_masked)
                if contour == []:
                    continue
                centroid, angle = self.get_pose(contour)
                pt3D = self.get_3Dpt(label, centroid, angle, pointcloud_msg)
                self.objects.append([pt3D, angle, label])
                self.capture_done = True
                # self.transformToWorld(pt3D, angle, label)

            self.servicecall = False

        if len(self.objects) != 0:
            for object in self.objects:
                self.transformToWorldSource(object[0], object[1], object[2])

    def block_detection_1(self, image_msg, pointcloud_msg):
        assert image_msg.header.stamp == pointcloud_msg.header.stamp

        if self.servicecall_1:
            self.get_logger().info("Processing request")

            # Convert ROS Image message to OpenCV image
            self.img_raw = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
            self.img_hsv = cv2.cvtColor(self.img_raw, cv2.COLOR_BGR2HSV)

            masks = []
            masks.append(self.get_redMask())
            masks.append(self.get_greenMask())
            masks.append(self.get_blueMask())
            masks.append(self.get_yellowMask())

            self.objects_1 = []
            for mask in masks:
                color_mask = mask[0]
                label = mask[1]

                # set my output img to zero everywhere except my mask
                img_masked = self.img_raw.copy()
                img_masked[np.where(color_mask == 0)] = 0

                contour = self.get_contour(img_masked)
                if contour == []:
                    continue
                centroid, angle = self.get_pose(contour)
                pt3D = self.get_3Dpt(label, centroid, angle, pointcloud_msg)
                self.objects_1.append([pt3D, angle, label + '_1'])
                self.capture_done = True
                # self.transformToWorld(pt3D, angle, label)

            self.servicecall_1 = False

        if len(self.objects_1) != 0:
            for object in self.objects_1:
                self.transformToWorldDestination(object[0], object[1], object[2])


def main(args=None):
    rclpy.init(args=args)
    node = PythonOpenCV()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
