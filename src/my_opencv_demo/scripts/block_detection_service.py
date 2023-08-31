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
from opencv_interfaces.srv import BlockPose
from opencv_interfaces.msg import ObjectList


class PythonOpenCVService(Node):
    def __init__(self):
        super().__init__("opencv_service")
        

        self.subscription = self.create_subscription(
            Image,
            "/rgb/image_raw",
            self.block_detection,
            10)
        

        # OpenCV ROS2 bridge
        self.bridge = CvBridge()



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

    def block_detection(self, image_msg):

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

        mask = masks[0][0] + masks[1][0] + masks[2][0]+ masks[3][0]

        color_mask = mask
        

        # set my output img to zero everywhere except my mask
        img_masked = self.img_raw.copy()
        img_masked[np.where(color_mask == 0)] = 0

        edges = cv2.Canny(img_masked, 400, 650)
        cnts, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        # print (cnts)
        contour = []
        for c in cnts:
            perimeter = cv2.arcLength(c, True)
            # print ("Perimeter: " + str(perimeter))
            if perimeter >= 90 and perimeter <= 350:
                contour.append(c)
                # break

        out_img = self.img_raw.copy()
        out_ellipse = []
        for c in contour:
            (x, y), (MA, ma), temp = cv2.fitEllipse(c)
            angle = self.getOrientation(c, self.img_raw.copy())
            out_ellipse.append(cv2.fitEllipse(c))

        
        for each_ellipse in out_ellipse:
            cv2.ellipse(out_img, each_ellipse, (255, 0, 0), 2)

        label = 'shadow_masked'
        cv2.imshow(f'{label}', img_masked)
        cv2.imwrite(f'{label}.jpg', img_masked)
        cv2.waitKey(3)


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

        # label = 'Distorted Angle'
        # cv2.imshow(f'{label}', img)
        # cv2.imwrite(f'{label}.jpg', img)

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
        self.drawAxis(img, cntr, p1, (255, 255, 0), 10)
        self.drawAxis(img, cntr, p2, (0, 0, 255), 10)

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


def main():
    rclpy.init()
    opencv_service = PythonOpenCVService()
    rclpy.spin(opencv_service)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
