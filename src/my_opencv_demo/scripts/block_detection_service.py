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

        self.srv = self.create_service(BlockPose, "get_block_pose", self.get_block_pose)

    def get_block_pose(self, request, response):
        name = 'red'
        point = Point(x=0.0, y=0.0, z=0.0)
        orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=0.0)

        object = ObjectList(name=name, center=point, orientation=orientation)

        response.objects.append(object)
        return response




def main():
    rclpy.init()
    opencv_service = PythonOpenCVService()
    rclpy.spin(opencv_service)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
