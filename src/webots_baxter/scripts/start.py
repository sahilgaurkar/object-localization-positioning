#!/usr/bin/env python3
import sys
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Quaternion, TransformStamped
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf_transformations import euler_from_quaternion, quaternion_from_euler

from opencv_interfaces.srv import BlockPose
from opencv_interfaces.msg import ObjectList
from opencv_interfaces.srv import GripperCmd
from opencv_interfaces.srv import TargetPose
from opencv_interfaces.srv import CaptureSource


class MainTask(Node):
    def __init__(self):
        super().__init__("main_task_node")

        self._loop_rate = self.create_rate(10)

        self.red_s_captured = False
        self.red_d_captured = False

        self.cli = self.create_client(CaptureSource, "get_source_block_pose")

        self.cli_d = self.create_client(CaptureSource, "get_destination_block_pose")

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
        self.req = CaptureSource.Request()

        while not self.cli_d.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
        self.req_d = CaptureSource.Request()

        self.source_subscription = self.create_subscription(
            ObjectList, "source/objects", self.source_callback, 10
        )

        self.destination_subscription = self.create_subscription(
            ObjectList, "destination/objects", self.destination_callback, 10
        )

        self.moveit_cli = self.create_client(TargetPose, "moveit_service")
        while not self.moveit_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
        self.req_moveit = TargetPose.Request()

        self.gripper_cli = self.create_client(GripperCmd, "gripper_service")
        while not self.gripper_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
        self.req_gripper = GripperCmd.Request()

    def send_request(self, capture):
        self.req.capture_image = capture
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def send_request_d(self, capture):
        self.req_d.capture_image = capture
        self.future = self.cli_d.call_async(self.req_d)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def send_motion_request(self, destination_pose):
        self.req_moveit = destination_pose
        self.future = self.moveit_cli.call_async(self.req_moveit)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def send_gripper_request(self, arm, action):
        self.req_gripper.arm = arm
        self.req_gripper.action = action
        self.future = self.gripper_cli.call_async(self.req_gripper)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def source_callback(self, msg):
        if msg.name == "red":
            self.source_red = msg
            self.red_s_captured = True
        if msg.name == "green":
            self.source_green = msg
        if msg.name == "yellow":
            self.source_yellow = msg
        if msg.name == "blue":
            self.source_blue = msg

    def destination_callback(self, msg):
        if msg.name == "red_1":
            self.dest_red = msg
            self.red_d_captured = True
        if msg.name == "green_1":
            self.dest_green = msg
        if msg.name == "yellow_1":
            self.dest_yellow = msg
        if msg.name == "blue_1":
            self.dest_blue = msg


def main():
    rclpy.init()
    task = MainTask()

    response = None
    response = task.send_request(True)
    task.get_logger().info(
        f"Source Captured Successfully: {response.capture_sucessfull}"
    )

    response = None
    response = task.send_request_d(True)
    task.get_logger().info(
        f"Destination Captured Successfully: {response.capture_sucessfull}"
    )

    response = None
    response = task.send_gripper_request(arm='left', action='open')

    # if response != None:
    #     response = None
    #     response = task.send_gripper_request(arm='left', action='close')

    while task.red_s_captured == False or task.red_d_captured == False:
        rclpy.spin_once(task)

    pre_pick_offset = 0.05

    moveitreq = TargetPose.Request()
    moveitreq.object = "red"
    moveitreq.action = "pick"
    moveitreq.arm = "left"
    moveitreq.position = Point(x=task.dest_red.center.x, y=task.dest_red.center.y, z=task.dest_red.center.z + pre_pick_offset)
    # pick_rot = pick_angle(task.dest_red.orientation)
    moveitreq.orientation = Quaternion(x=task.dest_red.orientation.x, y=task.dest_red.orientation.y, z=task.dest_red.orientation.z, w=task.dest_red.orientation.w)
    # moveitreq.orientation = Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)
    moveitreq.cartesian = False
    
    response = None
    response = task.send_motion_request(moveitreq)

    # if response !=None:
    #     moveitreq.position = Point(x=task.dest_red.center.x, y=task.dest_red.center.y, z=task.dest_red.center.z + 0.01)
    #     moveitreq.cartesian = True
    #     response = None
    #     response = task.send_motion_request(moveitreq)
    #     task._loop_rate.sleep()

    # if response !=None:
    #     response = None
    #     response = task.send_gripper_request(arm='left', action='close')
    #     task._loop_rate.sleep()

    # if response !=None:
    #     moveitreq.position = Point(x=task.dest_red.center.x, y=task.dest_red.center.y, z=task.dest_red.center.z + pre_pick_offset)
    #     moveitreq.cartesian = True
    #     response = None
    #     response = task.send_motion_request(moveitreq)

    # if response !=None:
    #     moveitreq.position = Point(x=task.source_red.center.x, y=task.source_red.center.y, z=task.source_red.center.z + pre_pick_offset)
    #     moveitreq.cartesian = False
    #     response = None
    #     response = task.send_motion_request(moveitreq)

    # if response !=None:
    #     moveitreq.position = Point(x=task.source_red.center.x, y=task.source_red.center.y, z=task.source_red.center.z + 0.01)
    #     moveitreq.cartesian = True
    #     response = None
    #     response = task.send_motion_request(moveitreq)

    # if response !=None:
    #     response = None
    #     response = task.send_gripper_request(arm='left', action='open')

    # if response !=None:
    #     moveitreq.position = Point(x=task.source_red.center.x, y=task.source_red.center.y, z=task.source_red.center.z + pre_pick_offset)
    #     moveitreq.cartesian = True
    #     response = None
    #     response = task.send_motion_request(moveitreq)


    rclpy.spin(task)
    task.destroy_node()

    rclpy.shutdown()


def moveit():
    pass

def pick_angle(quat):
    q_x = quat.x
    q_y = quat.y
    q_z = quat.z
    q_w = quat.w

    # print([q_x, q_y, q_z, q_w])

    (roll, pitch, yaw) = euler_from_quaternion([q_x, q_y, q_z, q_w])

    if math.degrees(roll) < 0 :
        roll_d = math.degrees(roll)
    else:
        roll_d = math.degrees(roll)


    # print(roll_d, math.degrees(pitch), math.degrees(yaw))
    return 0





if __name__ == "__main__":
    main()
