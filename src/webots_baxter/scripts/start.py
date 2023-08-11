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

        self.source_captured = [0, 0, 0, 0]
        self.dest_captured = [0, 0, 0, 0]
        print('init')

        #Transfer Position
        self.transfer_point = ObjectList()
        self.transfer_point.center = Point(x= 0.5, y= 0.0, z=0.71)
        self.transfer_point.orientation = Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)

        #Transfer Position
        self.safe_point = ObjectList()
        self.safe_point.center = Point(x= 0.5, y= 0.0, z=0.9)
        self.safe_point.orientation = Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)

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
            self.source_captured[0] = 1
        if msg.name == "green":
            self.source_green = msg
            self.source_captured[1] = 1
        if msg.name == "yellow":
            self.source_yellow = msg
            self.source_captured[2] = 1
        if msg.name == "blue":
            self.source_blue = msg
            self.source_captured[3] = 1

    def destination_callback(self, msg):
        if msg.name == "red_1":
            self.dest_red = msg
            self.dest_captured[0] = 1
        if msg.name == "green_1":
            self.dest_green = msg
            self.dest_captured[1] = 1
        if msg.name == "yellow_1":
            self.dest_yellow = msg
            self.dest_captured[2] = 1
        if msg.name == "blue_1":
            self.dest_blue = msg
            self.dest_captured[3] = 1

    def goto_home(self, arm):
        #Goto Home
        req = TargetPose.Request()

        req.arm = arm
        req.orientation = Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)
        req.cartesian = False

        if arm == 'left':
            req.position = Point(x=0.6, y=0.6, z=1.1)
        elif arm == 'right':
            req.position = Point(x=0.6, y=-0.6, z=1.1)

        self.send_motion_request(req)

    def pick_object(self, arm, object ):
        req = TargetPose.Request()
        req.arm = arm

        pre_pick_offset = 0.2
        pick_offset = 0.008

        # #Move to Safe Point
        # req.cartesian = False
        # req.position, req.orientation = self.get_pose(obj_msg=self.safe_point, z_offset=0)
        # self.send_motion_request(req)

        # Move to Prepick
        req.cartesian = False
        if object == 'red':
            req.position, req.orientation = self.get_pose(obj_msg=self.dest_red, z_offset=pre_pick_offset)
        if object == 'green':
            req.position, req.orientation = self.get_pose(obj_msg=self.dest_green, z_offset=pre_pick_offset)
        if object == 'blue':
            req.position, req.orientation = self.get_pose(obj_msg=self.dest_blue, z_offset=pre_pick_offset)
        if object == 'yellow':
            req.position, req.orientation = self.get_pose(obj_msg=self.dest_yellow, z_offset=pre_pick_offset)
        if object == 'transfer':
            req.position, req.orientation = self.get_pose(obj_msg=self.transfer_point, z_offset=pre_pick_offset)

        prepick_req = req
        self.send_motion_request(prepick_req)

        #Open Gripper
        self.send_gripper_request(arm=arm, action='open')

        # Move to Pick
        req.cartesian = True
        if object == 'red':
            req.position, req.orientation = self.get_pose(obj_msg=self.dest_red, z_offset=pick_offset)
        if object == 'green':
            req.position, req.orientation = self.get_pose(obj_msg=self.dest_green, z_offset=pick_offset)
        if object == 'blue':
            req.position, req.orientation = self.get_pose(obj_msg=self.dest_blue, z_offset=pick_offset)
        if object == 'yellow':
            req.position, req.orientation = self.get_pose(obj_msg=self.dest_yellow, z_offset=pick_offset)
        if object == 'transfer':
            req.position, req.orientation = self.get_pose(obj_msg=self.transfer_point, z_offset=pick_offset)

        self.send_motion_request(req)

        #Close Gripper
        self.send_gripper_request(arm=arm, action='close')

        #Move to Prepick
        if object == 'red':
            req.position, req.orientation = self.get_pose(obj_msg=self.dest_red, z_offset=pre_pick_offset)
        if object == 'green':
            req.position, req.orientation = self.get_pose(obj_msg=self.dest_green, z_offset=pre_pick_offset)
        if object == 'blue':
            req.position, req.orientation = self.get_pose(obj_msg=self.dest_blue, z_offset=pre_pick_offset)
        if object == 'yellow':
            req.position, req.orientation = self.get_pose(obj_msg=self.dest_yellow, z_offset=pre_pick_offset)
        if object == 'transfer':
            req.position, req.orientation = self.get_pose(obj_msg=self.transfer_point, z_offset=pre_pick_offset)

        prepick_req.cartesian = True
        self.send_motion_request(prepick_req)

        # #Move to Safe Point
        # req.cartesian = False
        # req.position, req.orientation = self.get_pose(obj_msg=self.safe_point, z_offset=0)
        # self.send_motion_request(req)

        # #Goto Home
        # self.goto_home(arm=arm)

    def place_object(self, arm, object ):
        req = TargetPose.Request()
        req.arm = arm

        pre_place_offset = 0.2
        place_offset = 0.008

        # Move to Preplace
        req.cartesian = False
        if object == 'red':
            req.position, req.orientation = self.get_pose(obj_msg=self.source_red, z_offset=pre_place_offset)
        if object == 'green':
            req.position, req.orientation = self.get_pose(obj_msg=self.source_green, z_offset=pre_place_offset)
        if object == 'blue':
            req.position, req.orientation = self.get_pose(obj_msg=self.source_blue, z_offset=pre_place_offset)
        if object == 'yellow':
            req.position, req.orientation = self.get_pose(obj_msg=self.source_yellow, z_offset=pre_place_offset)
        if object == 'transfer':
            req.position, req.orientation = self.get_pose(obj_msg=self.transfer_point, z_offset=pre_place_offset)


        preplace_req = req
        self.send_motion_request(preplace_req)

        # Move to Place
        req.cartesian = True
        if object == 'red':
            req.position, req.orientation = self.get_pose(obj_msg=self.source_red, z_offset=place_offset)
        if object == 'green':
            req.position, req.orientation = self.get_pose(obj_msg=self.source_green, z_offset=place_offset)
        if object == 'blue':
            req.position, req.orientation = self.get_pose(obj_msg=self.source_blue, z_offset=place_offset)
        if object == 'yellow':
            req.position, req.orientation = self.get_pose(obj_msg=self.source_yellow, z_offset=place_offset)
        if object == 'transfer':
            req.position, req.orientation = self.get_pose(obj_msg=self.transfer_point, z_offset=place_offset)

        self.send_motion_request(req)

        #Close Gripper
        self.send_gripper_request(arm=arm, action='open')

        #Move to Preplace
        if object == 'red':
            req.position, req.orientation = self.get_pose(obj_msg=self.source_red, z_offset=pre_place_offset)
        if object == 'green':
            req.position, req.orientation = self.get_pose(obj_msg=self.source_green, z_offset=pre_place_offset)
        if object == 'blue':
            req.position, req.orientation = self.get_pose(obj_msg=self.source_blue, z_offset=pre_place_offset)
        if object == 'yellow':
            req.position, req.orientation = self.get_pose(obj_msg=self.source_yellow, z_offset=pre_place_offset)
        if object == 'transfer':
            req.position, req.orientation = self.get_pose(obj_msg=self.transfer_point, z_offset=pre_place_offset)

        preplace_req.cartesian = True
        self.send_motion_request(preplace_req)

        #Close Gripper
        self.send_gripper_request(arm=arm, action='close')

        # #Move to Safe Point
        # req.cartesian = False
        # req.position, req.orientation = self.get_pose(obj_msg=self.safe_point, z_offset=0)
        # self.send_motion_request(req)

        #Goto Home
        self.goto_home(arm=arm)

    def get_pose(self, obj_msg, z_offset):
        position = Point(x=obj_msg.center.x, y=obj_msg.center.y, z=obj_msg.center.z + z_offset)
        orientation = Quaternion(x=obj_msg.orientation.x, y=obj_msg.orientation.y, z=obj_msg.orientation.z, w=obj_msg.orientation.w)
        return position, orientation





def main():
    rclpy.init()
    task = MainTask()

    # Camera Trigger
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

    #Goto Home
    task.goto_home(arm='left')
    task.goto_home(arm='right')


    while 1:
        rclpy.spin_once(task)
        if (task.source_captured[0] and task.source_captured[1] and task.source_captured[2] and task.source_captured[3]):
            if (task.dest_captured[0] and task.dest_captured[1] and task.dest_captured[2] and task.dest_captured[3]):
                print('Received all Objects')
                break


    # Move Red
    if task.dest_red.center.y >= 0:
        pick_arm = 'left'
    else:
        pick_arm = 'right'

    if task.source_red.center.y >= 0:
        place_arm = 'left'
    else:
        place_arm = 'right'
    
    task.pick_object(arm=pick_arm, object='red')

    if pick_arm != place_arm:
        task.place_object(arm=pick_arm, object='transfer')
        task.pick_object(arm=place_arm, object='transfer')

    task.place_object(arm=place_arm, object='red')

    task.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()
