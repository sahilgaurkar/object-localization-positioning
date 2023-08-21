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
import pandas as pd
import openpyxl
from controller import Supervisor, Robot

from opencv_interfaces.srv import BlockPose
from opencv_interfaces.msg import ObjectList
from opencv_interfaces.srv import GripperCmd
from opencv_interfaces.srv import TargetPose
from opencv_interfaces.srv import CaptureSource


class MainTask(Node):
    def __init__(self):
        super().__init__("main_task_node")

        self.pose_dict = {}

        self._loop_rate = self.create_rate(10)

        self.source_captured = [0, 0, 0, 0]
        self.dest_captured = [0, 0, 0, 0]

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

        #Evaluation
        self.pose_dict[f'{msg.name}_source'] = self.get_position(msg) + self.get_euler(msg)

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
        # self.send_gripper_request(arm=arm, action='close')

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

    def perform_task(self, object):
        # Move object

        if object == 'red':
            dest_object = self.dest_red
            source_object = self.source_red
        if object == 'blue':
            dest_object = self.dest_blue
            source_object = self.source_blue
        if object == 'green':
            dest_object = self.dest_green
            source_object = self.source_green
        if object == 'yellow':
            dest_object = self.dest_yellow
            source_object = self.source_yellow

        if dest_object.center.y >= 0:
            pick_arm = 'left'
        else:
            pick_arm = 'right'

        if source_object.center.y >= 0:
            place_arm = 'left'
        else:
            place_arm = 'right'

        [x, y, z, roll, pitch, yaw] = self.get_difference(source_msg=source_object, dest_msg=dest_object)
        
        #Tolerance for moving pieces
        if (x < 0.001 and y < 0.001 and yaw < 1):
            self.get_logger().info(f"{object} object already in desired Position")
            return
        else:
            self.get_logger().info(f"Moving {object} object already to desired Position")
            self.pick_object(arm=pick_arm, object=object)

            if pick_arm != place_arm:
                self.place_object(arm=pick_arm, object='transfer')
                self.pick_object(arm=place_arm, object='transfer')

            self.place_object(arm=place_arm, object=object)

    def get_euler(self, obj_msg):
        orientation = [obj_msg.orientation.x, obj_msg.orientation.y, obj_msg.orientation.z, obj_msg.orientation.w]
        orientation = euler_from_quaternion(orientation, 'sxyz')
        r = orientation[0] * (180/math.pi)
        p = orientation[1] * (180/math.pi)
        y = orientation[2] * (180/math.pi)
        return [r, p, y]

    def get_position(self, obj_msg):
        return [obj_msg.center.x, obj_msg.center.y, obj_msg.center.z]
        
    def get_difference(self, source_msg, dest_msg):
        object = source_msg.name
        x = math.dist([source_msg.center.x], [dest_msg.center.x])
        y = math.dist([source_msg.center.y], [dest_msg.center.y])
        z = math.dist([source_msg.center.z], [dest_msg.center.z])

        s_r, s_p, s_y = self.get_euler(source_msg)
        d_r, d_p, d_y = self.get_euler(dest_msg)

        roll = abs(abs(s_r) - abs(d_r))
        pitch = abs(abs(s_p) - abs(d_p))
        yaw = abs(abs(s_y) - abs(d_y))

        self.get_logger().info(
            f'\nObject: {object}\nX_error = {x}\nY_error = {y}\nZ_error = {z}\nAngle_error = {yaw}'
        )

        return [x, y, z, roll, pitch, yaw]

    def clear_msg(self):
        self.dest_red = None
        self.dest_blue = None
        self.dest_green = None
        self.dest_yellow = None



def main():
    rclpy.init()
    task = MainTask()

    # desk_node = task.webots_supervisor.getFromDef('desk') 
    # translation_field = desk_node.getField('translation')
    # task.get_logger().info(
    #     f"Desk Translation field: {translation_field}"
    # )

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

    # For Evaluation
    task.pose_dict[f'red_initial'] = task.get_position(task.dest_red) + task.get_euler(task.dest_red)
    task.pose_dict[f'blue_initial'] = task.get_position(task.dest_blue) + task.get_euler(task.dest_blue)
    task.pose_dict[f'green_initial'] = task.get_position(task.dest_green) + task.get_euler(task.dest_green)
    task.pose_dict[f'yellow_initial'] = task.get_position(task.dest_yellow) + task.get_euler(task.dest_yellow)


    for object in ['red', 'blue', 'green', 'yellow']:
        task.perform_task(object)
        task.dest_captured = [0, 0, 0, 0]
        response = None
        task.clear_msg()
        response = task.send_request_d(True)
        task.get_logger().info(
            f"Destination Captured Successfully: {response.capture_sucessfull}"
        )
        while 1:
            rclpy.spin_once(task)
            if (task.source_captured[0] and task.source_captured[1] and task.source_captured[2] and task.source_captured[3]):
                if (task.dest_captured[0] and task.dest_captured[1] and task.dest_captured[2] and task.dest_captured[3]):
                    print('Received all Objects')
                    break

    #Goto Home
    task.goto_home(arm='left')
    task.goto_home(arm='right')

    # For Evaluation
    task.pose_dict[f'red_final'] = task.get_position(task.dest_red) + task.get_euler(task.dest_red)
    task.pose_dict[f'blue_final'] = task.get_position(task.dest_blue) + task.get_euler(task.dest_blue)
    task.pose_dict[f'green_final'] = task.get_position(task.dest_green) + task.get_euler(task.dest_green)
    task.pose_dict[f'yellow_final'] = task.get_position(task.dest_yellow) + task.get_euler(task.dest_yellow)

    task.pose_dict[f'red_error'] = task.get_difference(source_msg=task.source_red, dest_msg=task.dest_red)
    task.pose_dict[f'blue_error'] = task.get_difference(source_msg=task.source_blue, dest_msg=task.dest_blue)
    task.pose_dict[f'green_error'] = task.get_difference(source_msg=task.source_green, dest_msg=task.dest_green)
    task.pose_dict[f'yellow_error'] = task.get_difference(source_msg=task.source_yellow, dest_msg=task.dest_yellow)

    
    # Save Results
    df = get_dataframe(task.pose_dict)
    home_dir = 'Evaluation'
    test_number = '100'
    file = f'Report_{test_number}.xlsx'
    path = f'{home_dir}/{file}'

    df.to_excel(path)


    task.destroy_node()

    rclpy.shutdown()


def get_dataframe(dict):
    result = pd.DataFrame()
    for object in ['red', 'blue', 'green', 'yellow']:

        s_pose = dict.get(f'{object}_source')
        i_pose = dict.get(f'{object}_initial')
        f_pose = dict.get(f'{object}_final')
        e_pose = dict.get(f'{object}_error')

        df = pd.DataFrame(
            [s_pose, i_pose, f_pose, e_pose],
            index=[f"Source {object}", f"Initial {object}", f"Final {object}", f"Error {object}"],
            columns=["x (m)", "y (m)", "z (m)", "r (deg)", "p (deg)", "y (deg)"],
        )
        
        result = pd.concat([result, df], ignore_index=False)
            
    return result


if __name__ == "__main__":
    main()
