#!/usr/bin/env python3
import math

from geometry_msgs.msg import Point, Quaternion

import rclpy
from rclpy.node import Node

from opencv_interfaces.msg import ObjectList

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


from turtlesim.srv import Spawn


class FrameListener(Node):

    def __init__(self):
        super().__init__('turtle_tf2_frame_listener')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Call on_timer function every second
        self.timer = self.create_timer(1.0, self.on_timer)

        self.publisher = self.create_publisher(ObjectList, 'source/objects', 10)
        self.publisher1 = self.create_publisher(ObjectList, 'destination/objects', 10)

    def on_timer(self):
        # Store frame names in variables that will be used to
        # compute transformations
        for idx, object in enumerate(['red', 'green', 'blue', 'yellow', 'red_1', 'green_1', 'blue_1', 'yellow_1']):
            from_frame_rel = object

            if idx < 4:
                to_frame_rel = 'world_1'
            else:
                to_frame_rel = 'world'
            


            # Look up for the transformation between target_frame and turtle2 frames
            # and send velocity commands for turtle2 to reach target_frame
            try:
                t = self.tf_buffer.lookup_transform(
                    to_frame_rel,
                    from_frame_rel,
                    rclpy.time.Time())
            except TransformException as ex:
                self.get_logger().info(
                    f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
                continue


            msg = ObjectList()

            msg.name = from_frame_rel
            msg.center = Point(x=t.transform.translation.x, y=t.transform.translation.y, z=t.transform.translation.z)
            msg.orientation = Quaternion(x=t.transform.rotation.x, y=t.transform.rotation.y, z=t.transform.rotation.z, w=t.transform.rotation.w)

            if idx < 4:
                self.publisher.publish(msg)
            else:
                self.publisher1.publish(msg)


def main():
    rclpy.init()
    node = FrameListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()