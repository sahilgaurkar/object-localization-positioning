# Copyright 1996-2023 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Trajectory follower client for the UR5 robot used for multi-robot demonstration."""

import rclpy
from my_package.follow_joint_trajectory_client import FollowJointTrajectoryClient


GOAL = {
    'joint_names': [
        'right_s0',
        'right_s1',
        'right_e0',
        'right_e1',
        'right_w0',
        'right_w1',
        'right_w2'
    ],
    'points': [
        {
            'positions': [0.0, 0.0, 0.0, 0.0, 0.0495, 0.0495, 0.0495],
            'time_from_start': {'sec': 0, 'nanosec': 0}
        },
        {
            'positions': [0.0, 0.0, 0.0, 0.0, 0.0495, 0.0495, 0.0495],
            'time_from_start': {'sec': 3, 'nanosec': 0}
        },
        {
            'positions': [0.0, 0.0, 0.0, 0.0, 0.85, 0.85, 0.6],
            'time_from_start': {'sec': 4, 'nanosec': 0}
        },
        {
            'positions': [0.63, -2.26, -1.88, -2.14, 0.85, 0.85, 0.6],
            'time_from_start': {'sec': 5, 'nanosec': 0}
        },
        {
            'positions': [0.63, -2.26, -1.88, -2.14, 0.0495, 0.0495, 0.0495],
            'time_from_start': {'sec': 6, 'nanosec': 0}
        },
        {
            'positions': [0.63, -2.0, -1.88, -2.14, 0.0495, 0.0495, 0.0495],
            'time_from_start': {'sec': 7, 'nanosec': 0}
        },
        {
            'positions': [0.0, 0.0, 0.0, 0.0, 0.0495, 0.0495, 0.0495],
            'time_from_start': {'sec': 8, 'nanosec': 0}
        },
        {
            'positions': [0.0, 0.0, 0.0, 0.0, 0.0495, 0.0495, 0.0495],
            'time_from_start': {'sec': 9, 'nanosec': 0}
        }
    ]
}


def main(args=None):
    rclpy.init(args=args)
    controller = FollowJointTrajectoryClient('baxter_controller', '/baxter/rightarm_joint_trajectory_controller')

    controller.send_goal(GOAL, 10)
    rclpy.spin(controller)


if __name__ == '__main__':
    main()