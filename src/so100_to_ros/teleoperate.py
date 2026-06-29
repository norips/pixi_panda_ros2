# !/usr/bin/env python

# Copyright 2025 The HuggingFace Inc. team. All rights reserved.
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

import time

from pathlib import Path

from lerobot.model.kinematics import RobotKinematics
from lerobot.processor import RobotProcessorPipeline
from lerobot.processor.converters import (
    robot_action_to_transition,
    transition_to_robot_action,
)
from lerobot.robots.so_follower.robot_kinematic_processor import (
    ForwardKinematicsJointsToEE,
)
from lerobot.teleoperators.so_leader import SO100Leader, SO100LeaderConfig
from lerobot.types import RobotAction
from lerobot.utils.robot_utils import precise_sleep

import zmq
import json
from scipy.spatial.transform import Rotation
import math
import numpy as np

FPS = 60

# Retrieve SO-ARM100 EE position & orientation through LeRobot library and send them over ZMQ as LeRobot is not compatible with python version shipped with ROS1 noetic.
# See "send_to_ros.py" that receive this information and send it back as ros topic
def main():
    # Initialize the robot and teleoperator config
    leader_config = SO100LeaderConfig(port="/dev/ttyACM0", id="my_awesome_leader_arm", calibration_dir=Path(__file__).parent / "calibration_so100_leader")

    # Initialize the robot and teleoperator
    leader = SO100Leader(leader_config)

    # NOTE: It is highly recommended to use the urdf in the SO-ARM100 repo: https://github.com/TheRobotStudio/SO-ARM100/blob/main/Simulation/SO101/so101_new_calib.urdf
    leader_kinematics_solver = RobotKinematics(
        urdf_path=str(Path(__file__).parent / "so101"),
        target_frame_name="gripper_frame_joint",
        joint_names=list(leader.bus.motors.keys()),
    )

    # Build pipeline to convert teleop joints to EE action
    leader_to_ee = RobotProcessorPipeline[RobotAction, RobotAction](
        steps=[
            ForwardKinematicsJointsToEE(
                kinematics=leader_kinematics_solver, motor_names=list(leader.bus.motors.keys())
            ),
        ],
        to_transition=robot_action_to_transition,
        to_output=transition_to_robot_action,
    )

    leader_wrist_kinematics_solver = RobotKinematics(
        urdf_path=str(Path(__file__).parent / "so101"),
        target_frame_name="wrist_link",
        joint_names=list(leader.bus.motors.keys()),
    )

    # Build pipeline to convert teleop joints to EE action
    leader_to_wrist = RobotProcessorPipeline[RobotAction, RobotAction](
        steps=[
            ForwardKinematicsJointsToEE(
                kinematics=leader_wrist_kinematics_solver, motor_names=list(leader.bus.motors.keys())
            ),
        ],
        to_transition=robot_action_to_transition,
        to_output=transition_to_robot_action,
    )

    # Connect to the robot and teleoperator
    leader.connect()

    # Init rerun viewer
    # init_rerun(session_name="so100_EE_to_ros")


    # Used to send data to python 3.8 ROS process
    context = zmq.Context()
    socket = context.socket(zmq.PAIR)
    socket.bind("ipc:///tmp/test.sock")

    # create 90° rotation about Z
    rot_z90 = Rotation.from_euler('z', 90, degrees=True)   # rotation object


    print("Starting teleop loop...")
    while True:
        t0 = time.perf_counter()

        # Get teleop observation
        leader_joints_obs = leader.get_action()
        # teleop joints -> teleop EE action
        leader_ee_act = leader_to_ee(leader_joints_obs)

        leader_wrist_act = leader_to_wrist(leader_joints_obs)


        # Convert wrist roll to angle (-100, 100)
        angle_wrist = (leader_joints_obs["wrist_roll.pos"] + 100) / 200 * 360 - 180
        # x_angle = 0 + math.radians(180)
        # y_angle = 0 
        # z_angle = 0 + math.radians(angle_wrist)
        pos = np.array([leader_ee_act["ee.x"], leader_ee_act["ee.y"], leader_ee_act["ee.z"]])          # example position
        pos_rotated = rot_z90.apply(pos)
        leader_ee_act["ee.x"] = pos_rotated[0]
        leader_ee_act["ee.y"] = pos_rotated[1]
        leader_ee_act["ee.z"] = pos_rotated[2]

        x_angle = leader_ee_act["ee.wx"]
        y_angle = leader_ee_act["ee.wy"] - math.radians(10)
        z_angle = leader_ee_act["ee.wz"]
        # x_angle = leader_wrist_act["ee.wx"]
        # y_angle = leader_wrist_act["ee.wy"]
        # z_angle = leader_wrist_act["ee.wz"]


        # Create a Rotation object from Euler angles in 'xyz' sequence
        # Use degrees=True if your angles are in degrees, False if in radians
        rot = Rotation.from_euler('zyx', [x_angle, y_angle, z_angle], degrees=False)

        # Convert the Rotation object to a quaternion
        # The result is a numpy array [x, y, z, w]
        quat = rot.as_quat()

        
        r_old = Rotation.from_quat(quat)
        r_new_global = rot_z90 * r_old
        quat = r_new_global.as_quat()  # [x,y,z,w]

        leader_ee_act["ee.qx"] = quat[0]
        leader_ee_act["ee.qy"] = quat[1]
        leader_ee_act["ee.qz"] = quat[2]
        leader_ee_act["ee.qw"] = quat[3]


        # quat = rot.as_quat()

        # leader_wrist_act["ee.qx"] = quat[0]
        # leader_wrist_act["ee.qy"] = quat[1]
        # leader_wrist_act["ee.qz"] = quat[2]
        # leader_wrist_act["ee.qw"] = quat[3]

        # leader_ee_act["ee.qx"] = 1
        # leader_ee_act["ee.qy"] = -3.342687705298886e-05
        # leader_ee_act["ee.qz"] = 0.0002046426379820332
        # leader_ee_act["ee.qw"] = 7.346553502429742e-06

        socket.send_string(json.dumps(leader_ee_act))

        precise_sleep(max(1.0 / FPS - (time.perf_counter() - t0), 0.0))

if __name__ == "__main__":
    main()