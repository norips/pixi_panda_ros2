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
import argparse

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
    parser = argparse.ArgumentParser(description="SO100 teleoperation publisher")
    parser.add_argument(
        "--port",
        default="/dev/ttyACM0",
        help="Serial port for the SO100 leader arm (default: /dev/ttyACM0)",
    )
    args = parser.parse_args()

    # Initialize the robot and teleoperator config
    leader_config = SO100LeaderConfig(port=args.port, id="my_awesome_leader_arm", calibration_dir=Path(__file__).parent / "calibration_so100_leader")

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
    socket = context.socket(zmq.PUB)
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

        rot = Rotation.from_rotvec([leader_ee_act["ee.wx"], leader_ee_act["ee.wy"], leader_ee_act["ee.wz"]])
        quat = rot.as_quat()
        leader_ee_act["ee.qx"] = quat[0]
        leader_ee_act["ee.qy"] = quat[1]
        leader_ee_act["ee.qz"] = quat[2]
        leader_ee_act["ee.qw"] = quat[3]

        socket.send_string(json.dumps(leader_ee_act))
        print("Sending data", leader_ee_act)

        precise_sleep(max(1.0 / FPS - (time.perf_counter() - t0), 0.0))


if __name__ == "__main__":
    main()