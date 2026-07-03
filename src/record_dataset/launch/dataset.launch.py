from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    #
    # Franka
    #
    franka = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("franka_bringup"),
                "launch",
                "franka.launch.py",
            )
        ),
        launch_arguments={
            "robot_ip": "192.168.8.2",
        }.items(),
    )

    #
    # Third person camera
    #
    third_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("realsense2_camera"),
                "launch",
                "rs_launch.py",
            )
        ),
        launch_arguments={
            "camera_name": "third_person",
            "serial_no": "_260322271701",
        }.items(),
    )

    #
    # Gripper camera
    #
    gripper_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("realsense2_camera"),
                "launch",
                "rs_launch.py",
            )
        ),
        launch_arguments={
            "camera_name": "gripper",
            "serial_no": "_260322278985",
        }.items(),
    )

    #
    # Send SO100 states
    #
    send_to_ros = TimerAction(
        period=15.0,
        actions=[
            ExecuteProcess(
                    cmd=[
                        "python",
                        "src/so100_to_ros/send_to_ros.py",
                    ],
                    output="screen",
                )
        ],
    )

    #
    # Audio capture
    #
    microphone = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "audio_capture_py",
            "audio_capture",
        ],
        output="screen",
    )

    #
    # Teleoperation (lerobot environment)
    #
    teleoperate = TimerAction(
        period=25.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "env",
                    "-i",
                    "HOME=" + os.environ["HOME"],
                    "PATH=" + os.environ["PATH"],
                    "pixi",
                    "run",
                    "-e",
                    "lerobot",
                    "teleoperate",
                    "--port",
                    "/dev/serial/by-id/usb-1a86_USB_Single_Serial_58FD017158-if00",
                ],
            )
        ],
    )

    return LaunchDescription(
        [
            franka,
            third_camera,
            gripper_camera,
            microphone,
            send_to_ros,
            teleoperate,
        ]
    )