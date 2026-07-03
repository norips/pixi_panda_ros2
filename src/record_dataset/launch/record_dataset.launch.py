from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

from datetime import datetime
import os


def generate_launch_description():

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    bag_path = f"dataset/ros_bags/episode_{timestamp}"

    dataset = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("record_dataset"),
                "launch",
                "dataset.launch.py",
            )
        )
    )

    rviz_config = os.path.join(
        get_package_share_directory("record_dataset"),
        "rviz",
        "recording.rviz",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
    )

    image_view_gripper = Node(
        package="rqt_image_view",
        executable="rqt_image_view",
        name="gripper_camera",
        arguments=["/camera/gripper/color/image_raw"],
        output="screen",
    )
    image_view_top = Node(
        package="rqt_image_view",
        executable="rqt_image_view",
        name="third_person_camera",
        arguments=["/camera/third_person/color/image_raw"],
        output="screen",
    )
    rosbag = TimerAction(
        period=40.0,
        actions=[
            ExecuteProcess(
                cmd=["echo", "======== STARTING THE DATASET RECORDING ======="],
                output="screen",
            ),
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "bag",
                    "record",
                    "-o",
                    bag_path,
                    "-a",
                ],
                output="screen",
            ),
        ],
    )

    return LaunchDescription([
        dataset,
        #rviz,
        #image_view_gripper,
        #image_view_top,
        rosbag,
    ])