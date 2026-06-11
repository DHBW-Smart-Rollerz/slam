"""ROS 2 launch file for playing a rosbag through camera preprocessing and ORB-SLAM3."""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generates the launch description for the rosbag-driven SLAM pipeline."""

    preprocessing_params_default = os.path.join(
        FindPackageShare("camera_preprocessing").find("camera_preprocessing"),
        "config",
        "ros_params.yaml",
    )

    args = [
        DeclareLaunchArgument("node_name_arg", default_value="mono_slam_cpp"),
        DeclareLaunchArgument("settings_name", default_value="smartrollerz"),
        DeclareLaunchArgument(
            "bag_path",
            default_value="TEST_ROSBAGS/rosbag2_2026_01_24-13_56_00",
        ),
        DeclareLaunchArgument(
            "input_topic", default_value="/camera/image/raw"
        ),
        DeclareLaunchArgument("playback_rate", default_value="1.0"),
        DeclareLaunchArgument("use_roi_mask", default_value="true"),
        DeclareLaunchArgument("roi_mask_top_percent", default_value="0.45"),
        DeclareLaunchArgument("enable_imu", default_value="true"),
        DeclareLaunchArgument("auto_detect_imu_topic", default_value="true"),
        DeclareLaunchArgument("imu_topic", default_value="/imu/data"),
        DeclareLaunchArgument("num_skip_frames", default_value="0"),
        DeclareLaunchArgument("debug", default_value="false"),
        DeclareLaunchArgument(
            "preprocessing_params_file",
            default_value=preprocessing_params_default,
        ),
        DeclareLaunchArgument(
            "preprocessed_topic", default_value="/camera/image/undistorted"
        ),
        DeclareLaunchArgument(
            "roi_output_topic", default_value="/camera/image/roi_masked"
        ),
        DeclareLaunchArgument("deduplicate_by_stamp", default_value="true"),
    ]

    mono_node = Node(
        package="ros2_orb_slam3",
        executable="mono_node_cpp",
        output="screen",
        parameters=[
            {"node_name_arg": LaunchConfiguration("node_name_arg")},
        ],
        # Clear specific variables to prevent GTK/UI conflicts with OpenCV/ORB-SLAM3
        additional_env={
            "GTK_PATH": "",
            "GTK_EXE_PREFIX": "",
            "GIO_MODULE_DIR": "",
            "LOCPATH": "",
        },
        remappings=[
            (
                "/mono_py_driver/img_msg",
                LaunchConfiguration("roi_output_topic"),
            ),
        ],
    )

    preprocessing_node = Node(
        package="camera_preprocessing",
        executable="camera_preprocessing_node",
        name="camera_preprocessing_node",
        output="screen",
        parameters=[
            LaunchConfiguration("preprocessing_params_file"),
            {"debug": LaunchConfiguration("debug")},
            {"image_topic": LaunchConfiguration("input_topic")},
            {"num_skip_frames": LaunchConfiguration("num_skip_frames")},
        ],
    )

    rosbag_driver_node = Node(
        package="ros2_orb_slam3",
        executable="rosbag_driver_node.py",
        name="rosbag_driver_node",
        output="screen",
        parameters=[
            {"settings_name": LaunchConfiguration("settings_name")},
            {"bag_path": LaunchConfiguration("bag_path")},
            {"input_topic": LaunchConfiguration("input_topic")},
            {"playback_rate": LaunchConfiguration("playback_rate")},
            {"use_roi_mask": LaunchConfiguration("use_roi_mask")},
            {
                "roi_mask_top_percent": LaunchConfiguration(
                    "roi_mask_top_percent"
                )
            },
            {"enable_imu": LaunchConfiguration("enable_imu")},
            {
                "auto_detect_imu_topic": LaunchConfiguration(
                    "auto_detect_imu_topic"
                )
            },
            {"imu_topic": LaunchConfiguration("imu_topic")},
        ],
    )

    roi_mask_node = Node(
        package="ros2_orb_slam3",
        executable="roi_mask_node.py",
        name="roi_mask_node",
        output="screen",
        parameters=[
            {"input_topic": LaunchConfiguration("preprocessed_topic")},
            {"output_topic": LaunchConfiguration("roi_output_topic")},
            {"enabled": False},
            {
                "deduplicate_by_stamp": LaunchConfiguration(
                    "deduplicate_by_stamp"
                )
            },
            {
                "roi_mask_top_percent": LaunchConfiguration(
                    "roi_mask_top_percent"
                )
            },
        ],
    )

    return LaunchDescription(
        args
        + [
            mono_node,
            preprocessing_node,
            rosbag_driver_node,
            roi_mask_node,
        ]
    )