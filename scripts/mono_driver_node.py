#!/usr/bin/env python3
"""
Python node for the MonocularMode cpp node.

Requirements
* Dataset must be configured in EuRoC MAV format
* Paths to dataset must be set before building (or running) this node
* Make sure to set path to your workspace in common.hpp

Command line arguments
-- settings_name: EuRoC, TUM2, KITTI etc; the name of the .yaml file containing camera intrinsics and other configurations
-- image_seq: MH01, V102, etc; the name of the image sequence you want to run
"""

import sys
import os
import glob
import time
import copy
import shutil
from pathlib import Path
import argparse
import natsort
import yaml
import copy
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)

from sensor_msgs.msg import Image
from std_msgs.msg import String, Float64
from cv_bridge import CvBridge, CvBridgeError


class MonoDriver(Node):
    """ROS 2 driver node to stream EuRoC MAV dataset sequences to a C++ SLAM node."""

    def __init__(self, node_name = "mono_py_node"):
        """Initializes parameters, resolves dataset paths, and sets up publishers/subscribers."""
        super().__init__(node_name)

        self.declare_parameter("settings_name","EuRoC")
        self.declare_parameter("image_seq","NULL")

        self.settings_name = str(self.get_parameter('settings_name').value) 
        self.image_seq = str(self.get_parameter('image_seq').value)

        print(f"-------------- Received parameters --------------------------\n")
        print(f"self.settings_name: {self.settings_name}")
        print(f"self.image_seq: {self.image_seq}")
        print()

        self.package_share_dir = self._resolve_package_share_dir()
        self.package_source_dir = self._resolve_package_source_dir(self.package_share_dir)
        default_dataset_parent = self._guess_default_dataset_parent()
        self.declare_parameter("dataset_parent_path", default_dataset_parent)
        dataset_parent = str(self.get_parameter("dataset_parent_path").value)
        self.parent_dir = self._resolve_user_path(dataset_parent)
        self.image_sequence_dir = os.path.join(self.parent_dir, self.image_seq)

        if not os.path.isdir(self.image_sequence_dir):
            raise FileNotFoundError(
                f"Image sequence '{self.image_seq}' nicht gefunden unter {self.image_sequence_dir}. "
                "Setze --ros-args -p dataset_parent_path:=/abs/pfad/zum/Datensatz oder setze die Umgebungsvariable DATASET_PARENT_PATH."
            )

        print(f"self.image_sequence_dir: {self.image_sequence_dir}\n")

        self.node_name = "mono_py_driver"
        self.image_seq_dir = ""
        self.imgz_seqz = []
        self.time_seqz = []

        self.br = CvBridge()

        self.imgz_seqz_dir, self.imgz_seqz, self.time_seqz = self.get_image_dataset_asl(self.image_sequence_dir, "mav0") 

        print(self.image_seq_dir)
        print(len(self.imgz_seqz))

        self.pub_exp_config_name = "/mono_py_driver/experiment_settings" 
        self.sub_exp_ack_name = "/mono_py_driver/exp_settings_ack"
        self.pub_img_to_agent_name = "/mono_py_driver/img_msg"
        self.pub_timestep_to_agent_name = "/mono_py_driver/timestep_msg"
        self.send_config = True
        
        self.publish_exp_config_ = self.create_publisher(String, self.pub_exp_config_name, 1)

        #self.exp_config_msg = self.settings_name + "/" + self.image_seq
        self.exp_config_msg = self.settings_name
        print(f"Configuration to be sent: {self.exp_config_msg}")

        self.subscribe_exp_ack_ = self.create_subscription(String, 
                                                           self.sub_exp_ack_name, 
                                                           self.ack_callback ,10)
        self.subscribe_exp_ack_

        self.publish_img_msg_ = self.create_publisher(Image, self.pub_img_to_agent_name, 1)
        self.publish_timestep_msg_ = self.create_publisher(Float64, self.pub_timestep_to_agent_name, 1)

        self.start_frame = 0
        self.end_frame = -1
        self.frame_stop = -1
        self.show_imgz = False
        self.frame_id = 0
        self.frame_count = 0
        self.inference_time = []

        print()
        print(f"MonoDriver initialized, attempting handshake with CPP node")

    def _resolve_package_share_dir(self) -> str:
        """Resolves the share directory for the ros2_orb_slam3 package."""
        try:
            return str(Path(get_package_share_directory("ros2_orb_slam3")))
        except (PackageNotFoundError, ValueError) as exc:
            fallback = Path(__file__).resolve().parent.parent
            print(f"[WARN] Using fallback package path {fallback}: {exc}")
            return str(fallback)

    def _resolve_package_source_dir(self, share_dir: str):
        """Finds the source directory path of the package based on the share path."""
        share_path = Path(share_dir)
        for parent in share_path.parents:
            candidate = parent / "src" / "ros2_orb_slam3"
            if candidate.exists():
                return str(candidate)
        return None

    def _resolve_relative_path(self, relative_path: str) -> str:
        """Resolves a given relative path against package share or source directories."""
        candidate = Path(relative_path)
        if candidate.is_absolute():
            return str(candidate)

        share_candidate = Path(self.package_share_dir) / candidate
        if share_candidate.exists():
            return str(share_candidate)

        if self.package_source_dir is not None:
            source_candidate = Path(self.package_source_dir) / candidate
            if source_candidate.exists():
                return str(source_candidate)

        return str(share_candidate)

    def _resolve_user_path(self, configured_path: str) -> str:
        """Expands user directories and resolves relative paths to absolute paths."""
        expanded = Path(configured_path).expanduser()
        if expanded.is_absolute():
            return str(expanded)

        return self._resolve_relative_path(str(expanded))

    def _guess_default_dataset_parent(self) -> str:
        """Guess a sensible default dataset parent path.

        Order:
        1. Environment variables `DATASET_PARENT_PATH` or `ORB_SLAM_DATASET_PATH`
        2. `TEST_DATASET` relative to package share or source
        3. Search upward from this file for `TEST_DATASETS` or `TEST_DATASET`
        4. Fallback to `~/TEST_DATASETS`
        """
        env_path = os.environ.get("DATASET_PARENT_PATH") or os.environ.get("ORB_SLAM_DATASET_PATH")
        if env_path:
            return str(Path(env_path).expanduser())

        pkg_candidate = Path(self.package_share_dir) / "TEST_DATASET"
        if pkg_candidate.exists():
            return str(pkg_candidate)

        if self.package_source_dir is not None:
            src_candidate = Path(self.package_source_dir) / "TEST_DATASET"
            if src_candidate.exists():
                return str(src_candidate)

        current = Path(__file__).resolve().parent
        for _ in range(6):
            cand1 = current / "TEST_DATASETS"
            cand2 = current / "TEST_DATASET"
            if cand1.exists():
                return str(cand1)
            if cand2.exists():
                return str(cand2)
            current = current.parent

        return str(Path("~/TEST_DATASETS").expanduser())

    def get_image_dataset_asl(self, exp_dir, agent_name = "mav0"):
        """Returns images and list of timesteps in ascending order from an ASL formatted dataset."""
        imgz_file_list = []
        time_list = []

        agent_cam0_fld = os.path.join(exp_dir, agent_name, "cam0")
        imgz_file_dir = os.path.join(agent_cam0_fld, "data") + "/"
        imgz_file_list = natsort.natsorted(os.listdir(imgz_file_dir),reverse=False)

        for iox in imgz_file_list:
            time_step = iox.split(".")[0]
            time_list.append(time_step)

        return imgz_file_dir, imgz_file_list, time_list

    def ack_callback(self, msg):
        """Handles acknowledgement messages from the C++ node to confirm configuration receipt."""
        print(f"Got ack: {msg.data}")
        
        if(msg.data == "ACK"):
            self.send_config = False
            # self.subscribe_exp_ack_.destory() 

    def handshake_with_cpp_node(self):
        """Publishes the configuration settings repeatedly until an acknowledgment is received."""
        if (self.send_config == True):
            msg = String()
            msg.data = self.exp_config_msg
            self.publish_exp_config_.publish(msg)
            time.sleep(0.01)

    def run_py_node(self, idx, imgz_name):
        """Reads a specific image frame, synchronizes its timestamp, and publishes it."""
        img_msg = None

        img_look_up_path = self.imgz_seqz_dir  + imgz_name
        timestamp_ns = int(imgz_name.split(".")[0])
        timestep = timestamp_ns / 1e9
        self.frame_id = self.frame_id + 1  

        img_msg = self.br.cv2_to_imgmsg(cv2.imread(img_look_up_path), encoding="passthrough")
        img_msg.header.stamp.sec = int(timestamp_ns // 1_000_000_000)
        img_msg.header.stamp.nanosec = int(timestamp_ns % 1_000_000_000)
        img_msg.header.frame_id = "cam0"
        timestep_msg = Float64()
        timestep_msg.data = timestep

        try:
            self.publish_timestep_msg_.publish(timestep_msg) 
            self.publish_img_msg_.publish(img_msg)
        except CvBridgeError as e:
            print(e)


def main(args = None):
    """Main entry point to initialize the driver node and handle the image streaming loop."""
    rclpy.init(args=args)
    n = MonoDriver("mono_py_node")
    rate = n.create_rate(20)
    
    while(n.send_config == True):
        n.handshake_with_cpp_node()
        rclpy.spin_once(n)
        #self.rate.sleep(10)

        if(n.send_config == False):
            break
        
    print(f"Handshake complete. C++ node is ready.")
    print(f"Waiting 1 second as safety delay...")
    time.sleep(1.0)
    print(f"Starting image sequence playback...")

    for idx, imgz_name in enumerate(n.imgz_seqz[n.start_frame:n.end_frame]):
        try:
            rclpy.spin_once(n)
            n.run_py_node(idx, imgz_name)
            rate.sleep()

            if (n.frame_id>n.frame_stop and n.frame_stop != -1):
                print(f"BREAK!")
                break
        
        except KeyboardInterrupt:
            break

    cv2.destroyAllWindows()
    n.destroy_node()
    rclpy.shutdown()


if __name__=="__main__":
    main()