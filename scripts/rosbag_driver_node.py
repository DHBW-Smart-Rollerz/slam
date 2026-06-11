#!/usr/bin/env python3
"""
ROS2 Bag Playback Node für ORB-SLAM3

Dieser Node spielt einen ROS2 Bag ab und republisht die Kamerabilder 
auf den Topics, die der ORB-SLAM3 C++ Node erwartet.

Unterstützt ROI-Masking für Straßenszenarien, bei denen das Fahrzeug
im unteren Bildbereich maskiert werden soll.
"""

import sys
import os
import time
from pathlib import Path
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)

from sensor_msgs.msg import Image, Imu
from std_msgs.msg import String, Float64

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rosidl_runtime_py.utilities import get_message


ENCODING_SPECS = {
    "mono8": (np.uint8, 1),
    "bgr8": (np.uint8, 3),
    "rgb8": (np.uint8, 3),
}


class RosbagDriverNode(Node):
    """ROS 2 Node to read camera images and IMU data from an MCAP/db3 rosbag and republish them."""
    
    def __init__(self, node_name="rosbag_driver_node"):
        """Initializes parameters, sets up publishers/subscribers, and verifies bag path accessibility."""
        super().__init__(node_name)
        
        self.declare_parameter("settings_name", "EuRoC")
        self.declare_parameter("bag_path", "TEST_ROSBAGS/rosbag2_2026_01_24-13_56_00")
        self.declare_parameter("input_topic", "/camera/image/raw")
        self.declare_parameter("playback_rate", 1.0)
        self.declare_parameter("use_roi_mask", False)
        self.declare_parameter("roi_mask_top_percent", 0.6)
        self.declare_parameter("enable_imu", False)
        self.declare_parameter("imu_topic", "/sensor/imu")
        self.declare_parameter("auto_detect_imu_topic", True)
        
        self.settings_name = str(self.get_parameter('settings_name').value)
        self.bag_path_rel = str(self.get_parameter('bag_path').value)
        self.input_topic = str(self.get_parameter('input_topic').value)
        self.playback_rate = float(self.get_parameter('playback_rate').value)
        self.use_roi_mask = bool(self.get_parameter('use_roi_mask').value)
        self.roi_mask_top_percent = float(self.get_parameter('roi_mask_top_percent').value)
        self.enable_imu = bool(self.get_parameter('enable_imu').value)
        self.imu_topic = str(self.get_parameter('imu_topic').value).strip()
        self.auto_detect_imu_topic = bool(self.get_parameter('auto_detect_imu_topic').value)
        self.use_imu_stream = self.enable_imu and (self.auto_detect_imu_topic or len(self.imu_topic) > 0)

        self.package_share_dir = self._resolve_package_share_dir()
        self.package_source_dir = self._resolve_package_source_dir(self.package_share_dir)
        self.bag_path = self._resolve_user_path(self.bag_path_rel)

        clamped_roi = min(max(self.roi_mask_top_percent, 0.0), 1.0)
        if not np.isclose(clamped_roi, self.roi_mask_top_percent):
            self.get_logger().warn(
                "roi_mask_top_percent %.2f außerhalb [0,1], verwende %.2f",
                self.roi_mask_top_percent,
                clamped_roi,
            )
        self.roi_mask_top_percent = clamped_roi
        
        self.default_encoding = "bgr8"
        
        self.get_logger().info("=" * 70)
        self.get_logger().info("ROS2 Bag Driver Node gestartet")
        self.get_logger().info("=" * 70)
        self.get_logger().info(f"Settings Name: {self.settings_name}")
        self.get_logger().info(f"Bag Pfad: {self.bag_path}")
        self.get_logger().info(f"Input Topic: {self.input_topic}")
        self.get_logger().info(f"Wiedergaberate: {self.playback_rate}x")
        self.get_logger().info(f"ROI-Masking: {'Aktiviert' if self.use_roi_mask else 'Deaktiviert'}")
        if self.use_roi_mask:
            self.get_logger().info(f"ROI-Bereich: Obere {self.roi_mask_top_percent*100:.0f}% des Bildes")
        imu_status = "Aktiviert" if self.use_imu_stream else "Deaktiviert"
        self.get_logger().info(f"IMU-Wiedergabe: {imu_status}")
        if self.use_imu_stream:
            if self.auto_detect_imu_topic and not self.imu_topic:
                self.get_logger().info("IMU-Topic: Automatische Erkennung (sensor_msgs/msg/Imu)")
            else:
                self.get_logger().info(f"IMU-Topic: {self.imu_topic}")
        elif self.enable_imu:
            self.get_logger().warn(
                "IMU-Wiedergabe angefordert, aber kein Topic konfiguriert. Bitte Topic setzen oder Auto-Detect aktivieren."
            )
        self.get_logger().info("=" * 70)
        
        if not os.path.exists(self.bag_path):
            self.get_logger().error(f"FEHLER: Bag-Pfad existiert nicht: {self.bag_path}")
            sys.exit(1)
        
        self.pub_exp_config_name = "/mono_py_driver/experiment_settings"
        self.sub_exp_ack_name = "/mono_py_driver/exp_settings_ack"
        self.pub_img_to_agent_name = "/camera/image/raw"
        self.pub_timestep_to_agent_name = "/mono_py_driver/timestep_msg"
        self.pub_imu_to_agent_name = "/mono_py_driver/imu_msg"
        
        self.publish_exp_config_ = self.create_publisher(
            String, self.pub_exp_config_name, 1
        )
        self.publish_img_msg_ = self.create_publisher(
            Image, self.pub_img_to_agent_name, 10
        )
        self.publish_timestep_msg_ = self.create_publisher(
            Float64, self.pub_timestep_to_agent_name, 10
        )
        self.publish_imu_msg_ = self.create_publisher(
            Imu, self.pub_imu_to_agent_name, 50
        )
        
        self.subscribe_exp_ack_ = self.create_subscription(
            String, self.sub_exp_ack_name, self.ack_callback, 10
        )
        
        self.send_config = True
        self.exp_config_msg = self.settings_name
        
        self.frame_id = 0
        
        self._cached_roi_height = None
        
        self.get_logger().info("Node initialisiert, starte Handshake mit C++ Node...")

    def _resolve_package_share_dir(self) -> str:
        """Resolves the share directory path of the package."""
        try:
            return str(Path(get_package_share_directory("ros2_orb_slam3")))
        except (PackageNotFoundError, ValueError) as exc:
            fallback = Path(__file__).resolve().parent.parent
            print(f"[WARN] Using fallback package path {fallback}: {exc}")
            return str(fallback)

    def _resolve_package_source_dir(self, share_dir: str):
        """Locates the absolute source directory path corresponding to the package layout."""
        share_path = Path(share_dir)
        for parent in share_path.parents:
            candidate = parent / "src" / "ros2_orb_slam3"
            if candidate.exists():
                return str(candidate)
        return None

    def _resolve_relative_path(self, relative_path: str) -> str:
        """Matches a relative file path against package resources or source trees."""
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
        """Expands home path shorthand tokens and unrolls relative system mappings."""
        expanded = Path(configured_path).expanduser()
        if expanded.is_absolute():
            return str(expanded)

        return self._resolve_relative_path(str(expanded))

    def _find_topic_of_type(self, topic_types, target_type: str):
        """Queries the bag's active metadata catalog for a specific ROS message datatype."""
        for topic_meta in topic_types:
            if target_type in topic_meta.type:
                return topic_meta.name, topic_meta.type
        return None, None

    def _image_msg_to_array(self, msg: Image):
        """Parses raw buffer data of a ROS Image into a compatible multidimensional NumPy array."""
        encoding = (msg.encoding or self.default_encoding).lower()
        if encoding == "passthrough":
            encoding = self.default_encoding
        if encoding not in ENCODING_SPECS:
            raise ValueError(f"Encoding {encoding} wird nicht unterstützt")

        dtype, channels = ENCODING_SPECS[encoding]
        expected_pixels = msg.width * msg.height * channels
        np_buffer = np.frombuffer(msg.data, dtype=dtype)
        if np_buffer.size < expected_pixels:
            raise ValueError(
                f"Bilddaten zu klein (got {np_buffer.size}, expected {expected_pixels})"
            )
        np_buffer = np_buffer[:expected_pixels]
        if channels == 1:
            image = np_buffer.reshape((msg.height, msg.width))
        else:
            image = np_buffer.reshape((msg.height, msg.width, channels))
        return image, encoding

    def _array_to_image_msg(self, image: np.ndarray, msg: Image, encoding: str):
        """Serializes a processed NumPy matrix back into a formatted ROS Image structure."""
        out_msg = msg
        out_msg.height = int(image.shape[0])
        out_msg.width = int(image.shape[1])
        channels = 1 if image.ndim == 2 else int(image.shape[2])
        out_msg.encoding = encoding
        out_msg.is_bigendian = 0
        out_msg.step = out_msg.width * channels * image.dtype.itemsize
        out_msg.data = image.tobytes()
        return out_msg
    
    def apply_roi_mask(self, cv_image):
        """Applies a geometric row-based mask to drop the lower vehicle perspective layers."""
        height = cv_image.shape[0]
        
        if self._cached_roi_height is None or self._cached_roi_height[0] != height:
            roi_h = max(0, min(height, int(height * self.roi_mask_top_percent)))
            self._cached_roi_height = (height, roi_h)
        
        roi_height = self._cached_roi_height[1]
        if roi_height < height:
            masked_image = cv_image.copy()
            masked_image[roi_height:] = 0
            return masked_image

        return cv_image

    def ack_callback(self, msg):
        """Receives configuration confirmation signals back from the tracking node."""
        self.get_logger().info(f"Acknowledgement erhalten: {msg.data}")
        if msg.data == "ACK":
            self.send_config = False
    
    def handshake_with_cpp_node(self):
        """Broadcasts internal settings string until an ACK handshake token trips."""
        if self.send_config:
            msg = String()
            msg.data = self.exp_config_msg
            self.publish_exp_config_.publish(msg)
            time.sleep(0.001)
    
    def play_bag(self):
        """Unpacks sequential data layers from the bag stream and outputs image/IMU records."""
        storage_options = StorageOptions(uri=self.bag_path, storage_id='mcap')
        converter_options = ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'
        )
        
        reader = SequentialReader()
        reader.open(storage_options, converter_options)
        
        topic_types = reader.get_all_topics_and_types()
        type_map = {topic.name: topic.type for topic in topic_types}

        if self.input_topic not in type_map:
            requested_topic = self.input_topic
            detected_name, _ = self._find_topic_of_type(topic_types, "sensor_msgs/msg/Image")
            if detected_name is not None:
                self.get_logger().warn(
                    f"Input-Topic '{requested_topic or '<leer>'}' nicht gefunden. Verwende automatisch {detected_name}."
                )
                self.input_topic = detected_name
            else:
                self.get_logger().error(
                    f"FEHLER: Topic {requested_topic} nicht im Bag gefunden!"
                )
                self.get_logger().info("Verfügbare Topics:")
                for topic in topic_types:
                    self.get_logger().info(f"  - {topic.name} ({topic.type})")
                return

        imu_msg_type = None
        if self.use_imu_stream:
            imu_type_name = None
            imu_topic_in_bag = self.imu_topic in type_map

            if (not self.imu_topic or not imu_topic_in_bag) and self.auto_detect_imu_topic:
                detected_name, detected_type = self._find_topic_of_type(topic_types, "sensor_msgs/msg/Imu")
                if detected_name is not None:
                    self.imu_topic = detected_name
                    imu_type_name = detected_type
                    imu_topic_in_bag = True
                    self.get_logger().info(
                        f"IMU-Topic automatisch erkannt: {self.imu_topic}"
                    )
                else:
                    self.get_logger().warn(
                        "Kein sensor_msgs/msg/Imu Topic im Bag gefunden. IMU-Wiedergabe wird deaktiviert."
                    )
                    self.use_imu_stream = False

            if self.use_imu_stream:
                if not imu_topic_in_bag:
                    self.get_logger().warn(
                        f"IMU-Topic {self.imu_topic} nicht gefunden. IMU-Wiedergabe wird deaktiviert."
                    )
                    self.use_imu_stream = False
                else:
                    if imu_type_name is None:
                        imu_type_name = type_map[self.imu_topic]
                    if "sensor_msgs/msg/Imu" not in imu_type_name:
                        self.get_logger().warn(
                            f"Topic {self.imu_topic} ist kein sensor_msgs/Imu ({imu_type_name}). IMU-Wiedergabe deaktiviert."
                        )
                        self.use_imu_stream = False
                    else:
                        imu_msg_type = get_message(imu_type_name)

        self.get_logger().info(f"Starte Wiedergabe von {self.input_topic}...")
        if self.use_imu_stream:
            self.get_logger().info(f"IMU-Wiedergabe gekoppelt mit Topic {self.imu_topic}.")
        
        image_msg_type = get_message(type_map[self.input_topic])
        
        last_timestamp = None
        message_count = 0
        start_time = time.time()
        imu_message_count = 0
        last_log_time = start_time
        log_interval = 2.0
        
        timestep_msg = Float64()
        spin_counter = 0
        
        while reader.has_next():
            spin_counter += 1
            if spin_counter >= 10:
                rclpy.spin_once(self, timeout_sec=0.0)
                spin_counter = 0
            
            (topic, data, timestamp) = reader.read_next()
            
            if topic != self.input_topic and not (self.use_imu_stream and topic == self.imu_topic):
                continue

            if self.use_imu_stream and topic == self.imu_topic:
                if imu_msg_type is None:
                    continue
                try:
                    imu_msg = deserialize_message(data, imu_msg_type)
                    self.publish_imu_msg_.publish(imu_msg)
                    imu_message_count += 1
                except Exception as e:
                    self.get_logger().error(f"Fehler beim Publishen des IMU-Topics: {e}")
                continue
            
            msg = deserialize_message(data, image_msg_type)
            
            if last_timestamp is not None and self.playback_rate > 0:
                time_diff = (timestamp - last_timestamp) / 1e9
                sleep_time = time_diff / self.playback_rate
                if sleep_time > 0:
                    time.sleep(sleep_time)
            
            last_timestamp = timestamp
            
            try:
                if self.use_roi_mask:
                    cv_image, encoding = self._image_msg_to_array(msg)
                    cv_image = self.apply_roi_mask(cv_image)
                    msg = self._array_to_image_msg(cv_image, msg, msg.encoding or self.default_encoding)
            except Exception as e:
                self.get_logger().error(f"Fehler bei Bildaufbereitung: {e}")
            
            timestep_msg.data = float(timestamp) / 1e9
            
            try:
                self.publish_timestep_msg_.publish(timestep_msg)
                self.publish_img_msg_.publish(msg)
                self.frame_id += 1
                message_count += 1
                
                current_time = time.time()
                if current_time - last_log_time > log_interval:
                    elapsed = current_time - start_time
                    fps = message_count / elapsed if elapsed > 0 else 0
                    self.get_logger().info(
                        f"Verarbeitet: {message_count} Frames ({fps:.1f} FPS)"
                    )
                    last_log_time = current_time
            
            except Exception as e:
                self.get_logger().error(f"Fehler beim Publishen: {e}")
        
        elapsed = time.time() - start_time
        avg_fps = message_count / elapsed if elapsed > 0 else 0
        self.get_logger().info("=" * 70)
        self.get_logger().info("Bag-Wiedergabe abgeschlossen!")
        self.get_logger().info(f"Gesamt Frames: {message_count}")
        self.get_logger().info(f"Gesamtzeit: {elapsed:.2f} Sekunden")
        self.get_logger().info(f"Durchschnittliche FPS: {avg_fps:.2f}")
        if self.use_imu_stream:
            self.get_logger().info(f"IMU-Nachrichten veröffentlicht: {imu_message_count}")
        self.get_logger().info("=" * 70)


def main(args=None):
    """Initializes and runs the driver node, handling the sync handshake prior to data playback."""
    rclpy.init(args=args)
    
    node = RosbagDriverNode("rosbag_driver_node")
    
    node.get_logger().info("Warte auf Handshake mit C++ Node...")
    while node.send_config:
        node.handshake_with_cpp_node()
        rclpy.spin_once(node, timeout_sec=0.05)
        if not node.send_config:
            break
    
    node.get_logger().info("Handshake erfolgreich! Starte Bag-Wiedergabe...")
    
    try:
        node.play_bag()
    except KeyboardInterrupt:
        node.get_logger().info("Wiedergabe durch Benutzer abgebrochen.")
    except Exception as e:
        node.get_logger().error(f"Fehler bei Bag-Wiedergabe: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()