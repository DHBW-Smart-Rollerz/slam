#!/usr/bin/env python3
"""ROS 2 node for monitoring image topic statistics in the preprocessing pipeline."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time
from collections import defaultdict


class SimpleImageMonitor(Node):
    """Monitors and logs frame counts and frequencies of various image topics."""

    def __init__(self):
        """Initializes the monitor node and sets up subscriptions for image topics."""
        super().__init__("simple_image_monitor")

        self.declare_parameter("orb_input_topic", "/camera/image/undistorted")
        self.orb_input_topic = str(self.get_parameter("orb_input_topic").value)
        
        self.bridge = CvBridge()
        self.start_time = time.time()
        self.frame_counts = defaultdict(int)
        self.last_print = time.time()
        
        self.subscribers = {}
        self.subscribe_to_topic("/camera/image/raw")
        self.subscribe_to_topic("/camera/image/undistorted")
        self.subscribe_to_topic("/camera/image/bev")
        self.subscribe_to_topic(
            self.orb_input_topic,
            "ORB-SLAM3 Input (effective)",
        )
        
        print("\nImage Pipeline Monitor")
        print("=" * 70)
        print("Topics being monitored:")
        print("  ✓ /camera/image/raw - Rosbag raw input")
        print("  ✓ /camera/image/undistorted - After preprocessing")
        print("  ✓ /camera/image/bev - Bird's eye view")
        print(f"  ✓ {self.orb_input_topic} - ORB-SLAM3 effective input")
        print("  i mono_node internal topic is /mono_py_driver/img_msg")
        print("  i effective topic can differ because of remapping")
        print("=" * 70)
        print()
    
    def subscribe_to_topic(self, topic_name, label=None):
        """Creates a subscription for a specific image topic."""
        if label is None:
            label = topic_name
        
        self.subscribers[topic_name] = {
            "label": label,
            "sub": self.create_subscription(
                Image,
                topic_name,
                lambda msg, t=topic_name: self.image_callback(msg, t),
                10,
            ),
        }
    
    def image_callback(self, msg, topic_name):
        """Callback triggered upon receiving a new image message."""
        self.frame_counts[topic_name] += 1
        
        now = time.time()
        if now - self.last_print > 2.0:
            self.print_status()
            self.last_print = now
    
    def print_status(self):
        """Calculates and prints the current frame statistics and frequency."""
        elapsed = time.time() - self.start_time
        
        print(f"\nElapsed: {elapsed:.1f}s")
        print("-" * 70)
        
        for topic_name, info in self.subscribers.items():
            count = self.frame_counts[topic_name]
            fps = count / elapsed if elapsed > 0 else 0
            status = "✓" if count > 0 else "✗"
            print(
                f"{status} {info['label']:<40} "
                f"Frames: {count:>5}  FPS: {fps:>5.2f}"
            )
        
        total_frames = sum(self.frame_counts.values())
        if total_frames > 0:
            print("-" * 70)
            if self.frame_counts[self.orb_input_topic] > 0:
                print("SUCCESS: ORB-SLAM3 is receiving images!")
            else:
                print(
                    "WARNING: No frames on configured ORB input topic "
                    f"{self.orb_input_topic}"
                )
        print()


def main(args=None):
    """Main entry point to initialize and spin the monitor node."""
    rclpy.init(args=args)
    monitor = SimpleImageMonitor()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        print("\n\n👋 Monitor stopped")
    finally:
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()