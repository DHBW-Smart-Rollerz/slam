#!/usr/bin/env python3
"""ROS 2 node for monitoring and displaying image topics in the preprocessing pipeline."""

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class ImageDebugViewer(Node):
    """Subscribes to pipeline image topics and manages OpenCV conversions for visualization."""

    def __init__(self):
        super().__init__("image_debug_viewer")
        
        self.bridge = CvBridge()
        
        self.latest_raw = None
        self.latest_undistorted = None
        self.latest_bev = None
        
        self.raw_count = 0
        self.undist_count = 0
        self.bev_count = 0
        
        self.sub_raw = self.create_subscription(
            Image, "/camera/image/raw", self.callback_raw, 10
        )
        self.sub_undist = self.create_subscription(
            Image, "/camera/image/undistorted", self.callback_undistorted, 10
        )
        self.sub_bev = self.create_subscription(
            Image, "/camera/image/bev", self.callback_bev, 10
        )
        
        self.sub_orb_input = self.create_subscription(
            Image, "/mono_py_driver/img_msg", self.callback_orb_input, 10
        )
        
        self.get_logger().info("Image Debug Viewer started.")

    def _convert_to_mono(self, msg):
        """Converts ROS image message to a single-channel OpenCV image safely."""
        # Keep source encoding where possible (e.g. 8UC1), then normalize to mono if needed.
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        if img.ndim == 2:
            return img
        if img.ndim == 3:
            return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        raise ValueError(f"Unsupported image shape: {img.shape}")
        
    def callback_raw(self, msg):
        """Processes the raw input image."""
        try:
            self.latest_raw = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.raw_count += 1
        except Exception as e:
            self.get_logger().error(f"Raw image conversion error: {e}")
    
    def callback_undistorted(self, msg):
        """Processes the undistorted camera image."""
        try:
            self.latest_undistorted = self._convert_to_mono(msg)
            self.undist_count += 1
        except Exception as e:
            self.get_logger().error(f"Undistorted image conversion error: {e}")
    
    def callback_bev(self, msg):
        """Processes the bird's eye view image."""
        try:
            self.latest_bev = self._convert_to_mono(msg)
            self.bev_count += 1
        except Exception as e:
            self.get_logger().error(f"BEV image conversion error: {e}")
    
    def callback_orb_input(self, msg):
        """Logs the receipt of the ORB-SLAM3 input message."""
        self.get_logger().info(
            f"ORB-SLAM3 input received: {msg.height}x{msg.width}, {msg.encoding}"
        )


def main(args=None):
    """Initializes the ROS 2 node and handles the OpenCV display loop."""
    rclpy.init(args=args)
    viewer = ImageDebugViewer()
    
    try:
        cv2.namedWindow("Pipeline Debug", cv2.WINDOW_NORMAL)
        
        while rclpy.ok():
            # spin_once processes callbacks while yielding to the OpenCV GUI thread
            try:
                rclpy.spin_once(viewer, timeout_sec=0.5)
            except ExternalShutdownException:
                break
            
            images = []
            labels = []
            
            if viewer.latest_raw is not None:
                images.append(viewer.latest_raw)
                labels.append(f"RAW ({viewer.raw_count})")
            
            if viewer.latest_undistorted is not None:
                undist_bgr = cv2.cvtColor(viewer.latest_undistorted, cv2.COLOR_GRAY2BGR)
                images.append(undist_bgr)
                labels.append(f"UNDISTORTED ({viewer.undist_count})")
            
            if viewer.latest_bev is not None:
                bev_bgr = cv2.cvtColor(viewer.latest_bev, cv2.COLOR_GRAY2BGR)
                images.append(bev_bgr)
                labels.append(f"BEV ({viewer.bev_count})")
            
            if images:
                target_height = 480
                resized = []

                for img in images:
                    h, w = img.shape[:2]
                    scale = target_height / h
                    new_w = int(w * scale)
                    resized.append(cv2.resize(img, (new_w, target_height)))
                
                for img, label in zip(resized, labels):
                    cv2.putText(
                        img, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2
                    )
                
                collage = np.hstack(resized)
                cv2.imshow("Pipeline Debug", collage)
                
                total = viewer.raw_count + viewer.undist_count + viewer.bev_count
                if total % 30 == 0 and total > 0:
                    print(f"Status: RAW={viewer.raw_count} | UNDIST={viewer.undist_count} | BEV={viewer.bev_count}")
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        viewer.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()