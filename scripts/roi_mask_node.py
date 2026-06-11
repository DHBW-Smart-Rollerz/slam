#!/usr/bin/env python3
"""ROS 2 node to apply a Region of Interest (ROI) mask and handle frame deduplication."""

import zlib

import numpy as np

import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from sensor_msgs.msg import Image


class RoiMaskNode(Node):
    """Apply a simple top-region ROI mask to incoming images."""

    def __init__(self):
        """Initializes parameters, caches, subscriptions, and publishers for the ROI node."""
        super().__init__("roi_mask_node")

        self.input_topic = str(
            self.declare_parameter(
                "input_topic", "/camera/image/undistorted"
            ).value
        )
        self.output_topic = str(
            self.declare_parameter(
                "output_topic", "/camera/image/roi_masked"
            ).value
        )
        self.enabled = bool(self.declare_parameter("enabled", True).value)
        self.deduplicate_by_stamp = bool(
            self.declare_parameter("deduplicate_by_stamp", True).value
        )
        self.roi_mask_top_percent = float(
            self.declare_parameter("roi_mask_top_percent", 0.45).value
        )

        clamped_roi = min(max(self.roi_mask_top_percent, 0.0), 1.0)
        if clamped_roi != self.roi_mask_top_percent:
            self.get_logger().warn(
                "roi_mask_top_percent %.2f outside [0,1], using %.2f"
                % (self.roi_mask_top_percent, clamped_roi)
            )
        self.roi_mask_top_percent = clamped_roi

        self.bridge = CvBridge()
        self._cached_roi_height = None
        self._last_message_key = None
        self._dropped_duplicates = 0

        self.sub_image = self.create_subscription(
            Image, self.input_topic, self.on_image, 10
        )
        self.pub_image = self.create_publisher(Image, self.output_topic, 10)

        self.get_logger().info(
            "ROI node ready: input=%s output=%s enabled=%s dedup=%s top=%.2f"
            % (
                self.input_topic,
                self.output_topic,
                "true" if self.enabled else "false",
                "true" if self.deduplicate_by_stamp else "false",
                self.roi_mask_top_percent,
            )
        )

    def _message_key(self, msg: Image):
        """Generates a unique tracking key for the image based on timestamp or content."""
        stamp_key = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        if stamp_key != (0, 0):
            return ("stamp", stamp_key[0], stamp_key[1])

        return (
            "content",
            msg.height,
            msg.width,
            msg.step,
            msg.encoding,
            zlib.crc32(msg.data),
        )

    def _is_duplicate(self, msg: Image) -> bool:
        """Checks if the current image message matches the previously received message key."""
        key = self._message_key(msg)

        if self._last_message_key == key:
            self._dropped_duplicates += 1
            self.get_logger().debug(
                "Dropping duplicate image key %s (count=%d)"
                % (key, self._dropped_duplicates)
            )
            return True

        self._last_message_key = key
        return False

    def _apply_mask(self, image: np.ndarray) -> np.ndarray:
        """Applies the geometric ROI mask by zeroing out rows below the threshold percentage."""
        height = image.shape[0]

        if (
            self._cached_roi_height is None
            or self._cached_roi_height[0] != height
        ):
            roi_h = max(
                0, min(height, int(height * self.roi_mask_top_percent))
            )
            self._cached_roi_height = (height, roi_h)

        roi_height = self._cached_roi_height[1]
        if roi_height < height:
            masked = image.copy()
            masked[roi_height:] = 0
            return masked

        return image

    def on_image(self, msg: Image):
        """Callback that processes incoming image messages, handles deduplication, and applies the mask."""
        if self.deduplicate_by_stamp and self._is_duplicate(msg):
            return

        if not self.enabled:
            self.pub_image.publish(msg)
            return

        try:
            cv_img = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding="passthrough"
            )
            cv_masked = self._apply_mask(cv_img)
            out_msg = self.bridge.cv2_to_imgmsg(
                cv_masked, encoding=msg.encoding
            )
            out_msg.header = msg.header
            self.pub_image.publish(out_msg)
        except CvBridgeError as exc:
            self.get_logger().error(f"CvBridge error in roi_mask_node: {exc}")


def main(args=None):
    """Main function to initialize the node and execute the ROS 2 spin loop."""
    rclpy.init(args=args)
    node = RoiMaskNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()