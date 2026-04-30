# Copyright 2026 SwissCAT+
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster
from tf_transformations import (
    concatenate_matrices,
    quaternion_from_euler,
    quaternion_matrix,
    translation_matrix,
)

from em_robot.diagnostic_utils import build_diagnostic_array, build_diagnostic_status
from em_robot.marker_map_loader import load_marker_map_config
from em_robot.transform_utils import build_transform


class MarkerMapPublisher(Node):
    def __init__(self):
        super().__init__("marker_map_publisher")

        self.declare_parameter("config_file", "")
        self.declare_parameter("diagnostics_rate_hz", 1.0)

        self.config_file = str(self.get_parameter("config_file").value)
        diagnostics_rate_hz = float(self.get_parameter("diagnostics_rate_hz").value)

        if not self.config_file:
            raise ValueError("marker_map_publisher requires a config_file parameter")

        self.marker_map = load_marker_map_config(self.config_file)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.diagnostics_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)

        transforms = self._build_transforms()
        if transforms:
            self.static_tf_broadcaster.sendTransform(transforms)
            self.get_logger().info(
                f"Published {len(transforms)} marker transforms from {self.config_file}"
            )
        else:
            self.get_logger().warn(
                f"Marker map {self.config_file} contains no markers; localization corrections are disabled."
            )

        timer_period = 1.0 / diagnostics_rate_hz if diagnostics_rate_hz > 0.0 else 1.0
        self.diagnostics_timer = self.create_timer(timer_period, self.publish_diagnostics)

    def _build_transforms(self):
        transforms = []
        map_frame = self.marker_map["map_frame"]
        marker_prefix = self.marker_map["marker_prefix"]
        stamp = self.get_clock().now().to_msg()

        for marker in self.marker_map["markers"]:
            transform_matrix = concatenate_matrices(
                translation_matrix([marker["x"], marker["y"], marker["z"]]),
                quaternion_matrix(
                    quaternion_from_euler(
                        marker["roll"],
                        marker["pitch"],
                        marker["yaw"],
                    )
                ),
            )
            transforms.append(
                build_transform(
                    map_frame,
                    f"{marker_prefix}{marker['id']}",
                    transform_matrix,
                    stamp,
                )
            )

        return transforms

    def publish_diagnostics(self):
        marker_count = len(self.marker_map["markers"])
        level = DiagnosticStatus.OK if marker_count > 0 else DiagnosticStatus.WARN
        message = "Marker map loaded" if marker_count > 0 else "Marker map is empty"
        status = build_diagnostic_status(
            "em_robot/marker_map",
            level,
            message,
            values={
                "config_file": self.config_file,
                "map_frame": self.marker_map["map_frame"],
                "marker_prefix": self.marker_map["marker_prefix"],
                "marker_count": marker_count,
            },
        )
        self.diagnostics_pub.publish(
            build_diagnostic_array([status], self.get_clock().now().to_msg())
        )


def main(args=None):
    rclpy.init(args=args)
    node = MarkerMapPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down marker map publisher...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
