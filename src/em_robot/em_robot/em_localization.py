#!/usr/bin/env python3
import os
import yaml
import rclpy
from rclpy.node import Node
import numpy as np
import cv2 as cv

from tf2_ros import (
    TransformBroadcaster,
    StaticTransformBroadcaster,
    Buffer,
    TransformListener,
)
from geometry_msgs.msg import TransformStamped, PoseStamped
from nav_msgs.msg import Odometry
from tf_transformations import (
    quaternion_from_euler,
    quaternion_from_matrix,
    quaternion_matrix,
    translation_matrix,
    translation_from_matrix,
    concatenate_matrices,
)
from picamera2 import Picamera2
from libcamera import controls
from ament_index_python.packages import get_package_share_directory
from em_robot_srv.srv import SetInitialPose

# --- ArUco Detection Setup ---
dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_ARUCO_ORIGINAL)
detector_params = cv.aruco.DetectorParameters()

# Robustness tweaks
detector_params.cornerRefinementMethod = cv.aruco.CORNER_REFINE_SUBPIX
detector_params.cornerRefinementWinSize = 5
detector_params.cornerRefinementMaxIterations = 50
detector_params.cornerRefinementMinAccuracy = 0.01

detector_params.minCornerDistanceRate = 0.05
detector_params.minDistanceToBorder = 3
detector_params.perspectiveRemovePixelPerCell = 10

detector_params.adaptiveThreshWinSizeMin = 3
detector_params.adaptiveThreshWinSizeMax = 23
detector_params.adaptiveThreshWinSizeStep = 10
detector_params.adaptiveThreshConstant = 7

detector = cv.aruco.ArucoDetector(dictionary, detector_params)

# Simple quality gates (no voting)
MIN_AREA_PX = 400.0       # discard very small detections
MAX_REPROJ_ERR_PX = 2.5   # discard if mean reprojection error is high


def _marker_area_px(corners4x2: np.ndarray) -> float:
    return float(cv.contourArea(corners4x2.astype(np.float32)))

def _mean_reprojection_error(corners4x2, rvec, tvec, marker_size, K, D) -> float:
    # define marker corners in its own frame: TL, TR, BR, BL (matches OpenCV order)
    s = marker_size * 0.5
    obj = np.array([[-s,  s, 0.0],
                    [ s,  s, 0.0],
                    [ s, -s, 0.0],
                    [-s, -s, 0.0]], dtype=np.float32).reshape(-1, 1, 3)
    img_proj, _ = cv.projectPoints(obj, rvec, tvec, K, D)  # (4,1,2)
    img_proj = img_proj.reshape(-1, 2)
    err = np.linalg.norm(img_proj - corners4x2.astype(np.float32), axis=1)
    return float(err.mean())


def detect_aruco_corners(frame):
    """Detect ArUco markers and return their corners and IDs."""
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY) if len(frame.shape) == 3 else frame
    gray = cv.GaussianBlur(gray, (3, 3), 0)  # small denoise helps robustness
    corners_list, ids, _ = detector.detectMarkers(gray)
    if ids is None or len(ids) == 0:
        return []
    return [{'id': int(i), 'corners': c[0]} for c, i in zip(corners_list, ids.flatten())]


# --- Main Node ---
class MarkerLocalizationNode(Node):
    def __init__(self):
        super().__init__('marker_localization_node')
        self.get_logger().info("Starting MarkerLocalizationNode...")

        # --- Load Camera Configuration ---
        pkg_share = get_package_share_directory('em_robot')
        config_path = os.path.join(pkg_share, 'config/calibration.yaml')
        self.get_logger().info(f"Loading configuration: {config_path}")
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)

        self.camera_matrix = np.array(config.get('camera_matrix', []), dtype=np.float32)
        self.dist_coeffs = np.array(config.get('dist_coeff', []), dtype=np.float32)
        self.camera_height = config.get('camera_height', 0.337)
        self.marker_size = config.get('marker_size', 0.038)

        # --- Initialize Camera ---
        self.size = (1536, 864)
        self.picam2 = Picamera2()
        preview_cfg = self.picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": self.size})
        self.picam2.configure(preview_cfg)
        self.picam2.start()
        self.picam2.set_controls({"AfMode": controls.AfModeEnum.Manual, "LensPosition": 8.0})
        self.get_logger().info("Camera initialized.")

        # --- ROS Setup ---
        self.tf_broadcaster = TransformBroadcaster(self)
               self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pose_pub = self.create_publisher(PoseStamped, 'aruco_markers_pose', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)

        self.set_pose_srv = self.create_service(SetInitialPose, 'set_initial_pose', self.handle_set_initial_pose)

        self.latest_odom_pose = None
        self.last_map_to_odom = None
        self.last_marker_time = self.get_clock().now()

        # --- Initialize TF ---
        self.publish_static_transform()
        self.publish_initial_map_to_odom()

        # --- Timers ---
        self.timer = self.create_timer(0.2, self.process_frame)  # 5 Hz
        self.map_odom_timer = self.create_timer(0.2, self.broadcast_last_map_to_odom)

    # -------------------------------------------------------------------------
    # Callbacks and Core Functions
    # -------------------------------------------------------------------------
    def odom_callback(self, msg: Odometry):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        self.latest_odom_pose = concatenate_matrices(
            translation_matrix([pos.x, pos.y, pos.z]),
            quaternion_matrix([ori.x, ori.y, ori.z, ori.w])
        )

    def handle_set_initial_pose(self, request, response):
        quat_map_base = quaternion_from_euler(0.0, 0.0, request.yaw)
        T_map_base = concatenate_matrices(
            translation_matrix([request.x, request.y, 0.0]),
            quaternion_matrix(quat_map_base)
        )

        if self.latest_odom_pose is None:
            self.get_logger().warn("No odometry available — cannot set initial pose.")
            response.success = False
            return response

        T_map_odom = T_map_base @ np.linalg.inv(self.latest_odom_pose)
        trans = translation_from_matrix(T_map_odom)
        quat = quaternion_from_matrix(T_map_odom)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "odom"
        t.transform.translation.x, t.transform.translation.y = trans[:2]
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = quat
        self.last_map_to_odom = t
        self.tf_broadcaster.sendTransform(t)

        self.get_logger().info(f"Updated map→odom from service: x={request.x}, y={request.y}, yaw={request.yaw}")
        response.success = True
        return response

    # -------------------------------------------------------------------------
    # Transform Initialization
    # -------------------------------------------------------------------------
    def publish_static_transform(self):
        static_t = TransformStamped()
        static_t.header.stamp = self.get_clock().now().to_msg()
        static_t.header.frame_id = "camera_frame"
        static_t.child_frame_id = "cam_base_link"
        static_t.transform.translation.x = 0.176
        static_t.transform.translation.y = 0.0
        static_t.transform.translation.z = 0.0
        qz180 = quaternion_from_euler(0.0, 0.0, np.pi)
        static_t.transform.rotation.x, static_t.transform.rotation.y, static_t.transform.rotation.z, static_t.transform.rotation.w = qz180
        self.static_tf_broadcaster.sendTransform([static_t])
        self.get_logger().info("Published static transform: camera_frame → cam_base_link")

    def publish_initial_map_to_odom(self):
        quat = quaternion_from_euler(0.0, 0.0, np.pi / 2)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "odom"
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = quat
        self.last_map_to_odom = t
        self.tf_broadcaster.sendTransform(t)
        self.get_logger().warn("Initialized map→odom at identity (0,0,0).")

    def broadcast_last_map_to_odom(self):
        if self.last_map_to_odom:
            self.last_map_to_odom.header.stamp = self.get_clock().now().to_msg()
            self.tf_broadcaster.sendTransform(self.last_map_to_odom)

    # -------------------------------------------------------------------------
    # Vision Processing
    # -------------------------------------------------------------------------
    def process_frame(self):
        frame = self.picam2.capture_array()
        detections = detect_aruco_corners(frame)
        if not detections:
            return

        # Keep only reasonably large candidates (more reliable)
        detections = [d for d in detections if _marker_area_px(d['corners']) >= MIN_AREA_PX]
        if not detections:
            return

        # Prefer the largest (closest) marker in view
        det = max(detections, key=lambda d: _marker_area_px(d['corners']))
        marker_id = det['id']
        corners = det['corners'].astype(np.float32).reshape(1, 4, 2)  # shape for OpenCV APIs

        # Make sure this marker exists in the TF map (your original guard)
        try:
            tf_map_to_aruco = self.tf_buffer.lookup_transform(
                "map", f"aruco_{marker_id}", rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
        except Exception:
            # Unknown marker in map → ignore
            return

        # --- Robust pose: estimate marker pose via PnP
        # rvec/tvec: pose of the marker w.r.t the camera (marker->camera transform)
        rvecs, tvecs, _ = cv.aruco.estimatePoseSingleMarkers(
            corners,
            self.marker_size,
            self.camera_matrix,
            self.dist_coeffs
        )
        if rvecs is None or len(rvecs) == 0:
            return

        rvec = rvecs[0]
        tvec = tvecs[0]

        # Reprojection error filter (reject shaky ID without voting)
        reproj_err = _mean_reprojection_error(det['corners'], rvec, tvec,
                                              self.marker_size, self.camera_matrix, self.dist_coeffs)
        if reproj_err > MAX_REPROJ_ERR_PX:
            self.get_logger().debug(f"Rejected marker {marker_id} due to reproj error {reproj_err:.2f}px")
            return

        # Build T_aruco_camera directly from rvec/tvec (marker->camera)
        R_cm, _ = cv.Rodrigues(rvec)  # rotation from marker to camera
        t_cm = tvec.reshape(3)        # translation of marker in camera frame
        T_aruco_camera = np.eye(4, dtype=np.float32)
        T_aruco_camera[:3, :3] = R_cm
        T_aruco_camera[:3, 3] = t_cm

        # camera_frame -> cam_base_link via TF (single source of truth)
        try:
            t_cam_to_base = self.tf_buffer.lookup_transform(
                "camera_frame", "cam_base_link", rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            T_camera_cam_base = concatenate_matrices(
                translation_matrix([
                    t_cam_to_base.transform.translation.x,
                    t_cam_to_base.transform.translation.y,
                    t_cam_to_base.transform.translation.z
                ]),
                quaternion_matrix([
                    t_cam_to_base.transform.rotation.x,
                    t_cam_to_base.transform.rotation.y,
                    t_cam_to_base.transform.rotation.z,
                    t_cam_to_base.transform.rotation.w
                ])
            )
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed for camera→base: {e}")
            return

        # Compute map→cam_base
        T_map_aruco = concatenate_matrices(
            translation_matrix([
                tf_map_to_aruco.transform.translation.x,
                tf_map_to_aruco.transform.translation.y,
                tf_map_to_aruco.transform.translation.z
            ]),
            quaternion_matrix([
                tf_map_to_aruco.transform.rotation.x,
                tf_map_to_aruco.transform.rotation.y,
                tf_map_to_aruco.transform.rotation.z,
                tf_map_to_aruco.transform.rotation.w
            ])
        )
        T_map_cam_base = T_map_aruco @ T_aruco_camera @ T_camera_cam_base

        if self.latest_odom_pose is None:
            self.get_logger().warn("No EKF pose available, skipping map→odom update.")
            return

        # Compute map→odom
        T_map_odom = T_map_cam_base @ np.linalg.inv(self.latest_odom_pose)
        trans_map_odom = translation_from_matrix(T_map_odom)
        quat_map_odom = quaternion_from_matrix(T_map_odom)

        t_map_odom = TransformStamped()
        t_map_odom.header.stamp = self.get_clock().now().to_msg()
        t_map_odom.header.frame_id = "map"
        t_map_odom.child_frame_id = "odom"
        t_map_odom.transform.translation.x, t_map_odom.transform.translation.y = trans_map_odom[:2]
        t_map_odom.transform.rotation.x, t_map_odom.transform.rotation.y, t_map_odom.transform.rotation.z, t_map_odom.transform.rotation.w = quat_map_odom

        self.last_map_to_odom = t_map_odom
        self.tf_broadcaster.sendTransform(t_map_odom)
        self.last_marker_time = self.get_clock().now()
        self.get_logger().info(f"Updated map→odom using marker {marker_id} (reproj err {reproj_err:.2f}px)")
        # only use first visible marker
