import sys
import time
from pathlib import Path

import cv2
import numpy as np
from picamera2 import Picamera2
import yaml

try:
    from libcamera import controls
except ImportError:  # pragma: no cover - depends on Pi camera stack
    controls = None

repo_src_dir = Path(__file__).resolve().parents[2]
if str(repo_src_dir) not in sys.path:
    sys.path.insert(0, str(repo_src_dir))

from em_robot.aruco_utils import build_marker_object_points, create_aruco_detector, detect_aruco_markers, estimate_marker_pose


config_path = Path(__file__).resolve().parent.parent / "config" / "calibration.yaml"
with config_path.open("r", encoding="utf-8") as config_file:
    calibration_config = yaml.safe_load(config_file) or {}

image_size = tuple(calibration_config.get("image_size", [1280, 720]))
lens_position = float(calibration_config.get("lens_position", 8.0))
marker_size = float(calibration_config.get("marker_size", 0.04))
max_reprojection_error = float(calibration_config.get("max_reprojection_error_px", 3.0))
camera_matrix = np.array(calibration_config.get("camera_matrix", []), dtype=np.float32)
dist_coeffs = np.array(calibration_config.get("dist_coeff", []), dtype=np.float32)

window_name = "ArUco Distance Validation"
save_dir = Path(__file__).resolve().parent / "validation_outputs"
save_dir.mkdir(exist_ok=True)

marker_object_points = build_marker_object_points(marker_size)
aruco_detector = create_aruco_detector()

picam2 = Picamera2()
preview_config = picam2.create_preview_configuration(main={"format": "XRGB8888", "size": image_size})
picam2.configure(preview_config)
picam2.start()
time.sleep(1.0)

camera_controls = getattr(picam2, "camera_controls", {})
if controls is not None and "AfMode" in camera_controls and "LensPosition" in camera_controls:
    picam2.set_controls(
        {
            "AfMode": controls.AfModeEnum.Manual,
            "LensPosition": lens_position,
        }
    )

print(f"Using calibration from {config_path}")
print("Place a marker at a known distance (for example 0.30 m) and compare with the reported z value.")
print("Press 's' to save a snapshot, 'q' to quit.")

cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
cv2.resizeWindow(window_name, 1280, 800)

snapshot_index = 0

while True:
    frame = picam2.capture_array()
    annotated = frame.copy()
    detections = detect_aruco_markers(frame, detector=aruco_detector)

    status_lines = [
        f"marker_size: {marker_size:.3f} m",
        "Compare reported z against a ruler-measured distance",
        "s: save snapshot  q: quit",
    ]

    if detections:
        detection = detections[0]
        camera_to_marker, reprojection_error = estimate_marker_pose(
            detection["corners"],
            marker_object_points,
            camera_matrix,
            dist_coeffs,
            max_reprojection_error,
        )
        if camera_to_marker is not None:
            z_distance = float(camera_to_marker[2, 3])
            x_offset = float(camera_to_marker[0, 3])
            y_offset = float(camera_to_marker[1, 3])
            status_lines.insert(0, f"id={detection['id']} z={z_distance:.3f} m")
            status_lines.insert(1, f"x={x_offset:.3f} m  y={y_offset:.3f} m  err={reprojection_error:.2f} px")

            corners = detection["corners"].reshape((-1, 1, 2)).astype(np.int32)
            cv2.polylines(annotated, [corners], True, (0, 255, 0), 2)
        else:
            status_lines.insert(0, "Marker detected but pose rejected")
    else:
        status_lines.insert(0, "No marker detected")

    for index, line in enumerate(status_lines):
        cv2.putText(
            annotated,
            line,
            (10, 30 + index * 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )

    cv2.imshow(window_name, annotated)
    key = cv2.waitKey(1) & 0xFF
    if key == ord("s"):
        snapshot_path = save_dir / f"aruco_validation_{snapshot_index:02d}.jpg"
        cv2.imwrite(str(snapshot_path), annotated)
        print(f"Saved {snapshot_path}")
        snapshot_index += 1
    elif key == ord("q"):
        break

picam2.stop()
cv2.destroyAllWindows()
