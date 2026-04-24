import cv2
import glob
from pathlib import Path
import sys

import numpy as np
import yaml


square_size = 8  # Change this value to match your printed chessboard squares (in mm)
initial_chessboard_size = [23, 17]  # Inner corners: columns, rows

camera_calibration_dir = Path(__file__).resolve().parent
config_path = Path(__file__).resolve().parent.parent / "config" / "calibration.yaml"
with config_path.open("r", encoding="utf-8") as config_file:
    calibration_config = yaml.safe_load(config_file) or {}

images = sorted(glob.glob(str(camera_calibration_dir / "calibration_images" / "*.jpg")))
print(f"Found {len(images)} images.")

if not images:
    print("No calibration images found.")
    sys.exit(1)

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
classic_flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
sb_flags = cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_EXHAUSTIVE + cv2.CALIB_CB_ACCURACY


def build_object_points(chessboard_size):
    objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)
    return objp * square_size


def detect_corners(gray, chessboard_size):
    if hasattr(cv2, "findChessboardCornersSB"):
        ret, corners = cv2.findChessboardCornersSB(gray, chessboard_size, flags=sb_flags)
        if ret:
            return True, corners

    ret, corners = cv2.findChessboardCorners(gray, chessboard_size, flags=classic_flags)
    if not ret:
        return False, None

    refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
    return True, refined


def gather_detections(chessboard_size):
    objpoints = []
    imgpoints = []
    valid_images = []
    last_gray_shape = None

    for fname in images:
        img = cv2.imread(fname)
        if img is None:
            continue

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        last_gray_shape = gray.shape[::-1]
        ret, corners = detect_corners(gray, chessboard_size)
        if not ret:
            continue

        objpoints.append(build_object_points(chessboard_size))
        imgpoints.append(corners)
        valid_images.append(fname)

    return objpoints, imgpoints, valid_images, last_gray_shape


def render_preview(image_path, chessboard_size, success_count):
    img = cv2.imread(image_path)
    if img is None:
        raise RuntimeError(f"Could not read {image_path}")

    display = img.copy()
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, corners = detect_corners(gray, chessboard_size)
    if ret:
        cv2.drawChessboardCorners(display, chessboard_size, corners, ret)
        status = "DETECTED"
        color = (0, 255, 0)
    else:
        status = "NOT DETECTED"
        color = (0, 0, 255)

    overlay_lines = [
        f"{Path(image_path).name}",
        f"inner corners: {chessboard_size[0]} x {chessboard_size[1]}",
        f"result: {status}",
        f"valid images: {success_count}/{len(images)}",
        "n/p: next/prev",
        "j/k: cols -/+",
        "u/i: rows -/+",
        "c: calibrate with current size",
        "q: quit",
    ]

    for idx, line in enumerate(overlay_lines):
        cv2.putText(
            display,
            line,
            (10, 30 + idx * 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            color if idx < 3 else (255, 255, 0),
            2,
            cv2.LINE_AA,
        )

    return display


current_index = 0
chessboard_size = initial_chessboard_size[:]
window_name = "Calibration Review"

print("Interactive calibration review")
print("Use n/p to browse images, j/k to change columns, u/i to change rows.")
print("Press c to calibrate once detections look correct, or q to quit.")

cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
cv2.resizeWindow(window_name, 1400, 900)

while True:
    board_size_tuple = (chessboard_size[0], chessboard_size[1])
    objpoints, imgpoints, valid_images, image_shape = gather_detections(board_size_tuple)
    preview = render_preview(images[current_index], board_size_tuple, len(valid_images))
    cv2.imshow(window_name, preview)

    key = cv2.waitKey(0) & 0xFF
    if key == ord("q"):
        cv2.destroyAllWindows()
        sys.exit(0)
    if key == ord("n"):
        current_index = (current_index + 1) % len(images)
        continue
    if key == ord("p"):
        current_index = (current_index - 1) % len(images)
        continue
    if key == ord("j"):
        chessboard_size[0] = max(2, chessboard_size[0] - 1)
        continue
    if key == ord("k"):
        chessboard_size[0] += 1
        continue
    if key == ord("u"):
        chessboard_size[1] = max(2, chessboard_size[1] - 1)
        continue
    if key == ord("i"):
        chessboard_size[1] += 1
        continue
    if key == ord("c"):
        if len(objpoints) < 3 or image_shape is None:
            print("Need at least 3 valid images before calibration.")
            continue

        ret, mtx, dist, _, _ = cv2.calibrateCamera(
            objpoints,
            imgpoints,
            image_shape,
            None,
            None,
        )
        print("Calibration was successful:", ret)
        print("Using inner corners:", board_size_tuple)
        print("Valid images:", len(valid_images))
        print("Camera matrix:\n", mtx)
        print("Distortion coefficients:\n", dist)

        calibration_config["camera_matrix"] = mtx.tolist()
        calibration_config["dist_coeff"] = dist.tolist()
        with config_path.open("w", encoding="utf-8") as config_file:
            yaml.safe_dump(calibration_config, config_file, sort_keys=False)

        print(f"Calibration parameters saved to {config_path}.")
        cv2.destroyAllWindows()
        sys.exit(0)
