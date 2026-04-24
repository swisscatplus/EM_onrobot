import cv2
import numpy as np
import glob
from pathlib import Path
import sys
import yaml

# --- Calibration Pattern Settings ---
# Define the number of inner corners per chessboard row and column.
# (For example, a 7x6 chessboard has 7 inner corners horizontally and 6 vertically.)
chessboard_size = (10, 7)
# Define the size of a chessboard square in millimeters.
square_size = 22  # Change this value to match your printed chessboard squares (in mm)

# Prepare object points based on the real-world coordinates of the chessboard corners.
objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)
objp = objp * square_size

# Arrays to store 3D points in real-world space and 2D points in image plane.
objpoints = []
imgpoints = []

camera_calibration_dir = Path(__file__).resolve().parent
config_path = Path(__file__).resolve().parent.parent / "config" / "calibration.yaml"
with config_path.open("r", encoding="utf-8") as config_file:
    calibration_config = yaml.safe_load(config_file) or {}

# --- Load Calibration Images ---
images = sorted(glob.glob(str(camera_calibration_dir / "calibration_images" / "*.jpg")))
print(f"Found {len(images)} images.")

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
classic_flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
sb_flags = cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_EXHAUSTIVE + cv2.CALIB_CB_ACCURACY

image_shape = None

for fname in images:
    img = cv2.imread(fname)
    if img is None:
        print(f"Could not read {fname}")
        continue

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    image_shape = gray.shape[::-1]

    # Prefer the more robust SB detector when available, then fall back to the classic detector.
    if hasattr(cv2, "findChessboardCornersSB"):
        ret, corners = cv2.findChessboardCornersSB(gray, chessboard_size, flags=sb_flags)
        refined_corners = corners if ret else None
    else:
        ret, corners = cv2.findChessboardCorners(gray, chessboard_size, flags=classic_flags)
        refined_corners = None

    if ret:
        objpoints.append(objp)
        if refined_corners is None:
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        else:
            corners2 = refined_corners
        imgpoints.append(corners2)

        cv2.drawChessboardCorners(img, chessboard_size, corners2, ret)
        cv2.imshow("Detected Corners", img)
        cv2.waitKey(500)
    else:
        print(
            f"Chessboard corners not found in {fname}. "
            f"Check that the printed board really has {chessboard_size[0]}x{chessboard_size[1]} inner corners."
        )

cv2.destroyAllWindows()

if len(objpoints) < 1 or image_shape is None:
    print("Not enough valid images for calibration. Please capture more images with a visible chessboard.")
    sys.exit(1)

# --- Calibrate the Camera ---
ret, mtx, dist, _, _ = cv2.calibrateCamera(objpoints, imgpoints, image_shape, None, None)
print("Calibration was successful:", ret)
print("Camera matrix:\n", mtx)
print("Distortion coefficients:\n", dist)

# --- Save Calibration Parameters to a YAML File ---
calibration_config["camera_matrix"] = mtx.tolist()
calibration_config["dist_coeff"] = dist.tolist()

with config_path.open("w", encoding="utf-8") as config_file:
    yaml.safe_dump(calibration_config, config_file, sort_keys=False)

print(f"Calibration parameters saved to {config_path}.")
