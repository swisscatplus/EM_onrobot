import cv2
import numpy as np
import glob
import yaml

# --- Calibration Pattern Settings ---
# Define the number of inner corners per chessboard row and column.
# (For example, a 7x6 chessboard has 7 inner corners horizontally and 6 vertically.)
chessboard_size = (23, 17)
# Define the size of a chessboard square in millimeters.
square_size = 19  # Change this value to match your printed chessboard squares (in mm)

# Prepare object points based on the real-world coordinates of the chessboard corners.
# For example, (0,0,0), (1,0,0), ... are scaled by square_size.
objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)
objp = objp * square_size  # scale object points to the real square size

# Arrays to store 3D points in real-world space and 2D points in image plane.
objpoints = []  # 3D real-world points
imgpoints = []  # 2D image points

# --- Load Calibration Images ---
# Adjust the pattern if you saved your images with a different extension or folder.
images = glob.glob('calibration_images/*.jpg')
print(f"Found {len(images)} images.")

# Termination criteria for refining corner detections (maximum iterations and desired accuracy).
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Process each image
for fname in images:
    img = cv2.imread(fname)
    if img is None:
        print(f"Could not read {fname}")
        continue
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Find chessboard corners in the image
    ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)

    if ret:
        # If found, add object points and refined image points.
        objpoints.append(objp)
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)

        # Optionally display the detected corners for verification.
        cv2.drawChessboardCorners(img, chessboard_size, corners2, ret)
        cv2.imshow("Detected Corners", img)
        cv2.waitKey(500)
    else:
        print(f"Chessboard corners not found in {fname}.")

cv2.destroyAllWindows()

if len(objpoints) < 1:
    print("Not enough valid images for calibration. Please capture more images with a visible chessboard.")
    exit()

# --- Calibrate the Camera ---
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
print("Calibration was successful:", ret)
print("Camera matrix:\n", mtx)
print("Distortion coefficients:\n", dist)

# --- Save Calibration Parameters to a YAML File ---
calibration_data = {
    'camera_matrix': mtx.tolist(),
    'dist_coeff': dist.tolist()
}

with open("calibration.yaml", "w") as f:
    yaml.dump(calibration_data, f)

print("Calibration parameters saved to calibration.yaml.")
