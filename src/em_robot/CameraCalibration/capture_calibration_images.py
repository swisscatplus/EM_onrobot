# Copyright 2026 SwissCAT+
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

import cv2
import time
from pathlib import Path

from picamera2 import Picamera2
import yaml

try:
    from libcamera import controls
except ImportError:  # pragma: no cover - depends on Pi camera stack
    controls = None

camera_calibration_dir = Path(__file__).resolve().parent
save_dir = camera_calibration_dir / "calibration_images"
save_dir.mkdir(exist_ok=True)
config_path = Path(__file__).resolve().parent.parent / "config" / "calibration.yaml"
with config_path.open("r", encoding="utf-8") as config_file:
    calibration_config = yaml.safe_load(config_file) or {}

# Calibration capture should match the real robot runtime setup as closely as possible.
# For EM Robot we expect markers mostly around 0.3-0.4 m, so keep focus fixed.
image_size = tuple(calibration_config.get("image_size", [1280, 720]))
lens_position = float(calibration_config.get("lens_position", 8.0))
focus_step = 0.1
focus_min = 0.0
focus_max = 32.0
manual_focus_supported = False
window_name = "Calibration Image Capture"
focus_trackbar_name = "LensPosition x10"


def apply_manual_focus(camera, position):
    camera.set_controls(
        {
            "AfMode": controls.AfModeEnum.Manual,
            "LensPosition": position,
        }
    )


def save_lens_position(position):
    calibration_config["lens_position"] = float(position)
    with config_path.open("w", encoding="utf-8") as config_file:
        yaml.safe_dump(calibration_config, config_file, sort_keys=False)
    print(f"Saved lens_position={position:.2f} to {config_path}")


def on_focus_trackbar(raw_value):
    global lens_position
    if not manual_focus_supported:
        return
    lens_position = max(focus_min, min(focus_max, raw_value / 10.0))
    apply_manual_focus(picam2, lens_position)

# Initialize Picamera2
picam2 = Picamera2()
# Configure the camera for a preview (adjust resolution if needed)
preview_config = picam2.create_preview_configuration(
    main={"format": "XRGB8888", "size": image_size}
)
picam2.configure(preview_config)
picam2.start()
time.sleep(1.0)

camera_controls = getattr(picam2, "camera_controls", {})
if controls is not None and "AfMode" in camera_controls and "LensPosition" in camera_controls:
    manual_focus_supported = True
    apply_manual_focus(picam2, lens_position)
    print(
        f"Manual focus enabled with LensPosition={lens_position}. "
        "Use the same focus setting during calibration and runtime."
    )
else:
    print("Manual focus controls are not available on this camera; focus is likely fixed.")

print(f"Using calibration settings from {config_path}")
print(f"Preview size: {image_size}")
print(f"Saving images to {save_dir}")

cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
cv2.resizeWindow(window_name, 1280, 800)
cv2.moveWindow(window_name, 50, 50)
if manual_focus_supported:
    cv2.createTrackbar(
        focus_trackbar_name,
        window_name,
        int(round(lens_position * 10.0)),
        int(focus_max * 10.0),
        on_focus_trackbar,
    )

img_counter = 0
print("Press 's' to capture an image. Capture at least 8-10 images from different angles and distances.")
if manual_focus_supported:
    print("Use the OpenCV trackbar to adjust focus. Press 'w' to write the current focus to calibration.yaml.")
print("Press 'q' to quit.")

while True:
    # Capture the current frame as a NumPy array
    frame = picam2.capture_array()

    status_lines = [
        f"lens_position: {lens_position:.2f}",
        "s: save image  q: quit",
    ]
    if manual_focus_supported:
        status_lines.append("Use trackbar above  w: save focus")

    for index, line in enumerate(status_lines):
        cv2.putText(
            frame,
            line,
            (10, 30 + index * 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )

    # Display the live feed
    cv2.imshow(window_name, frame)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('s'):
        # Save the current frame as a calibration image.
        img_name = save_dir / f"calib_{img_counter:02d}.jpg"
        cv2.imwrite(str(img_name), frame)
        print(f"Saved {img_name}")
        img_counter += 1
        # Optional: pause a moment between captures
        time.sleep(0.5)
    elif manual_focus_supported and key == ord('w'):
        save_lens_position(lens_position)
    elif key == ord('q'):
        break

picam2.stop()
cv2.destroyAllWindows()
