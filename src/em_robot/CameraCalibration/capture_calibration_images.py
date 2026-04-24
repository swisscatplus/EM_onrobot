import cv2
import os
import time
from picamera2 import Picamera2

try:
    from libcamera import controls
except ImportError:  # pragma: no cover - depends on Pi camera stack
    controls = None

# Directory where calibration images will be saved
save_dir = "calibration_images"
if not os.path.exists(save_dir):
    os.makedirs(save_dir)

# Calibration capture should match the real robot runtime setup as closely as possible.
# For EM Robot we expect markers mostly around 0.3-0.4 m, so keep focus fixed.
image_size = (1280, 720)
lens_position = 8.0

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
    picam2.set_controls(
        {
            "AfMode": controls.AfModeEnum.Manual,
            "LensPosition": lens_position,
        }
    )
    print(
        f"Manual focus enabled with LensPosition={lens_position}. "
        "Use the same focus setting during calibration and runtime."
    )
else:
    print("Manual focus controls are not available on this camera; focus is likely fixed.")

img_counter = 0
print("Press 's' to capture an image. Capture at least 8-10 images from different angles and distances.")
print("Press 'q' to quit.")

while True:
    # Capture the current frame as a NumPy array
    frame = picam2.capture_array()

    # Display the live feed
    cv2.imshow("Calibration Image Capture", frame)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('s'):
        # Save the current frame as a calibration image.
        img_name = os.path.join(save_dir, f"calib_{img_counter:02d}.jpg")
        cv2.imwrite(img_name, frame)
        print(f"Saved {img_name}")
        img_counter += 1
        # Optional: pause a moment between captures
        time.sleep(0.5)
    elif key == ord('q'):
        break

picam2.stop()
cv2.destroyAllWindows()
