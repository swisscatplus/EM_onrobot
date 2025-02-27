import cv2
import os
from picamera2 import Picamera2
import time

# Directory where calibration images will be saved
save_dir = "calibration_images"
if not os.path.exists(save_dir):
    os.makedirs(save_dir)

# Initialize Picamera2
picam2 = Picamera2()
# Configure the camera for a preview (adjust resolution if needed)
preview_config = picam2.create_preview_configuration(main={"format": "XRGB8888", "size": (4608, 2592)})
picam2.configure(preview_config)
picam2.start()

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
