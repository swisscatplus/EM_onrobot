from picamera2 import Picamera2
from libcamera import controls
import cv2
import time

picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (640, 480)}))
picam2.start()
picam2.set_controls({"AfMode": controls.AfModeEnum.Manual, "LensPosition": 2.32}) 

num = 0

while True:
    frame = picam2.capture_array()
    time.sleep(5)
    print('Firing in 1')
    time.sleep(1)

    cv2.imwrite('images/img' + str(num) + '.png', frame)
    print(f"Image saved: images/img{num}.png")
    print("image saved!")
    num += 1
