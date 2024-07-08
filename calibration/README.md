# Calibration

This folder was used to calibrate the camera, and can be extended to other cameras. Simply erase the .png images in the corresponding folder, take >12 images of a calibration grid using take_images.py (just wait and orient the calibration grid in another direction every time a message is printed). Then use calibration.py to compute the .pkl data necessary for undistorting the camera image. Don't forget to adapt the config at the beginning of the .py.
This folder is only present on the calibration git branch, for memory purposes.

Since we cannot run OpenCV on the Raspberry Pi 5 with the picamera, I would suggest running the docker image with the terminal activated (for this, you need to build the image with CMD ["bash"] and comment the if statement from the ros_entrypoint.sh). The instructions are already there, just uncomment them.
