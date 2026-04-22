#!/usr/bin/env python3
import cv2 as cv


class PicameraSource:
    def __init__(self, image_size, lens_position):
        from libcamera import controls
        from picamera2 import Picamera2

        self._camera = Picamera2()
        preview_config = self._camera.create_preview_configuration(
            main={"format": "XRGB8888", "size": image_size}
        )
        self._camera.configure(preview_config)
        self._camera.start()
        self._camera.set_controls(
            {"AfMode": controls.AfModeEnum.Manual, "LensPosition": lens_position}
        )

    def capture(self):
        return self._camera.capture_array()

    def close(self):
        self._camera.stop()


class OpenCVCameraSource:
    def __init__(self, source, image_size):
        source_value = int(source) if str(source).isdigit() else source
        self._capture = cv.VideoCapture(source_value)
        if image_size:
            self._capture.set(cv.CAP_PROP_FRAME_WIDTH, image_size[0])
            self._capture.set(cv.CAP_PROP_FRAME_HEIGHT, image_size[1])

        if not self._capture.isOpened():
            raise RuntimeError(f"Failed to open OpenCV camera source: {source}")

    def capture(self):
        ok, frame = self._capture.read()
        if not ok:
            raise RuntimeError("Failed to read frame from OpenCV camera source")
        return frame

    def close(self):
        self._capture.release()


def create_camera_source(camera_backend, source, image_size, lens_position):
    if camera_backend == "picamera2":
        return PicameraSource(image_size=image_size, lens_position=lens_position)
    if camera_backend == "opencv":
        return OpenCVCameraSource(source=source, image_size=image_size)
    raise ValueError(f"Unsupported camera backend: {camera_backend}")
