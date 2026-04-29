#!/usr/bin/env python3
from dataclasses import dataclass
import threading
import time

import cv2 as cv


def normalize_to_bgr8(frame):
    if frame is None:
        return frame
    if len(frame.shape) == 2:
        return cv.cvtColor(frame, cv.COLOR_GRAY2BGR)
    if len(frame.shape) == 3 and frame.shape[2] == 4:
        return cv.cvtColor(frame, cv.COLOR_BGRA2BGR)
    return frame


@dataclass(frozen=True)
class CapturedFrame:
    frame: object
    timestamp_ns: int


class PicameraSource:
    def __init__(self, image_size, lens_position):
        from libcamera import controls
        from picamera2 import Picamera2

        self._camera = Picamera2()
        preview_config = self._camera.create_preview_configuration(
            main={"format": "XRGB8888", "size": image_size},
            buffer_count=1,
            queue=False,
        )
        self._camera.configure(preview_config)
        self._camera.start()
        self._camera.set_controls(
            {"AfMode": controls.AfModeEnum.Manual, "LensPosition": lens_position}
        )

    def capture(self):
        return self.capture_with_timestamp().frame

    def capture_with_timestamp(self):
        frame = normalize_to_bgr8(self._camera.capture_array())
        return CapturedFrame(frame=frame, timestamp_ns=time.time_ns())

    def close(self):
        self._camera.stop()


class OpenCVCameraSource:
    def __init__(self, source, image_size, loop=False):
        source_value = int(source) if str(source).isdigit() else source
        self._capture = cv.VideoCapture(source_value)
        self._loop = loop
        self._source = source
        self._is_live_source = not loop and (
            isinstance(source_value, int) or str(source).startswith("/dev/video")
        )
        self._latest_frame = None
        self._latest_frame_timestamp_ns = None
        self._latest_error = None
        self._frame_lock = threading.Lock()
        self._stop_reader = threading.Event()
        self._reader_thread = None

        if self._is_live_source:
            # Keep only the freshest frame for live cameras to avoid multi-second lag.
            self._capture.set(cv.CAP_PROP_BUFFERSIZE, 1)

        if image_size:
            self._capture.set(cv.CAP_PROP_FRAME_WIDTH, image_size[0])
            self._capture.set(cv.CAP_PROP_FRAME_HEIGHT, image_size[1])

        if not self._capture.isOpened():
            raise RuntimeError(f"Failed to open OpenCV camera source: {source}")

        if self._is_live_source:
            self._reader_thread = threading.Thread(
                target=self._reader_loop,
                name="opencv-camera-reader",
                daemon=True,
            )
            self._reader_thread.start()

    def _reader_loop(self):
        while not self._stop_reader.is_set():
            ok, frame = self._capture.read()
            if not ok:
                self._latest_error = "Failed to read frame from OpenCV camera source"
                time.sleep(0.01)
                continue

            with self._frame_lock:
                self._latest_frame = frame
                self._latest_frame_timestamp_ns = time.time_ns()
                self._latest_error = None

    def capture(self):
        return self.capture_with_timestamp().frame

    def capture_with_timestamp(self):
        if self._is_live_source:
            deadline = time.monotonic() + 2.0
            while not self._stop_reader.is_set():
                with self._frame_lock:
                    if self._latest_frame is not None:
                        return CapturedFrame(
                            frame=self._latest_frame.copy(),
                            timestamp_ns=int(self._latest_frame_timestamp_ns or time.time_ns()),
                        )
                    latest_error = self._latest_error

                if time.monotonic() >= deadline:
                    raise RuntimeError(
                        latest_error or f"Timed out waiting for camera source: {self._source}"
                    )
                time.sleep(0.01)

        ok, frame = self._capture.read()
        if not ok and self._loop:
            self._capture.set(cv.CAP_PROP_POS_FRAMES, 0)
            ok, frame = self._capture.read()
        if not ok:
            raise RuntimeError("Failed to read frame from OpenCV camera source")
        return CapturedFrame(frame=frame, timestamp_ns=time.time_ns())

    def close(self):
        self._stop_reader.set()
        if self._reader_thread is not None:
            self._reader_thread.join(timeout=1.0)
        self._capture.release()


def create_camera_source(camera_backend, source, image_size, lens_position, loop=False):
    if camera_backend == "picamera2":
        return PicameraSource(image_size=image_size, lens_position=lens_position)
    if camera_backend == "opencv":
        return OpenCVCameraSource(source=source, image_size=image_size, loop=loop)
    raise ValueError(f"Unsupported camera backend: {camera_backend}")
