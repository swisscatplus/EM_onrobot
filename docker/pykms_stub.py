"""Headless Picamera2 import shim.

Picamera2 imports its DRM preview module at import time. The robot does not use
preview windows, but that eager import still expects the optional kms/pykms
Python bindings to exist. This shim lets Picamera2 import in headless capture
mode without building kmsxx. Creating a DRM preview will still fail clearly.
"""


class _Constants:
    def __getattr__(self, name):
        return name


class _Unavailable:
    def __init__(self, *args, **kwargs):
        raise RuntimeError(
            "DRM/KMS preview support is not installed in this headless image."
        )


PixelFormat = _Constants()
PlaneType = _Constants()

Card = _Unavailable
ResourceManager = _Unavailable
DumbFramebuffer = _Unavailable
DmabufFramebuffer = _Unavailable
AtomicReq = _Unavailable
