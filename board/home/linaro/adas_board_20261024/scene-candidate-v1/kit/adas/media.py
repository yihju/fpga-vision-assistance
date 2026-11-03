"""Local image/video reader; decoder PTS is not an external optical clock."""
from pathlib import Path
import math
import time

import cv2
import numpy as np

from .contracts import Frame, FrameKey

IMAGE_SUFFIXES = {'.png', '.jpg', '.jpeg', '.bmp', '.webp', '.tif', '.tiff'}


def fit_image(image, size):
    if (not isinstance(image, np.ndarray) or image.dtype != np.uint8 or image.ndim != 3 or
            image.shape[2] != 3 or min(image.shape[:2]) < 1 or len(size) != 2 or
            any(type(v) is not int or v < 1 for v in size)):
        raise ValueError('nonempty BGR and positive integer canvas required')
    width, height = size
    h, w = image.shape[:2]
    ratio = min(width/w, height/h)
    rw, rh = max(1, int(w*ratio)), max(1, int(h*ratio))
    left, top = (width-rw)//2, (height-rh)//2
    canvas = np.zeros((height, width, 3), np.uint8)
    canvas[top:top+rh, left:left+rw] = cv2.resize(image, (rw, rh))
    return canvas, dict(left=left, top=top, width=rw, height=rh, scale_x=rw/w, scale_y=rh/h)


class MediaReader:
    def __init__(self, path, *, epoch=0, camera_id=0):
        FrameKey(epoch, 0, camera_id)
        self.path = Path(path)
        if not self.path.is_file():
            raise ValueError('input file does not exist')
        self.epoch, self.camera_id = epoch, camera_id
        self.frame_id = 0
        self.status = 'ready'
        self.capture = None
        self.still = None
        self._last_pts = None
        if self.path.suffix.lower() in IMAGE_SUFFIXES:
            self.still = cv2.imdecode(np.frombuffer(self.path.read_bytes(), np.uint8), cv2.IMREAD_COLOR)
            if self.still is None:
                raise ValueError('image decode failed')
            self.fps = None
        else:
            self.capture = cv2.VideoCapture(str(self.path))
            if not self.capture.isOpened():
                self.capture.release()
                raise ValueError('video open/decode failed')
            fps = self.capture.get(cv2.CAP_PROP_FPS)
            self.fps = fps if math.isfinite(fps) and fps > 0 else None

    def read(self):
        if self.status == 'closed':
            raise ValueError('reader is closed')
        if self.status == 'ended':
            return None
        if self.still is not None:
            if self.frame_id:
                self.status = 'ended'
                return None
            image, pts, kind = self.still.copy(), None, 'not_applicable_image'
        else:
            success, image = self.capture.read()
            if not success:
                position = self.capture.get(cv2.CAP_PROP_POS_FRAMES)
                expected = self.capture.get(cv2.CAP_PROP_FRAME_COUNT)
                self.status = 'ended'
                if self.frame_id == 0 or (expected > 0 and position < expected-1):
                    self.status = 'error'
                    raise ValueError('decoder stopped before reported end; file may be damaged')
                return None
            msec = self.capture.get(cv2.CAP_PROP_POS_MSEC)
            pts = round(msec*1000) if math.isfinite(msec) and msec >= 0 else None
            if pts is not None and self._last_pts is not None and pts <= self._last_pts:
                pts = None
            if pts is not None:
                self._last_pts = pts
            kind = 'decoder_reported' if pts is not None else 'unavailable'
        result = Frame(FrameKey(self.epoch, self.frame_id, self.camera_id), pts, time.perf_counter_ns(), image, kind)
        self.frame_id += 1
        return result

    def seek(self, source_frame):
        if self.status == 'closed' or type(source_frame) is not int or source_frame < 0:
            raise ValueError('invalid seek')
        if self.capture is None or not self.capture.set(cv2.CAP_PROP_POS_FRAMES, source_frame):
            raise ValueError('source cannot seek')
        self.epoch += 1
        self.frame_id = 0
        self._last_pts = None
        self.status = 'ready'

    def close(self):
        if self.capture is not None:
            self.capture.release()
        self.status = 'closed'
        self.still = None

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()


def read_frames(path, epoch=0):
    with MediaReader(path, epoch=epoch) as reader:
        while (frame := reader.read()) is not None:
            yield frame
