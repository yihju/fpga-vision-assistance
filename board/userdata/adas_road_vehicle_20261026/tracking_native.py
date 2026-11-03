"""Explicit, validated 1080p→320 tracking resize; other geometries retain OpenCV."""
import ctypes as C,hashlib,json
from pathlib import Path
import cv2,numpy as np


def validation_passes(record,library_sha,source_sha):
    try:
        checks=record['checks']
        return (record['all_pixels_exact'] is True and record['library_sha256']==library_sha and record['source_sha256']==source_sha and
                len(checks)>=7 and all(row['pixels_exact'] is True and row['gray_exact'] is True and
                row['different_channels']==0 and row['maximum_difference']==0 for row in checks))
    except (KeyError,TypeError,ValueError):return False


class NativeTrackingGray:
    def __init__(self,root):
        root=Path(root);bridge=root/'libtracking_area6.so';source=root/'tracking_area6.c'
        record=json.loads((root/'tracking-gray-validation.json').read_text())
        library_sha=hashlib.sha256(bridge.read_bytes()).hexdigest();source_sha=hashlib.sha256(source.read_bytes()).hexdigest()
        if not validation_passes(record,library_sha,source_sha):raise ValueError('native tracking pixels have not passed equivalence')
        lib=C.CDLL(str(bridge));lib.tracking_area6.argtypes=[C.c_void_p,C.c_size_t,C.c_void_p,C.c_size_t];lib.tracking_area6.restype=C.c_int
        self.lib=lib;self.identity=dict(backend='native_neon_area6',library_sha256=library_sha,source_sha256=source_sha)
        self.native_calls=0;self.fallback_calls=0

    def __call__(self,frame,max_width):
        image=frame.bgr
        if max_width==320 and image.dtype==np.uint8 and image.shape==(1080,1920,3) and image.flags.c_contiguous:
            small=np.empty((180,320,3),np.uint8)
            if self.lib.tracking_area6(image.ctypes.data,image.nbytes,small.ctypes.data,small.nbytes):raise RuntimeError('native tracking resize failed')
            self.native_calls+=1;return cv2.cvtColor(small,cv2.COLOR_BGR2GRAY)
        self.fallback_calls+=1
        h,w=image.shape[:2];width=min(max_width,w);height=max(1,round(h*width/w))
        return cv2.cvtColor(cv2.resize(image,(width,height),interpolation=cv2.INTER_AREA),cv2.COLOR_BGR2GRAY)
