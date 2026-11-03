"""Bounded real pixel ROI exchange with FPGA RPS1; no camera/DMA/LED writes."""
import ctypes
from dataclasses import asdict
from pathlib import Path
import threading

import numpy as np

FIELDS=('sequence','count','gray_sum','red','green','yellow','binary_ones','gray_min','gray_max')


class Result(ctypes.Structure):
    _fields_=[(name,ctypes.c_uint32) for name in FIELDS]


class NativeBackend:
    def __init__(self, device='/sys/bus/pci/devices/0002:21:00.0', library=None):
        library=Path(library) if library else Path(__file__).with_name('libadas_roi.so')
        self.lib=ctypes.CDLL(str(library),use_errno=True)
        self.lib.adas_roi_open.argtypes=[ctypes.c_char_p]
        self.lib.adas_roi_open.restype=ctypes.c_void_p
        self.lib.adas_roi_close.argtypes=[ctypes.c_void_p]
        self.lib.adas_roi_key.argtypes=[ctypes.c_void_p]
        self.lib.adas_roi_key.restype=ctypes.c_int
        self.lib.adas_roi_compute.argtypes=[ctypes.c_void_p,ctypes.c_uint32,ctypes.c_void_p,
            ctypes.c_uint32,ctypes.c_uint32,ctypes.POINTER(Result)]
        self.lib.adas_roi_compute.restype=ctypes.c_int
        self.handle=self.lib.adas_roi_open(str(device).encode())
        if not self.handle:
            raise OSError(ctypes.get_errno(),'FPGA ROI identity, mapping or ownership failed')

    def compute(self, sequence, words, timeout_ms):
        if not self.handle: raise RuntimeError('closed FPGA ROI bridge')
        result=Result()
        if self.lib.adas_roi_compute(self.handle,sequence,words.ctypes.data,len(words),timeout_ms,ctypes.byref(result)):
            raise OSError(ctypes.get_errno(),'FPGA ROI exchange failed and aborted')
        return {name:int(getattr(result,name)) for name in FIELDS}

    def key_pressed(self):
        if not self.handle: raise RuntimeError('closed FPGA ROI bridge')
        result=self.lib.adas_roi_key(self.handle)
        if result<0: raise OSError(ctypes.get_errno(),'FPGA key identity failed')
        return bool(result)

    def close(self):
        if self.handle: self.lib.adas_roi_close(self.handle);self.handle=None


def reference_stats(image):
    if (not isinstance(image,np.ndarray) or image.dtype!=np.uint8 or image.ndim!=3 or
            image.shape[2]!=3 or not 1<=image.shape[0]*image.shape[1]<=4096):
        raise ValueError('one nonempty UINT8 BGR ROI of at most4096 pixels required')
    b,g,r=image.astype(np.int32).transpose(2,0,1)
    gray=(77*r+150*g+29*b)//256
    return dict(count=int(gray.size),gray_sum=int(gray.sum()),
        red=int(((r>=85)&(r>=g+32)&(r>=b+32)).sum()),
        green=int(((g>=85)&(g>=r+20)&(g>=b+10)).sum()),
        yellow=int(((r>=85)&(g>=85)&(r>=b+32)&(g>=b+32)).sum()),
        binary_ones=int((gray>=128).sum()),gray_min=int(gray.min()),gray_max=int(gray.max()))


class FpgaRoi:
    def __init__(self, *, backend=None, timeout_ms=100):
        if type(timeout_ms) is not int or not 1<=timeout_ms<=1000:
            raise ValueError('bounded integer timeout required')
        self.backend=backend if backend is not None else NativeBackend()
        self.timeout_ms=timeout_ms;self.sequence=0;self.lock=threading.Lock();self.closed=False

    def analyze(self, frame, bbox):
        if (not isinstance(bbox,(tuple,list)) or len(bbox)!=4 or any(type(v) is not int for v in bbox)):
            raise ValueError('exact integer source ROI bounds required')
        image=frame.bgr
        if (not isinstance(image,np.ndarray) or image.dtype!=np.uint8 or image.ndim!=3 or image.shape[2]!=3):
            raise ValueError('UINT8 BGR source required')
        x0,y0,x1,y1=bbox
        if not (0<=x0<x1<=image.shape[1] and 0<=y0<y1<=image.shape[0] and (x1-x0)*(y1-y0)<=4096):
            raise ValueError('ROI must fit actual source and at most4096 pixels')
        pixels=image[y0:y1,x0:x1].astype(np.uint32)
        words=np.ascontiguousarray((pixels[:,:,2]<<16)|(pixels[:,:,1]<<8)|pixels[:,:,0]).reshape(-1)
        with self.lock:
            if self.closed: raise RuntimeError('closed FPGA ROI bridge')
            if self.sequence==0xffffffff: raise RuntimeError('sequence exhausted; reopen bridge')
            self.sequence+=1
            result=self.backend.compute(self.sequence,words,self.timeout_ms)
            if (result.get('sequence')!=self.sequence or result.get('count')!=len(words) or
                    set(result)!=set(FIELDS) or any(type(v) is not int or v<0 for v in result.values()) or
                    any(result[k]>len(words) for k in ('red','green','yellow','binary_ones')) or
                    result['gray_sum']>255*len(words) or not 0<=result['gray_min']<=result['gray_max']<=255):
                raise RuntimeError('invalid or foreign FPGA completion sequence/count/statistics')
        return dict(result,key=asdict(frame.key),bbox_xyxy=list(bbox),source='FPGA_RPS1_actual_pixels')

    def close(self):
        with self.lock:
            if not self.closed: self.backend.close();self.closed=True
