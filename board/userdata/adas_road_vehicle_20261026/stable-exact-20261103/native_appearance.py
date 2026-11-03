"""Explicit same-HSV histogram accelerator, gated by exhaustive HSV proof."""
from pathlib import Path
import ctypes as C,hashlib,json,math
import numpy as np

def crop_bounds(shape,box):
    h,w=shape[:2];a,b,c,d=box['xyxy']
    if not all(math.isfinite(v) for v in (a,b,c,d)):raise ValueError('finite crop required')
    x,y,ex,ey=max(0,int(math.floor(a))),max(0,int(math.floor(b))),min(w,int(math.ceil(c))),min(h,int(math.ceil(d)))
    if not 0<=x<ex<=w or not 0<=y<ey<=h:raise ValueError('nonempty native crop required')
    return x,y,ex,ey

class NativeAppearance:
    def __init__(self,root,*,verify=True):
        root=Path(root);library=root/'libappearance_hist.so'
        if verify:
            record=json.loads((root/'appearance-validation.json').read_text())
            if (record.get('all_hsv_bins_exact') is not True or record.get('hsv_colors')!=11796480 or
                record.get('histograms_exact') is not True or record.get('tested_patches',0)<6 or
                record.get('library_sha256')!=hashlib.sha256(library.read_bytes()).hexdigest() or
                record.get('source_sha256')!=hashlib.sha256((root/'appearance_hist.c').read_bytes()).hexdigest()):
                raise ValueError('exact appearance validation required')
        lib=C.CDLL(str(library.resolve()))
        for name in ('appearance_bins','appearance_hist'):
            function=getattr(lib,name);function.argtypes=[C.c_void_p,C.c_size_t,C.c_size_t,C.c_size_t,C.c_void_p,C.c_size_t];function.restype=C.c_int
        self.lib=lib;self.calls=0
    @staticmethod
    def pixels(image):
        if not isinstance(image,np.ndarray) or image.dtype!=np.uint8 or image.ndim!=3 or image.shape[2]!=3 or min(image.shape[:2])<1:raise ValueError('native BGR required')
        if image.strides[2]!=1 or image.strides[1]!=3 or image.strides[0]<image.shape[1]*3:image=np.ascontiguousarray(image)
        return image
    def bins(self,image):
        image=self.pixels(image);h,w=image.shape[:2];out=np.empty((h,w),np.uint8)
        if self.lib.appearance_bins(image.ctypes.data,w,h,image.strides[0],out.ctypes.data,out.size):raise RuntimeError('native appearance bins failed')
        return out
    def __call__(self,image,box):
        import cv2
        x,y,ex,ey=crop_bounds(image.shape,box);patch=self.pixels(image[y:ey,x:ex]);hsv=cv2.cvtColor(patch,cv2.COLOR_BGR2HSV);h,w=hsv.shape[:2];counts=np.empty(128,np.uint32)
        if self.lib.appearance_hist(hsv.ctypes.data,w,h,hsv.strides[0],counts.ctypes.data,counts.size):raise RuntimeError('native appearance histogram failed')
        histogram=counts.astype(np.float32);self.calls+=1
        return histogram/max(float(histogram.sum()),1.)
