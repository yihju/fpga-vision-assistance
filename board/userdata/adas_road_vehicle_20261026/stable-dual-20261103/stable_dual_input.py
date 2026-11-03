"""Same stable FP16 graph; shared exact integer sums for NN and tracker."""
from pathlib import Path
import ctypes as C,hashlib,json
import numpy as np

class StableDualSession:
    def __init__(self,root,origin,cache=None,*,verify=True,base_class=None):
        root=Path(root);origin=Path(origin);library=root/'libvehicle_npu_dual.so'
        if verify:
            proof=json.loads((root/'dual-validation.json').read_text())
            if (any(proof.get(k) is not True for k in ('native_input_exact','outputs_bitwise_exact','tracking_rgb_exact','tracking_gray_exact')) or proof.get('cases',0)<10 or
                proof.get('library_sha256')!=hashlib.sha256(library.read_bytes()).hexdigest() or
                proof.get('source_sha256')!=hashlib.sha256((root/'vehicle_npu_dual.c').read_bytes()+(root/'vehicle_tracking_dual.h').read_bytes()).hexdigest()):raise ValueError('exact shared-input proof required')
        if base_class is None:
            from road_vehicle_npu import VehicleSession
        else:VehicleSession=base_class
        reference=VehicleSession(origin,native=True,fused=True);reference.close()
        lib=C.CDLL(str(library.resolve()));lib.vehicle_npu_open.argtypes=[C.c_char_p];lib.vehicle_npu_open.restype=C.c_void_p
        lib.vehicle_npu_close.argtypes=[C.c_void_p];lib.vehicle_npu_close.restype=None
        lib.vehicle_npu_run_bgr3_tracking.argtypes=[C.c_void_p,C.c_void_p,C.c_size_t,C.c_void_p,C.c_size_t,C.c_void_p,C.c_size_t];lib.vehicle_npu_run_bgr3_tracking.restype=C.c_int
        self.lib=lib;self.handle=lib.vehicle_npu_open(str(origin/'yolop-vehicle-only-fp16.rknn').encode());self.cache=cache;self.last_small=None;self.calls=0
        if not self.handle:raise ValueError('original stable shared-input model failed to initialize')
    def run_bgr3(self,image):
        if not self.handle or not isinstance(image,np.ndarray) or image.dtype!=np.uint8 or image.shape!=(1080,1920,3) or not image.flags.c_contiguous:raise ValueError('current contiguous1080p BGR required')
        raw=np.empty((1,25200,6),np.float32);small=np.empty((180,320,3),np.uint8)
        code=self.lib.vehicle_npu_run_bgr3_tracking(self.handle,image.ctypes.data,image.nbytes,raw.ctypes.data,raw.size,small.ctypes.data,small.nbytes)
        if code:raise RuntimeError('original stable shared-input inference failed: '+str(code))
        if not np.isfinite(raw).all():raise ValueError('nonfinite stable output')
        self.last_small=small;self.calls+=1
        if self.cache is not None:self.cache.put(image,small)
        return raw
    def close(self):
        if self.handle:self.lib.vehicle_npu_close(self.handle);self.handle=None
    def __enter__(self):return self
    def __exit__(self,*_):self.close()
