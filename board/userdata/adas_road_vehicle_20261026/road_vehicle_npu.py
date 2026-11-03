"""Synchronous FP16 vehicle detection on the RK3568 NPU."""
from pathlib import Path
import ctypes as C,hashlib,json
import numpy as np
try:from adas.road_vehicle_probe import prepare_rgb,decode_vehicles
except ModuleNotFoundError:from road_vehicle_probe import prepare_rgb,decode_vehicles

SHAPES=((1,25200,6),(1,2,640,640),(1,2,640,640))








class Session:
    """Context management shared by the current vehicle session."""
    def __enter__(self):return self
    def __exit__(self,*_):self.close()


class VehicleSession(Session):
    def __init__(self,root,*,native=False,fused=False):
        root=Path(root);record=json.loads((root/'vehicle-fp16-export.json').read_text());model=root/'yolop-vehicle-only-fp16.rknn'
        if (record['source_sha256']!='9e5954117a9357826332d92fb8da9a34fc089c7ea286e57ec72329d43475d9c4' or record['int8'] or record['pruning'] or
                record['outputs']!=['det_out'] or not record['segmentation_removed']):raise ValueError('fixed vehicle-only FP16 source required')
        if hashlib.sha256(model.read_bytes()).hexdigest()!=record['rknn_sha256']:raise ValueError('vehicle RKNN changed')
        if type(native) is not bool or type(fused) is not bool or fused and not native:raise ValueError('explicit native/fused flags required')
        lib=C.CDLL(str(root/('libvehicle_npu_fused.so' if fused else 'libvehicle_npu_native.so' if native else 'libvehicle_npu.so')))
        lib.vehicle_npu_open.argtypes=[C.c_char_p];lib.vehicle_npu_open.restype=C.c_void_p
        lib.vehicle_npu_run.argtypes=[C.c_void_p,C.c_void_p,C.c_size_t,C.c_void_p,C.c_size_t];lib.vehicle_npu_run.restype=C.c_int
        lib.vehicle_npu_close.argtypes=[C.c_void_p];lib.vehicle_npu_close.restype=None
        self.lib=lib;self.handle=lib.vehicle_npu_open(str(model).encode())
        if not self.handle:raise ValueError('vehicle-only model shape/init failed')

    def run_bgr3(self,image):
        if not self.handle or not isinstance(image,np.ndarray) or image.dtype!=np.uint8 or image.shape!=(1080,1920,3) or not image.flags.c_contiguous:
            raise ValueError('current contiguous native BGR 1080p required')
        function=self.lib.vehicle_npu_run_bgr3
        function.argtypes=[C.c_void_p,C.c_void_p,C.c_size_t,C.c_void_p,C.c_size_t];function.restype=C.c_int
        output=np.empty(SHAPES[0],np.float32)
        code=function(self.handle,image.ctypes.data,image.nbytes,output.ctypes.data,output.size)
        if code:raise RuntimeError('fused vehicle NPU failed: '+str(code))
        if not np.isfinite(output).all():raise ValueError('nonfinite fused output')
        return output

    def run(self,rgb):
        if not self.handle:raise ValueError('closed vehicle session')
        if not isinstance(rgb,np.ndarray) or rgb.dtype!=np.uint8 or rgb.shape!=(640,640,3) or not rgb.flags.c_contiguous:raise ValueError('contiguous RGB required')
        output=np.empty(SHAPES[0],np.float32)
        code=self.lib.vehicle_npu_run(self.handle,rgb.ctypes.data,rgb.nbytes,output.ctypes.data,output.size)
        if code:raise RuntimeError('vehicle NPU failed: '+str(code))
        if not np.isfinite(output).all():raise ValueError('nonfinite vehicle NPU output')
        return output

    def close(self):
        if self.handle:self.lib.vehicle_npu_close(self.handle);self.handle=None
