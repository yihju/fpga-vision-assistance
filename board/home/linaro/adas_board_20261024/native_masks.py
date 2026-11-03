"""Validated native mask builder; same source pixels and exact HSV thresholds."""
import ctypes,hashlib,json
from pathlib import Path
import numpy as np

KEYS=('red','green','yellow','weak_red','dark')


class NativeMasks:
    def __init__(self,*,verify=True):
        root=Path(__file__).resolve().parent;library=root/'libadas_masks.so'
        if verify:
            record=json.loads((root/'native-masks-validation.json').read_text())
            rows=record['rows']
            if (record.get('status')!='byte_exact_masks' or
                record['library_sha256']!=hashlib.sha256(library.read_bytes()).hexdigest() or
                len(rows)<3 or {r['name'] for r in rows}!={'HSV_boundary_crossproduct','random_all_byte_values','real_native_frame35'} or
                any(r['input_unchanged'] is not True or set(r['equal'])!=set(KEYS) or
                    any(v is not True for v in r['equal'].values()) for r in rows)):
                raise ValueError('native mask byte-equivalence gate failed')
        self.lib=ctypes.CDLL(str(library))
        self.lib.adas_lamp_masks.argtypes=[ctypes.c_void_p,ctypes.c_size_t,ctypes.c_size_t,ctypes.c_void_p,ctypes.c_size_t]
        self.lib.adas_lamp_masks.restype=ctypes.c_int

    def __call__(self,hsv):
        if (not isinstance(hsv,np.ndarray) or hsv.dtype!=np.uint8 or hsv.ndim!=3 or hsv.shape[2]!=3 or
            min(hsv.shape[:2])<1 or max(hsv.shape[:2])>8192 or hsv.shape[0]*hsv.shape[1]>16777216):
            raise ValueError('bounded nonempty UINT8 HSV input required')
        pixels=np.ascontiguousarray(hsv);height,width=pixels.shape[:2]
        planes=np.empty((5,height,width),np.uint8)
        if self.lib.adas_lamp_masks(pixels.ctypes.data,width,height,planes.ctypes.data,planes.nbytes):
            raise RuntimeError('native mask computation failed')
        return {key:planes[index] for index,key in enumerate(KEYS)}
