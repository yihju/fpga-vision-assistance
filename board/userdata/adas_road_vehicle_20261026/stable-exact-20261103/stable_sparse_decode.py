"""Exact current stable head scan; original float32 geometry and NMS unchanged."""
from pathlib import Path
import ctypes as C,hashlib,json,math
import numpy as np
try:from adas.road_vehicle_probe import decode_vehicles,_nms
except ModuleNotFoundError:from road_vehicle_probe import decode_vehicles,_nms

def decode_selected(detections,mapping,indices,scores,*,nms_iou=.45):
    rows=detections[0,indices]
    boxes=np.concatenate((rows[:,:2]-rows[:,2:4]/2,rows[:,:2]+rows[:,2:4]/2),axis=1)
    boxes=mapping.native_boxes(boxes)
    keep=_nms(boxes,scores,nms_iou)
    return [dict(xyxy=boxes[i].tolist(),score=float(scores[i]),label='vehicle') for i in keep]

class StableSparseDecoder:
    def __init__(self,root,*,verify=True):
        root=Path(root);library=root/'libstable_decode.so';source=root/'stable_decode.c'
        if verify:
            proof=json.loads((root/'decoder-validation.json').read_text())
            if (proof.get('all_outputs_exact') is not True or proof.get('cases',0)<12 or
                proof.get('library_sha256')!=hashlib.sha256(library.read_bytes()).hexdigest() or
                proof.get('source_sha256')!=hashlib.sha256(source.read_bytes()).hexdigest()):raise ValueError('stable exact decode proof required')
        self.library=C.CDLL(str(library.resolve()));self.library.stable_select.argtypes=[C.c_void_p,C.c_size_t,*([C.c_float]*5),C.c_void_p,C.c_void_p,C.c_size_t];self.library.stable_select.restype=C.c_int
        self.indices=np.empty(25200,np.uint32);self.scores=np.empty(25200,np.float32);self.calls=0
    def __call__(self,detections,mapping,*,confidence=.25,nms_iou=.45):
        # General input support stays on the original implementation. Production
        # uses one synchronous owner and the fixed contiguous float32 head.
        value=np.asarray(detections)
        if value.dtype!=np.float32 or value.shape!=(1,25200,6) or not value.flags.c_contiguous:
            return decode_vehicles(detections,mapping,confidence=confidence,nms_iou=nms_iou)
        if not 0<confidence<1 or not 0<nms_iou<1:raise ValueError('valid decoder threshold required')
        if not 0<float(np.float32(confidence))<1:
            return decode_vehicles(detections,mapping,confidence=confidence,nms_iou=nms_iou)
        if any(type(v) not in (float,int) or not math.isfinite(v) for v in (mapping.left,mapping.top,mapping.width,mapping.height)):
            return decode_vehicles(detections,mapping,confidence=confidence,nms_iou=nms_iou)
        count=self.library.stable_select(value.ctypes.data,25200,confidence,mapping.left,mapping.top,mapping.width,mapping.height,self.indices.ctypes.data,self.scores.ctypes.data,25200)
        if count<0:raise ValueError('invalid original probability head: '+str(count))
        self.calls+=1
        return decode_selected(value,mapping,self.indices[:count],self.scores[:count],nms_iou=nms_iou)
