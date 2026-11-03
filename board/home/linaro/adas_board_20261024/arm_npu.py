"""Same reference weights on synchronous RKNN, gated by a saved numeric probe."""
import ctypes
import hashlib
import json
import time
import os
from pathlib import Path
import numpy as np

def preprocess_rgb(image):
    """Exact reference letterbox pixels without its float/NCHW round trip."""
    import cv2
    if (not isinstance(image,np.ndarray) or image.dtype!=np.uint8 or image.ndim!=3 or
        image.shape[2]!=3 or min(image.shape[:2])<1):raise ValueError('nonempty UINT8 BGR required')
    h,w=image.shape[:2];ratio=min(640/w,640/h);rw,rh=int(w*ratio),int(h*ratio)
    if min(rw,rh)<1:raise ValueError('image aspect too extreme')
    left,top=(640-rw)//2,(640-rh)//2
    canvas=np.zeros((640,640,3),np.uint8)
    canvas[top:top+rh,left:left+rw]=cv2.resize(image,(rw,rh))
    return cv2.cvtColor(canvas,cv2.COLOR_BGR2RGB),dict(r=ratio,left=left,top=top)

def detect_rgb(detector,frame):
    from adas.perception.reference_yolo import decode_heads,Detection
    began=time.perf_counter_ns();image=frame.bgr
    fused=(getattr(detector.session,'supports_bgr3',False) and isinstance(image,np.ndarray) and
        image.shape==(1080,1920,3) and image.dtype==np.uint8 and image.flags.c_contiguous)
    if fused:transform=dict(r=640/1920,left=0,top=140)
    else:rgb,transform=preprocess_rgb(image)
    prepared=time.perf_counter_ns()
    started=time.perf_counter_ns();outputs=detector.session.run_bgr3(image) if fused else detector.session.run_rgb(rgb)
    detector.last_inference_ms=(time.perf_counter_ns()-started)/1e6
    rows=decode_heads(outputs,detector.anchors,domain=detector.manifest['onnx_output_domain'])
    if os.environ.get('ADAS_STAGE_TIMINGS')=='1':
        import sys
        print(json.dumps(dict(kind='npu_stage_timing',frame_id=frame.key.frame_id,preprocess_ms=(prepared-began)/1e6,
            inference_ms=detector.last_inference_ms,postprocess_ms=(time.perf_counter_ns()-started)/1e6-detector.last_inference_ms,
            native_stages_ms=getattr(detector.session,'native_stages_ms',None))),file=sys.stderr,flush=True)
    ratio,left,top=(transform[k] for k in ('r','left','top'));h,w=frame.bgr.shape[:2]
    detections=[]
    for row in rows:
        x0,y0,x1,y1=row['bbox_xyxy']
        box=(max(0.,(x0-left)/ratio),max(0.,(y0-top)/ratio),
             min(float(w),(x1-left)/ratio),min(float(h),(y1-top)/ratio))
        if box[0]<box[2] and box[1]<box[3]:
            detections.append(Detection(frame.key,row['label'],row['confidence'],box))
    return detections

def probe_passes(rows):
    """Mean numeric proximity AND class/box/score parity; averages cannot hide lamps."""
    if not rows:return False
    try:
        for row in rows:
            errors=row['probability_mae']
            if len(errors)!=3 or any(not np.isfinite(v) or not 0<=v<.01 for v in errors):return False
            expected=row['cpu_detections'];remaining=list(row['npu_detections'])
            if len(expected)!=len(remaining):return False
            for old in expected:
                match=None
                for index,new in enumerate(remaining):
                    if old['label']!=new['label'] or abs(old['confidence']-new['confidence'])>.03:continue
                    a,b=old['bbox_xyxy'],new['bbox_xyxy']
                    area=max(0,min(a[2],b[2])-max(a[0],b[0]))*max(0,min(a[3],b[3])-max(a[1],b[1]))
                    union=(a[2]-a[0])*(a[3]-a[1])+(b[2]-b[0])*(b[3]-b[1])-area
                    if union>0 and area/union>=.9:match=index;break
                if match is None:return False
                remaining.pop(match)
        return True
    except (KeyError,TypeError,ValueError,IndexError):return False

def native_probe_passes(record,reference_sha):
    try:
        rows=[row for row in record['rows'] if row['normalization']==1]
        return (record['status']=='native_input_probe_pass' and record['model_changed'] is False and
            record['reference_validation_sha256']==reference_sha and len(rows)>=3 and probe_passes(rows) and
            all(row['probability_mae']==[0.]*3 and row['raw_max_diff']==[0.]*3 for row in rows))
    except (KeyError,TypeError,ValueError):return False

def output_probabilities(outputs,domain,*,in_place=False):
    if domain not in ('probabilities','logits') or len(outputs)!=3:raise ValueError('three heads and explicit output domain required')
    for value,n in zip(outputs,(80,40,20)):
        if (value.shape!=(1,33,n,n) or value.dtype!=np.float32 or not np.isfinite(value).all()):
            raise ValueError('finite float32 reference heads required')
        if domain=='probabilities' and (np.any(value<0) or np.any(value>1)):
            raise ValueError('not probabilities')
    if domain=='probabilities':return outputs
    if not in_place:return [1/(1+np.exp(-np.clip(o,-80,80))) for o in outputs]
    # Caller owns these fresh synchronous output buffers; no retained raw heads.
    for value in outputs:
        np.clip(value,-80,80,out=value)
        np.negative(value,out=value);np.exp(value,out=value)
        np.add(value,1,out=value);np.reciprocal(value,out=value)
    return outputs

class Session:
    def __init__(self,root,*,native=False,fused=False):
        self.root=Path(root)
        record=json.loads((self.root/'npu-validation-fp16.json').read_text())
        if (record['status']!='numeric_probe_pass' or not probe_passes(record.get('rows',[])) or len(record['rows'])<3 or record.get('pruning',True) or record.get('quantization',True) or
            record.get('source_onnx_sha256')!='8e53bfd36cf848696a4eb3ca6727929dab2993968c00ecbf75aa92830f73e599'):
            raise ValueError('unapproved RKNN export')
        self.domain=record['domain']
        model=self.root/'reference_fp16.rknn';bridge=self.root/'libadas_npu.so'
        for path,key in ((model,'rknn_sha256'),(bridge,'bridge_sha256')):
            if hashlib.sha256(path.read_bytes()).hexdigest()!=record[key]:raise ValueError('NPU artifact drift')
        if native:
            candidate=json.loads((self.root/'native-input-validation.json').read_text())
            reference_sha=hashlib.sha256((self.root/'npu-validation-fp16.json').read_bytes()).hexdigest()
            bridge=self.root/'libadas_npu_native.so'
            if (not native_probe_passes(candidate,reference_sha) or candidate['rknn_sha256']!=record['rknn_sha256'] or
                hashlib.sha256(bridge.read_bytes()).hexdigest()!=candidate['bridge_sha256']):
                raise ValueError('native input equivalence gate failed')
        if fused:
            candidate=json.loads((self.root/'fused-input-validation.json').read_text())
            reference_sha=hashlib.sha256((self.root/'npu-validation-fp16.json').read_bytes()).hexdigest()
            bridge=self.root/'libadas_npu_fused.so'
            if (candidate.get('tensor_bit_exact') is not True or not native_probe_passes(candidate,reference_sha) or
                candidate['rknn_sha256']!=record['rknn_sha256'] or hashlib.sha256(bridge.read_bytes()).hexdigest()!=candidate['bridge_sha256']):
                raise ValueError('fused input equivalence gate failed')
        self.lib=ctypes.CDLL(str(bridge));self.lib.adas_npu_open.argtypes=[ctypes.c_char_p]
        self.lib.adas_npu_open.restype=ctypes.c_void_p;self.lib.adas_npu_run.argtypes=[ctypes.c_void_p]*5
        self.lib.adas_npu_run.restype=ctypes.c_int;self.lib.adas_npu_close.argtypes=[ctypes.c_void_p]
        self.supports_bgr3=fused
        if fused:
            self.lib.adas_npu_run_bgr3.argtypes=[ctypes.c_void_p,ctypes.c_void_p,ctypes.c_uint64]+[ctypes.c_void_p]*3
            self.lib.adas_npu_run_bgr3.restype=ctypes.c_int
        self.handle=self.lib.adas_npu_open(str(model).encode())
        if not self.handle:raise RuntimeError('RKNN init/shape validation failed')
    def run(self,names,feeds):
        tensor=next(iter(feeds.values()))
        if tensor.shape!=(1,3,640,640) or tensor.dtype!=np.float32 or not np.isfinite(tensor).all():
            raise ValueError('reference input contract')
        rgb=np.ascontiguousarray(np.rint(tensor[0].transpose(1,2,0)*255).clip(0,255),dtype=np.uint8)
        return self.run_rgb(rgb)
    def run_rgb(self,rgb):
        if rgb.shape!=(640,640,3) or rgb.dtype!=np.uint8 or not rgb.flags.c_contiguous:
            raise ValueError('contiguous reference RGB UINT8 required')
        return self._execute(self.lib.adas_npu_run,[self.handle,rgb.ctypes.data])
    def run_bgr3(self,bgr):
        if (not self.supports_bgr3 or bgr.shape!=(1080,1920,3) or bgr.dtype!=np.uint8 or not bgr.flags.c_contiguous):
            raise ValueError('fused input requires exact native contiguous BGR shape')
        return self._execute(self.lib.adas_npu_run_bgr3,[self.handle,bgr.ctypes.data,bgr.nbytes])
    def _execute(self,function,arguments):
        outputs=[np.empty((1,33,n,n),np.float32) for n in (80,40,20)]
        code=function(*arguments,*[o.ctypes.data for o in outputs])
        if code:raise RuntimeError('synchronous RKNN inference '+str(code))
        if os.environ.get('ADAS_STAGE_TIMINGS')=='1' and hasattr(self.lib,'adas_npu_timing'):
            self.lib.adas_npu_timing.argtypes=[ctypes.c_void_p]*2
            stages=np.zeros(3,np.float64);self.lib.adas_npu_timing(self.handle,stages.ctypes.data)
            self.native_stages_ms=stages.tolist()
        return output_probabilities(outputs,self.domain,in_place=True)
    def __del__(self):
        if getattr(self,'handle',None):self.lib.adas_npu_close(self.handle);self.handle=None

def install(root):
    import adas.perception.reference_yolo as reference
    Original=reference.ReferenceDetector
    class ArmNpuDetector(Original):
        detect=detect_rgb
        def __init__(self,*,threads=4):
            manifest_path=reference.ROOT/'adas/assets/reference-yolo.json'
            self.manifest=json.loads(manifest_path.read_text(encoding='utf-8'))
            folder=reference.ROOT/self.manifest['directory_relative_to_project']
            for kind,digest in (('model','model_sha256'),('anchors','anchor_sha256')):
                if hashlib.sha256((folder/self.manifest[kind]).read_bytes()).hexdigest()!=self.manifest[digest]:
                    raise ValueError('approved reference asset changed')
            self.anchors=np.fromstring((folder/self.manifest['anchors']).read_text().replace(',',' '),sep=' ',dtype=np.float32).reshape(9,2)
            self.session=Session(root,native=True,fused=True);self.name='images';self.last_inference_ms=None
            self.manifest=dict(self.manifest,execution_backend='RK3568 synchronous FP16 RKNN',
                npu_validation=str(Path(root)/'npu-validation-fp16.json'),training=False)
    reference.ReferenceDetector=ArmNpuDetector
