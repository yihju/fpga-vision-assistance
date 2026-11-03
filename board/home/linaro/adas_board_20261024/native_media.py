"""Synchronous MPP C bridge, latest-only live or exact sequential validation."""
import ctypes,time,hashlib,json
from pathlib import Path
import numpy as np
from adas.contracts import Frame,FrameKey
from adas.media import MediaReader as CpuReader,IMAGE_SUFFIXES

def validation_passes(record,bridge_sha):
    try:
        rows=record['rows'];keys={(row['source'],row['frame_id']) for row in rows}
        return (record['status']=='native_media_probe_pass' and record['bridge_sha256']==bridge_sha and
            len(rows)>=24 and len(keys)==len(rows) and len({row['source'] for row in rows})>=3 and
            all(row['pixels_equal'] is True and row['shape']==[1080,1920,3] and row['pts_us']>=0 for row in rows))
    except (KeyError,TypeError,ValueError):return False

class NativeMediaReader:
    verify=True
    library_name='libadas_media.so'
    validation_name='native-media-validation.json'
    live_mode=False
    reports=[]
    def __init__(self,path,*,epoch=0,camera_id=0):
        self.path=Path(path);self.epoch=epoch;self.camera_id=camera_id
        self.fallback=None;self.handle=None;self.status='ready';self.returned=0;self.skipped=0;self.last_id=-1;self.reported=False
        if self.path.suffix.lower() in IMAGE_SUFFIXES:
            self.fallback=CpuReader(path,epoch=epoch,camera_id=camera_id);self.fps=None;return
        if not self.path.is_file():raise ValueError('missing input')
        root=Path(__file__).resolve().parent;bridge=root/self.library_name
        if self.verify and not validation_passes(json.loads((root/self.validation_name).read_text()),hashlib.sha256(bridge.read_bytes()).hexdigest()):
            raise ValueError('native decoder equivalence gate failed')
        self.lib=ctypes.CDLL(str(bridge))
        self.lib.adas_media_open.argtypes=[ctypes.c_char_p,ctypes.c_int,ctypes.c_void_p];self.lib.adas_media_open.restype=ctypes.c_void_p
        self.lib.adas_media_read.argtypes=[ctypes.c_void_p,ctypes.c_void_p,ctypes.c_uint64,ctypes.c_void_p];self.lib.adas_media_read.restype=ctypes.c_int
        self.lib.adas_media_count.argtypes=[ctypes.c_void_p];self.lib.adas_media_count.restype=ctypes.c_uint64
        self.lib.adas_media_error.argtypes=[ctypes.c_void_p];self.lib.adas_media_error.restype=ctypes.c_char_p
        self.lib.adas_media_close.argtypes=[ctypes.c_void_p]
        dimensions=np.zeros(4,np.int32)
        self.handle=self.lib.adas_media_open(str(self.path.resolve()).encode(),int(self.live_mode),dimensions.ctypes.data)
        if not self.handle:raise RuntimeError('native MPP preroll failed')
        self.width,self.height,num,den=map(int,dimensions)
        self.fps=num/den if num>0 and den>0 else None
    def read(self):
        if self.fallback is not None:return self.fallback.read()
        if self.status=='closed':raise ValueError('closed reader')
        if self.status=='ended':return None
        pixels=np.empty((self.height,self.width,3),np.uint8);metadata=np.zeros(2,np.uint64)
        code=self.lib.adas_media_read(self.handle,pixels.ctypes.data,pixels.nbytes,metadata.ctypes.data)
        if code==0:self.status='ended';return None
        if code!=1:raise RuntimeError('native MPP '+str(code)+': '+self.lib.adas_media_error(self.handle).decode(errors='replace'))
        source_id,pts=map(int,metadata)
        if source_id<=self.last_id:raise RuntimeError('repeated or reordered decoder frame')
        self.skipped+=source_id-self.last_id-1;self.last_id=source_id;self.returned+=1
        pixels.flags.writeable=False
        return Frame(FrameKey(self.epoch,source_id,self.camera_id),pts//1000,time.perf_counter_ns(),pixels,'decoder_reported')
    def close(self):
        if self.fallback is not None:self.fallback.close()
        if not self.reported:
            type(self).reports.append(dict(source=str(self.path),epoch=self.epoch,live_latest_only=self.live_mode,
                decoded_source_frames=int(self.lib.adas_media_count(self.handle)) if self.handle else 0,
                returned_frames=self.returned,skipped_before_return=self.skipped,last_source_frame_id=self.last_id,
                reached_eos=self.status=='ended',bridge='native_C_MPP',identity_error=None))
            self.reported=True
        if self.handle:self.lib.adas_media_close(self.handle);self.handle=None
        self.status='closed'
    def __enter__(self):return self
    def __exit__(self,*unused):self.close()

class RgaMediaReader(NativeMediaReader):
    library_name='libadas_media_rga.so'
    validation_name='native-rga-validation.json'
