"""One in-flight spawned worker, one shared pixel slot; no image pipe copies."""
from dataclasses import replace
from multiprocessing import shared_memory
import numpy as np
from adas.background import AnalysisWorker,_serve
from adas.reference_profile import ReferencePerception
ACTIVE_WORKER=None

class SharedPixels(np.ndarray):
    def __array_finalize__(self,other):self.shared_name=getattr(other,'shared_name',None)

class _SharedRequests:
    def __init__(self,queue):self.queue=queue;self.memory=None
    def get(self):
        item=self.queue.get()
        if item is None:return None
        frame=item[1];metadata=frame.bgr
        if self.memory is None or self.memory.name!=metadata['name']:
            if self.memory is not None:self.memory.close()
            self.memory=shared_memory.SharedMemory(name=metadata['name'])
        pixels=np.ndarray(tuple(metadata['shape']),np.uint8,buffer=self.memory.buf).view(SharedPixels)
        pixels.shared_name=metadata['name']
        pixels.flags.writeable=False
        return (item[0],replace(frame,bgr=pixels),*item[2:])
    def close(self):
        if self.memory is not None:self.memory.close()

def _serve_shared(requests,results,factory,kwargs,parent_pid):
    from roi_process import bind_parent
    bind_parent(parent_pid)
    import sys
    sys.setswitchinterval(.0005)  # bound Python housing-search/GIL stalls on NPU postprocessing
    import cv2
    cv2.setNumThreads(1)  # spawn does not inherit the parent's OpenCV setting
    source=_SharedRequests(requests);owned=[]
    def create(**options):
        engine=factory(**options);owned.append(engine);return engine
    try:_serve(source,results,create,kwargs)
    finally:
        for engine in owned:
            target=getattr(engine,'reference',engine)
            if hasattr(target,'close'):target.close()
        source.close()

class SharedFrameWorker(AnalysisWorker):
    def __init__(self,*,engine_factory=ReferencePerception,engine_kwargs=None):
        global ACTIVE_WORKER
        ACTIVE_WORKER=self
        import multiprocessing as mp,os
        self.context=mp.get_context('spawn')
        self.requests=self.context.Queue(maxsize=1);self.results=self.context.Queue(maxsize=1)
        self.ready,self.closed,self.manifest=False,False,{}
        self.inflight,self.sequence=None,0;self.memory=None
        self.process=self.context.Process(target=_serve_shared,args=(self.requests,self.results,
            engine_factory,dict(engine_kwargs or {}),os.getpid()),daemon=False)
        self.process.start()
    def submit(self,frame,lane,revision,scenario,*,workload='default'):
        if self.closed:raise RuntimeError('worker is closed')
        if not self.alive:raise RuntimeError('worker exited')
        if not self.ready or self.busy:return False
        pixels=frame.bgr
        if (not isinstance(pixels,np.ndarray) or pixels.dtype!=np.uint8 or pixels.ndim!=3 or
            pixels.shape[2]!=3 or min(pixels.shape[:2])<1):raise ValueError('nonempty uint8 BGR required')
        if self.memory is None or self.memory.size<pixels.nbytes:
            self._free_slot();self.memory=shared_memory.SharedMemory(create=True,size=pixels.nbytes)
        slot=np.ndarray(pixels.shape,np.uint8,buffer=self.memory.buf);np.copyto(slot,pixels)
        metadata=replace(frame,bgr=dict(name=self.memory.name,shape=list(pixels.shape)))
        accepted=super().submit(metadata,lane,revision,scenario,workload=workload)
        if accepted:self.inflight=(self.inflight[0],frame,*self.inflight[2:])
        return accepted
    def _free_slot(self):
        if self.memory is not None:
            self.memory.close();self.memory.unlink();self.memory=None
    def close(self):
        global ACTIVE_WORKER
        try:super().close()
        finally:
            self._free_slot()
            if ACTIVE_WORKER is self:ACTIVE_WORKER=None
