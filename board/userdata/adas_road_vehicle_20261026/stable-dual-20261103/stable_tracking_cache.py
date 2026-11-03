"""Two owned current-frame gray images; never reuse a different source array."""
from collections import OrderedDict
import threading
import cv2,numpy as np

class StableTrackingCache:
    def __init__(self,fallback):self.fallback=fallback;self.rows=OrderedDict();self.lock=threading.Lock();self.hits=0;self.misses=0
    def put(self,image,small):
        if not isinstance(image,np.ndarray) or image.shape!=(1080,1920,3) or image.dtype!=np.uint8:raise ValueError('native current pixels required')
        if not isinstance(small,np.ndarray) or small.shape!=(180,320,3) or small.dtype!=np.uint8:raise ValueError('exact current tracking RGB required')
        gray=cv2.cvtColor(small,cv2.COLOR_BGR2GRAY);gray.flags.writeable=False
        with self.lock:
            self.rows[id(image)]=(image,gray);self.rows.move_to_end(id(image))
            while len(self.rows)>2:self.rows.popitem(last=False)
    @property
    def pending(self):
        with self.lock:return len(self.rows)
    @property
    def identity(self):return dict(backend='exact_shared_nn_tracking_input',fallback=getattr(self.fallback,'identity',None),cross_frame_reuse=False)
    @property
    def native_calls(self):return self.hits+getattr(self.fallback,'native_calls',0)
    def __call__(self,frame,max_width):
        with self.lock:row=self.rows.pop(id(frame.bgr),None) if max_width==320 else None
        if row is not None and row[0] is frame.bgr:self.hits+=1;return row[1]
        self.misses+=1;return self.fallback(frame,max_width)
