"""Two slots: ONE synchronous neural owner, overlapping previous-frame tracking."""
from concurrent.futures import ThreadPoolExecutor
from dataclasses import replace
import numpy as np
from .contracts import Frame


def _owned(frame):
    if frame is None:return None
    if not isinstance(frame,Frame) or not isinstance(frame.bgr,np.ndarray):raise ValueError('owned source frame required')
    # Readonly AND OWNDATA still permit a preexisting writable numpy alias.
    # No flag proves exclusive handoff from an arbitrary decoder callback.
    pixels=np.array(frame.bgr,copy=True,order='C');pixels.flags.writeable=False
    return replace(frame,bgr=pixels)


def run(read_latest,neural,track,*,limit):
    if type(limit) is not int or limit<1:raise ValueError('bounded positive result count required')
    with ThreadPoolExecutor(max_workers=1,thread_name_prefix='one-npu-owner') as pool:
        frame=_owned(read_latest())
        if frame is None:return
        pending=pool.submit(neural,frame)
        for index in range(limit):
            value=pending.result()
            current=frame
            # Dispatch next before consuming current result, without queuing
            # more than ONE request or changing either frame's source identity.
            frame=_owned(read_latest()) if index+1<limit else None
            pending=pool.submit(neural,frame) if frame is not None else None
            yield track(current,value)
            if pending is None:break
    # Executor shutdown drains a running call before the caller releases NPU.
