"""Bounded latest slot; retain decoder timestamps, source IDs and PTS unchanged."""
import threading,time
from native_media import RgaMediaReader

class AsyncLatestReader:
    live_mode=True
    demand_driven=False
    busy=staticmethod(lambda:False)
    reports=[]
    reader_factory=RgaMediaReader
    def __init__(self,path,*,epoch=0,camera_id=0):
        self.reader_factory.live_mode=self.live_mode
        self.reader=self.reader_factory(path,epoch=epoch,camera_id=camera_id)
        self.fps=self.reader.fps;self.condition=threading.Condition();self.stopping=threading.Event()
        self.thread=None;self.latest=None;self.finished=False;self.error=None;self.closed=False
        self.request=threading.Event();self.requested=False
        self.decode_ms=[];self.consumer_drops=0;self.consumed=0;self.consume_age_ms=[]
        self.static=self.reader.fallback is not None
    def _run(self):
        try:
            while not self.stopping.is_set():
                if self.demand_driven:
                    if not self.request.wait(.1):continue
                    self.request.clear()
                    if self.stopping.is_set():return
                started=time.perf_counter_ns();frame=self.reader.read()
                elapsed=(time.perf_counter_ns()-started)/1e6
                with self.condition:
                    self.requested=False
                    if frame is None:self.finished=True;self.condition.notify_all();return
                    self.decode_ms.append(elapsed)
                    if self.latest is not None:self.consumer_drops+=1
                    self.latest=frame;self.condition.notify_all()
        except Exception as exc:
            with self.condition:self.error=exc;self.finished=True;self.condition.notify_all()
    @property
    def ready(self):
        if self.closed:raise ValueError('closed reader')
        if self.static:return True
        with self.condition:
            self._start()
            if self.demand_driven and self.latest is None and not self.finished and not self.busy():self._request()
            return self.latest is not None or self.finished
    def _start(self):
        if self.thread is None:
            self.thread=threading.Thread(target=self._run,daemon=True,name='native-latest-decode');self.thread.start()
    def _request(self):
        if not self.requested:self.requested=True;self.request.set()
    def read(self):
        if self.closed:raise ValueError('closed reader')
        if self.static:return self.reader.read()
        with self.condition:
            # Only the first source read waits, to keep ReplayClock anchored
            # before decoder PLAYING. Later reads are guarded by ready.
            if self.thread is None:
                self._start()
                if self.demand_driven:self._request()
                deadline=time.monotonic()+6
                while self.latest is None and not self.finished and time.monotonic()<deadline:
                    self.condition.wait(.02)
            if self.error is not None:raise RuntimeError('decoder worker failed') from self.error
            if self.latest is not None:
                frame=self.latest;self.latest=None;self.consumed+=1
                self.consume_age_ms.append((time.perf_counter_ns()-frame.decoded_ns)/1e6)
                return frame
            if self.finished:return None
            raise BlockingIOError('decoder has not published a frame')
    def close(self):
        if self.closed:return
        self.stopping.set()
        self.request.set()
        if self.thread is not None:
            self.thread.join(6)
            if self.thread.is_alive():raise RuntimeError('native decoder did not stop; refusing to free active buffer')
        self.reader.close();self.closed=True
        self.latest=None
        type(self).reports.append(dict(source=str(self.reader.path),decoder_reports=self.reader.reports[-1:] if not self.static else [],
            async_consumed=self.consumed,async_overwritten=self.consumer_drops,
            decoder_call_ms=self.decode_ms,completed_to_consume_ms=self.consume_age_ms,
            decoder_error=str(self.error) if self.error else None,decoder_timestamp_retained=True))
    def __enter__(self):return self
    def __exit__(self,*unused):self.close()
