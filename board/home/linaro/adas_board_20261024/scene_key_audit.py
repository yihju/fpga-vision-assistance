"""Record actual physical levels and decoded events without a second decoder."""
import json
import time
from pathlib import Path


class AuditedKeys:
    def __init__(self,factory,path):
        self.source=factory();self.decoder=self.source.decoder
        try:self.stream=Path(path).open('x',encoding='utf-8',buffering=1)
        except Exception:self.source.close();raise
        self.closed=False;self.previous_levels=None;self.error_reported=False

    @property
    def last_levels(self):return self.source.last_levels

    def emit(self,kind,**values):
        self.stream.write(json.dumps(dict(kind=kind,monotonic_ns=time.perf_counter_ns(),**values))+'\n')

    def sample(self):
        try:events=self.source.sample()
        except Exception as exc:
            if not self.error_reported:
                self.error_reported=True;self.emit('input_error',error=type(exc).__name__,message=str(exc))
            raise
        levels=self.source.last_levels
        if levels!=self.previous_levels:
            self.previous_levels=levels;self.emit('raw_levels',levels=levels)
        if events:self.emit('physical_events',events=list(events),levels=levels)
        return events

    def close(self):
        if self.closed:return
        self.closed=True
        try:self.source.close()
        finally:
            try:self.emit('input_closed')
            finally:self.stream.close()
