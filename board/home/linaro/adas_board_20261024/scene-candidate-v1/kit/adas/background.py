"""One in-flight analysis in a spawned child; never owns UI or hardware."""
from dataclasses import dataclass
import multiprocessing as mp
from queue import Empty, Full
import time

from .contracts import Frame, Observation
from .engine import Analysis, RoadPerception
from .workload import validate_workload


def invalid_analysis(key, reason):
    kinds = ('signal', 'left_boundary', 'right_boundary', 'path_clear', 'left_clear', 'right_clear', 'navigation')
    return Analysis(tuple(Observation(key, k, 'unknown', fresh=False) for k in kinds), (), (reason,))


@dataclass(frozen=True)
class AnalysisResult:
    request_id: int
    frame: Frame
    lane: str
    revision: int
    analysis: Analysis
    submitted_ns: int
    observed_ns: int
    worker_ms: float
    workload: str = 'default'


def _serve(requests, results, factory, kwargs):
    try:
        engine = factory(**kwargs)
        detector = getattr(engine, 'detector', None)
        results.put(('ready', getattr(detector, 'manifest', getattr(engine, 'manifest', {}))))
        context = None
        while True:
            item = requests.get()
            if item is None:
                return
            request_id, frame, lane, revision, scenario = item[:5]
            workload = item[5] if len(item) == 6 else 'default'
            started = time.perf_counter_ns()
            try:
                validate_workload(workload, scenario)
                current = (frame.key.epoch, frame.key.camera_id, lane, revision, scenario, workload)
                if current != context:
                    engine.reset()
                    context = current
                # Preserve old injected providers for the unchanged default API.
                if workload == 'default':
                    analysis = engine.analyze(frame, lane, scenario=scenario)
                else:
                    analysis = engine.analyze(frame, lane, scenario=scenario, workload=workload)
            except Exception as exc:
                analysis = invalid_analysis(frame.key, f'{type(exc).__name__}: {exc}')
            elapsed = (time.perf_counter_ns()-started)/1e6
            result = ('result', request_id, frame.key, revision, analysis, elapsed)
            results.put((*result, workload) if workload != 'default' else result)
    except Exception as exc:
        results.put(('fatal', f'{type(exc).__name__}: {exc}'))


class AnalysisWorker:
    def __init__(self, *, engine_factory=RoadPerception, engine_kwargs=None):
        self.context = mp.get_context('spawn')
        self.requests = self.context.Queue(maxsize=1)
        self.results = self.context.Queue(maxsize=1)
        self.process = self.context.Process(target=_serve, args=(self.requests, self.results, engine_factory,
                                                               dict(engine_kwargs or {})), daemon=True)
        self.ready, self.closed, self.manifest = False, False, {}
        self.inflight, self.sequence = None, 0
        self.process.start()

    @property
    def busy(self):
        return self.inflight is not None

    @property
    def alive(self):
        return self.process.is_alive()

    @property
    def pending_since_ns(self):
        return self.inflight[4] if self.inflight is not None else None

    def submit(self, frame, lane, revision, scenario, *, workload='default'):
        if self.closed:
            raise RuntimeError('worker is closed')
        if not self.alive:
            raise RuntimeError(f'worker exited ({self.process.exitcode})')
        if lane not in ('left', 'straight', 'right') or scenario not in ('lane', 'intersection', 'navigation'):
            raise ValueError('invalid perception context')
        validate_workload(workload, scenario)
        if not self.ready or self.busy:
            return False
        request_id = self.sequence
        submitted = time.perf_counter_ns()
        try:
            request = (request_id, frame, lane, revision, scenario)
            self.requests.put_nowait((*request, workload) if workload != 'default' else request)
        except Full:
            return False
        self.inflight = (request_id, frame, lane, revision, submitted, workload)
        self.sequence += 1
        return True

    def poll(self):
        if self.closed:
            return None
        try:
            message = self.results.get_nowait()
        except Empty:
            if not self.alive:
                raise RuntimeError(f'worker exited ({self.process.exitcode})')
            return None
        if message[0] == 'fatal':
            raise RuntimeError('worker failed: '+message[1])
        if message[0] == 'ready':
            self.ready, self.manifest = True, message[1]
            return None
        if message[0] != 'result' or self.inflight is None:
            raise RuntimeError('unexpected worker result')
        request_id, frame, lane, revision, submitted, workload = self.inflight
        if message[1:4] != (request_id, frame.key, revision):
            raise RuntimeError('worker result identity mismatch')
        returned_workload = message[6] if len(message) == 7 else 'default'
        if len(message) not in (6, 7) or returned_workload != workload:
            raise RuntimeError('worker result workload mismatch')
        analysis = message[4]
        if (not isinstance(analysis, Analysis) or
                any(o.key != frame.key for o in (*analysis.observations, *analysis.detections))):
            analysis = invalid_analysis(frame.key, 'worker returned mixed frame identity')
        self.inflight = None
        return AnalysisResult(request_id, frame, lane, revision, analysis, submitted, time.perf_counter_ns(), message[5], workload)

    def close(self):
        if self.closed:
            return
        self.closed = True
        try:
            self.requests.put_nowait(None)
        except Full:
            pass
        self.process.join(.2)
        if self.alive:
            self.process.terminate()
            self.process.join(.5)
        if self.alive:
            self.process.kill()
            self.process.join(.5)
        # A pending large image must not make interpreter exit wait for a pipe.
        for channel in (self.requests, self.results):
            channel.cancel_join_thread()
            channel.close()
        self.inflight = None

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()
