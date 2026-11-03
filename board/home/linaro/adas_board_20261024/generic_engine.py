"""Original detector and native lamp search overlap on ONE immutable frame."""
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json,os,sys,time
from pathlib import Path

from adas.contracts import Observation
from adas.engine import RoadPerception,Analysis
from adas.native_signal import scan_native_signal,refine_native_signal


class GenericRoadPerception(RoadPerception):
    def __init__(self,*,search=None,feature_provider=None,**kwargs):
        super().__init__(**kwargs)
        self.fpga=None
        if search is None:
            import cv2
            import prepared_search
            import sparse_search
            import adas.reference_search
            from native_masks import NativeMasks
            root=Path(__file__).resolve().parent
            if (prepared_search.SPARSE_SOURCE_SHA256!=hashlib.sha256((root/'sparse_search.py').read_bytes()).hexdigest() or
                    sparse_search.SOURCE_SHA256!=hashlib.sha256(Path(adas.reference_search.__file__).read_bytes()).hexdigest()):
                raise ValueError('native search source identity changed')
            prepared_search.mask_builder=NativeMasks()
            cv_threads=int(os.environ.get('ADAS_GENERIC_CV_THREADS','1'))
            if cv_threads not in (1,2,4):raise ValueError('bounded OpenCV thread count required')
            cv2.setNumThreads(cv_threads)
            search=prepared_search.search_region
        self.search=search
        if feature_provider is None:
            from fpga_roi import FpgaRoi
            self.fpga=FpgaRoi();feature_provider=self.fpga.analyze
        self.feature_provider=feature_provider
        self.pool=ThreadPoolExecutor(max_workers=1,thread_name_prefix='same-frame-native')
        self.closed=False

    def analyze(self,frame,lane,*,scenario='intersection',workload='default'):
        if self.closed:raise RuntimeError('generic engine is closed')
        future=None
        began=time.perf_counter_ns();native_ms=native_cpu_ms=base_ms=merge_ms=None
        def scan():
            start=time.perf_counter_ns();cpu=time.thread_time_ns()
            packet=scan_native_signal(frame,self.search)
            return packet,(time.perf_counter_ns()-start)/1e6,(time.thread_time_ns()-cpu)/1e6
        try:
            if scenario=='intersection':future=self.pool.submit(scan)
            result=super().analyze(frame,lane,scenario=scenario,workload=workload)
            base_ms=(time.perf_counter_ns()-began)/1e6
            if future is not None:packet,native_ms,native_cpu_ms=future.result()
            else:packet=None
            if result.errors or scenario!='intersection':return result
            merge_start=time.perf_counter_ns()
            observations=tuple(refine_native_signal(frame,result.detections,lane,item,
                feature_provider=self.feature_provider,search_result=packet) if item.kind=='signal' else item
                for item in result.observations)
            merge_ms=(time.perf_counter_ns()-merge_start)/1e6
            return Analysis(observations,result.detections,())
        except Exception as exc:
            self.reset(frame.key.camera_id)
            kinds=('signal','left_boundary','right_boundary','path_clear','left_clear','right_clear','navigation')
            return Analysis(tuple(Observation(frame.key,k,'unknown',fresh=False) for k in kinds),(),
                            (f'{type(exc).__name__}: {exc}',))
        finally:
            # The worker's single shared pixel slot cannot be reused while any
            # branch still reads it, including detector failure/cancellation.
            if future is not None:
                try:future.result()
                except Exception:pass
            if os.environ.get('ADAS_STAGE_TIMINGS')=='1':
                print(json.dumps(dict(kind='generic_stage_timing',frame_id=frame.key.frame_id,
                    base_ms=base_ms,native_ms=native_ms,native_cpu_ms=native_cpu_ms,merge_ms=merge_ms,
                    total_ms=(time.perf_counter_ns()-began)/1e6)),file=sys.stderr,flush=True)

    def close(self):
        if not self.closed:
            self.closed=True;self.pool.shutdown(wait=True,cancel_futures=True)
            if self.fpga is not None:self.fpga.close()
