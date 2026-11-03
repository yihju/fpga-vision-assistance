"""Opt-in full native upper-view lamp search; no per-video calibrated ROI.

Only unknown intersection observations can be supplemented. Neural outputs
stay unchanged; geometric evidence is reported separately and never cached.
"""
import math
from dataclasses import asdict, replace, dataclass

from .engine import Analysis, RoadPerception
from .perception.reference_yolo import Detection
from .perception.signal import observe_signals
from .reference_search import search_region


@dataclass(frozen=True)
class NativeSearchResult:
    frame: object
    bounds: tuple
    color: str
    heads: tuple


def scan_native_signal(frame,search=None):
    height,width=frame.bgr.shape[:2]
    bounds=(0,0,width,math.ceil(height*.7))
    color,heads=(search or search_region)(frame.bgr,bounds)
    return NativeSearchResult(frame,bounds,color,tuple(heads))


def refine_native_signal(frame, detections, lane, baseline, *, feature_provider=None,search_result=None):
    if (baseline.key != frame.key or baseline.kind != 'signal' or
            any(item.key != frame.key for item in detections)):
        raise ValueError('current signal and same-frame detections required')
    if lane not in ('left', 'straight', 'right'):
        raise ValueError('valid lane required')
    if (baseline.value != 'unknown' or not baseline.fresh or not baseline.applicable or
            baseline.details.get('unresolved_prohibition')):
        return baseline
    height, width = frame.bgr.shape[:2]
    bounds = (0, 0, width, math.ceil(height*.7))
    if search_result is None:
        color,heads=search_region(frame.bgr,bounds)
    else:
        if (not isinstance(search_result,NativeSearchResult) or search_result.frame is not frame or
                search_result.bounds!=bounds):
            raise ValueError('native search belongs to a different frame or bounds')
        color,heads=search_result.color,list(search_result.heads)
    # No selecting a favorable housing among conflicting directions/colors.
    if not heads or color not in ('red', 'green', 'yellow'):
        return baseline
    geometry = []
    for head in heads:
        x0, y0, x1, y1 = head['bbox_xyxy']
        box = (max(0, x0-2), max(0, y0-2), min(width, x1+2), min(height, y1+2))
        # Existing signal code uses label/box pixels, not this geometric score.
        geometry.append(Detection(frame.key, head['color'], head['circularity'], box))
    signal = observe_signals(frame, (*detections, *geometry), lane, required=True)
    signal.details['native_fallback'] = dict(
        source='current_native_pixels', calibrated_roi=False, search_bounds_xyxy=list(bounds),
        evidence_kind='housing_geometry_not_neural_confidence', heads=heads,
        baseline_value=baseline.value, experimental=True)
    if feature_provider is not None and signal.value=='green':
        features=[]
        for detection in geometry:
            box=tuple(detection.bbox_xyxy)
            result=feature_provider(frame,box)
            area=(box[2]-box[0])*(box[3]-box[1])
            if (result.get('key')!=asdict(frame.key) or result.get('bbox_xyxy')!=list(box) or
                    result.get('count')!=area or
                    any(type(result.get(k)) is not int or not 0<=result[k]<=area for k in ('red','green')) or
                    type(result.get('gray_max')) is not int or not 0<=result['gray_max']<=255):
                raise ValueError('invalid or foreign FPGA frame/ROI feature identity')
            features.append(result)
            if result['green']<8 or result['green']<=2*result['red'] or result['gray_max']<128:
                return replace(signal,value='unknown',details=dict(signal.details,
                    fpga_features=features,fpga_veto='green_pixel_support_insufficient'))
        signal.details['fpga_features']=features
    return signal


class NativeRoadPerception(RoadPerception):
    """Same original model/tasks; replace only the unknown intersection signal."""
    def __init__(self, *args, feature_provider=None, **kwargs):
        super().__init__(*args,**kwargs)
        self.feature_provider=feature_provider

    def analyze(self, frame, lane, *, scenario='intersection', workload='default'):
        result = super().analyze(frame, lane, scenario=scenario, workload=workload)
        if result.errors or scenario != 'intersection':
            return result
        try:
            observations = tuple(refine_native_signal(frame, result.detections, lane, item,
                                                       feature_provider=self.feature_provider)
                                 if item.kind == 'signal' else item for item in result.observations)
            return Analysis(observations, result.detections, ())
        except Exception as exc:
            # No successful green can escape an invalid native supplement.
            from .contracts import Observation
            observations = tuple(Observation(frame.key, item.kind, 'unknown', fresh=False)
                                 for item in result.observations)
            return Analysis(observations, result.detections, (f'{type(exc).__name__}: {exc}',))
