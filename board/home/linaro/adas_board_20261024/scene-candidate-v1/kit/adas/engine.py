"""Synchronous PC perception, all observations bound to the analyzed frame."""
from dataclasses import dataclass

from .contracts import Frame, Observation
from .frame_tasks import frame_tasks
from .workload import validate_workload
from .perception.lane import observe_boundaries
from .perception.lane_sequence import LaneConsistency
from .perception.lane_sampled import SampledLaneConsistency
from .perception.pedestrian import observe_pedestrians
from .perception.reference_yolo import ReferenceDetector
from .perception.signal import SignalAssociation, observe_signals
from .perception.lamp_heads import refine_signal
from .perception.navigation import observe_navigation, validate_primary_region


@dataclass(frozen=True)
class Analysis:
    observations: tuple
    detections: tuple
    errors: tuple[str, ...]


class RoadPerception:
    def __init__(self, detector=None, *, threads=4, intersection_fast=False, parallel_perception=False, lamp_heads=False, sampled_lines=False, navigation_primary_region=None):
        if type(sampled_lines) is not bool:
            raise ValueError('sampled_lines must be an explicit boolean')
        if type(lamp_heads) is not bool:
            raise ValueError('lamp_heads must be an explicit boolean')
        if type(intersection_fast) is not bool:
            raise ValueError('intersection_fast must be an explicit boolean')
        if type(parallel_perception) is not bool:
            raise ValueError('parallel_perception must be an explicit boolean')
        if parallel_perception and intersection_fast:
            raise ValueError('parallel full perception and intersection subset are mutually exclusive')
        if parallel_perception and detector is not None and type(detector) is not ReferenceDetector:
            raise ValueError('parallel perception requires the built-in read-only ReferenceDetector')
        self.intersection_fast = intersection_fast
        self.parallel_perception = parallel_perception
        self.lamp_heads = lamp_heads
        self.navigation_primary_region = (validate_primary_region(navigation_primary_region)
                                          if navigation_primary_region is not None else None)
        self.detector = detector if detector is not None else ReferenceDetector(threads=threads)
        self.signal_association = SignalAssociation()
        self.lane_consistency = LaneConsistency()
        self.sampled_lane_consistency = SampledLaneConsistency() if sampled_lines else None

    def reset(self, camera=None):
        self.signal_association.reset(camera)
        self.lane_consistency.reset(camera)
        if self.sampled_lane_consistency is not None:self.sampled_lane_consistency.reset(camera)

    def _detect(self, frame):
        detections = tuple(self.detector.detect(frame))
        if any(d.key != frame.key for d in detections):
            raise ValueError('detector returned a different frame')
        return detections

    def analyze(self, frame, lane, *, scenario='intersection', workload='default'):
        if scenario not in ('intersection', 'lane', 'navigation'):
            raise ValueError('implemented scenario must be intersection, lane or navigation')
        validate_workload(workload, scenario)
        skip_boundaries = (self.intersection_fast and scenario == 'intersection') or workload == 'intersection_light'
        try:
            if self.sampled_lane_consistency is not None and scenario!='lane':
                self.sampled_lane_consistency.reset(frame.key.camera_id)
            navigation, road_frame = None, frame
            if scenario == 'navigation':
                navigation_options = (dict(primary_region=self.navigation_primary_region)
                                      if self.navigation_primary_region is not None else {})
                navigation = observe_navigation(frame, required=True, **navigation_options)
                screen = navigation.details['screen_bbox_xyxy']
                if navigation.details['view'] in ('display_only', 'unknown'):
                    self.reset(frame.key.camera_id)
                    reason = ('display_only_no_real_road' if navigation.details['view'] == 'display_only'
                              else 'navigation_road_domain_unresolved')
                    kinds = ('signal', 'left_boundary', 'right_boundary', 'path_clear', 'left_clear', 'right_clear')
                    unavailable = tuple(Observation(frame.key, kind, 'unknown', details=dict(reason=reason))
                                        for kind in kinds)
                    return Analysis((*unavailable, navigation), (), ())
                if screen is not None:
                    # Bottom display/dashboard pixels must not become road lines
                    # or people. No rescaling or translation: overlay coordinates
                    # in this top-origin road crop remain original-frame pixels.
                    top = screen[1]
                    if top < frame.bgr.shape[0]*.25:
                        self.reset(frame.key.camera_id)
                        kinds = ('signal', 'left_boundary', 'right_boundary', 'path_clear', 'left_clear', 'right_clear')
                        unavailable = tuple(Observation(frame.key, kind, 'unknown', details=dict(reason='road_region_ambiguous'))
                                            for kind in kinds)
                        return Analysis((*unavailable, navigation), (), ())
                    road_frame = Frame(frame.key, frame.pts_us, frame.decoded_ns, frame.bgr[:top], frame.pts_kind)
                    navigation.details['road_bbox_xyxy'] = [0, 0, frame.bgr.shape[1], top]
            if self.parallel_perception and not skip_boundaries:
                with frame_tasks() as tasks:
                    detector_job = tasks.submit(self._detect, road_frame)
                    lane_job = tasks.submit(observe_boundaries, road_frame, allow_solid_extension=road_frame is frame,
                                            allow_dashed_extension=road_frame is frame, parallel=True)
                    # Preserve detector error/identity priority. Context exit
                    # joins the other task before any reset/next frame.
                    detections = detector_job.result()
            else:
                detections = self._detect(road_frame)
            signal = observe_signals(road_frame, detections, lane, required=scenario == 'intersection',
                                     tracker=self.signal_association)
            if self.lamp_heads and scenario == 'intersection':
                signal = refine_signal(road_frame, detections, lane, signal)
            # Top-origin cropping preserves overlay coordinates, NOT the
            # full-view horizon/near-road geometry used by the new extension.
            if skip_boundaries:
                # Explicit task subset, not a line detector. Unknown boundaries
                # cannot authorize a lane change; never publish fake clear lines.
                self.lane_consistency.reset(frame.key.camera_id)
                if self.sampled_lane_consistency is not None:self.sampled_lane_consistency.reset(frame.key.camera_id)
                boundaries = tuple(Observation(frame.key, side+'_boundary', 'unknown', fresh=False,
                                    details=dict(reason='intersection_fast_not_evaluated')) for side in ('left', 'right'))
            else:
                if self.parallel_perception:
                    # Raw tasks are already joined. Promote a lane exception
                    # only after signal processing, as in the serial path.
                    boundaries = lane_job.result()
                else:
                    boundaries = observe_boundaries(road_frame, allow_solid_extension=road_frame is frame,
                                                    allow_dashed_extension=road_frame is frame)
                tracker=(self.sampled_lane_consistency if self.sampled_lane_consistency is not None
                         and scenario=='lane' and road_frame is frame else self.lane_consistency)
                boundaries = tracker.filter(road_frame, boundaries,
                                            domain='full' if road_frame is frame else 'top_crop')
            persons = observe_pedestrians(road_frame, detections)
            observations = (signal, *boundaries, *persons)
            return Analysis((*observations, navigation) if navigation is not None else observations, detections, ())
        except Exception as exc:
            self.reset(frame.key.camera_id)
            # Backend/provider exceptions need not inherit RuntimeError. A failed
            # perception call is invalid input, never an empty/clear observation.
            errors = (f'{type(exc).__name__}: {exc}',)
            kinds = ('signal', 'left_boundary', 'right_boundary', 'path_clear', 'left_clear', 'right_clear')
            if scenario == 'navigation':
                kinds += ('navigation',)
            return Analysis(tuple(Observation(frame.key, kind, 'unknown', fresh=False) for kind in kinds), (), errors)
