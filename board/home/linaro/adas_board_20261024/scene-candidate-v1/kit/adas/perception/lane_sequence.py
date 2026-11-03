"""Consecutive CURRENT boundary evidence, never a hold of old permission.

Decoder PTS orders offline source frames; it is not an optical/real-time clock.
Explicit still images retain spatial behavior and cannot seed a video streak.
"""
import math

from ..contracts import Observation


class LaneConsistency:
    def __init__(self):
        self._cameras = {}

    def reset(self, camera=None):
        if camera is None:
            self._cameras.clear()
        else:
            self._cameras.pop(camera, None)

    @staticmethod
    def _geometry(observation, shape):
        points = observation.details.get('points')
        try:
            if len(points) != 2 or any(len(point) != 2 for point in points):
                return None
            (x0, y0), (x1, y1) = ([float(v) for v in point] for point in points)
        except (TypeError, ValueError):
            return None
        h, w = shape
        if (not all(math.isfinite(v) for v in (x0, y0, x1, y1)) or
                not (0 <= x0 < w and 0 <= x1 < w and 0 <= y0 < h and 0 <= y1 < h) or
                abs(y1-y0) < max(3, h*.02)):
            return None
        slope = (x1-x0)/(y1-y0)
        return (slope, x0-slope*y0, min(y0, y1), max(y0, y1))

    @staticmethod
    def _same_boundary(first, second, shape):
        a, b, y0, y1 = first
        c, d, z0, z1 = second
        h, w = shape
        start, end = max(y0, z0), min(y1, z1)
        if end-start < max(3, h*.02):
            return False
        alignment = (a*c+1)/(math.hypot(a, 1)*math.hypot(c, 1))
        return (alignment >= .98 and
                max(abs((a-c)*start+b-d), abs((a-c)*end+b-d)) <= w*.025)

    def filter(self, frame, boundaries, *, domain='full'):
        if (domain not in ('full', 'top_crop') or len(boundaries) != 2 or
                [o.kind for o in boundaries] != ['left_boundary', 'right_boundary'] or
                any(o.key != frame.key for o in boundaries)):
            self.reset(frame.key.camera_id)
            raise ValueError('mismatched boundary identity or road domain')
        camera, pts, shape = frame.key.camera_id, frame.pts_us, frame.bgr.shape[:2]
        previous = self._cameras.pop(camera, None)
        still = frame.pts_kind == 'not_applicable_image' and pts is None
        timed = frame.pts_kind == 'decoder_reported' and type(pts) is int and pts >= 0
        consecutive = (timed and previous is not None and previous['epoch'] == frame.key.epoch and
                       previous['frame_id']+1 == frame.key.frame_id and previous['shape'] == shape and
                       previous['domain'] == domain and 0 < pts-previous['pts'] <= 150000)
        history = previous['sides'] if consecutive else {}
        current, output = {}, []
        for observed in boundaries:
            value, count, span = observed.value, 0, None
            status = 'current_non_dashed'
            if still:
                status = 'still_stateless'
            elif observed.value == 'dashed':
                value = 'unknown'
                status = 'no_video_timestamp' if not timed else 'current_unavailable'
                if timed and observed.fresh is True and observed.applicable is True:
                    geometry = self._geometry(observed, shape)
                    status = 'geometry_unknown'
                    if geometry is not None:
                        older = history.get(observed.kind, [])[-2:]
                        # Compare ALL retained evidence, not just a pairwise
                        # walk that could drift into another adjacent boundary.
                        if not all(self._same_boundary(p['geometry'], geometry, shape) for p in older):
                            older = []
                        evidence = [*older, dict(geometry=geometry, pts=pts)][-3:]
                        current[observed.kind] = evidence
                        count = len(evidence)
                        span = pts-evidence[0]['pts']
                        status = 'confirmed' if count == 3 else 'pending'
                        if count == 3:
                            value = 'dashed'
            temporal = dict(raw_value=observed.value, status=status, count=count, required=3,
                            source_span_us=span, source_clock='decoder_PTS_not_physical_latency',
                            historical_value_substitution=False)
            output.append(Observation(observed.key, observed.kind, value, observed.applicable, observed.fresh,
                                      dict(observed.details, temporal=temporal)))
        if timed:
            self._cameras[camera] = dict(epoch=frame.key.epoch, frame_id=frame.key.frame_id,
                                        pts=pts, shape=shape, domain=domain, sides=current)
        return tuple(output)
