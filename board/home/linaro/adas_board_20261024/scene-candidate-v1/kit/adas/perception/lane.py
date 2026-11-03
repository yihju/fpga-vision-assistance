"""Stable baseline with conservative SOLID and opt-in complete DASHED chains."""
import cv2
import numpy as np

from ..contracts import Observation
from ..frame_tasks import frame_tasks
from . import lane_candidate
from .segments import boolean_runs as _runs


def _baseline(frame, *, hsv=None):
    image = frame.bgr
    h, w = image.shape[:2]
    if hsv is None:
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    hue, saturation, value = cv2.split(hsv)
    white = (value >= 165) & (saturation <= 90)
    yellow = (hue >= 15) & (hue <= 38) & (saturation >= 90) & (value >= 100)
    mask = ((white | yellow).astype(np.uint8))*255
    mask[:int(.52*h)] = 0
    mask[int(.97*h):] = 0
    lines = cv2.HoughLinesP(mask, 1, np.pi/180, threshold=max(12, h//18),
                           minLineLength=max(12, h*.06), maxLineGap=max(3, h*.025))
    candidates = {'left': [], 'right': []}
    if lines is not None:
        for x0, y0, x1, y1 in lines[:, 0]:
            if abs(int(y1)-int(y0)) < h*.04:
                continue
            slope = (float(x1)-x0)/(float(y1)-y0)
            if not .08 <= abs(slope) <= 2.:
                continue
            intercept = x0-slope*y0
            bottom = slope*(.95*h)+intercept
            if .02*w < bottom < .5*w and slope < 0:
                candidates['left'].append((slope, intercept, bottom))
            elif .5*w < bottom < .98*w and slope > 0:
                candidates['right'].append((slope, intercept, bottom))
    observations = []
    for side in ('left', 'right'):
        group = candidates[side]
        details = dict(method='color_hough_continuity', coverage=0., runs=[], points=[])
        classification = 'unknown'
        if group:
            base = max(group, key=lambda g: g[2]) if side == 'left' else min(group, key=lambda g: g[2])
            near = np.asarray([line for line in group if abs(line[2]-base[2]) <= w*.04])
            slope, intercept = np.median(near[:, :2], axis=0)
            ys = np.arange(round(h*.55), round(h*.95))
            xs = np.rint(slope*ys+intercept).astype(int)
            radius = max(2, round(w*.004))
            sampled = np.array([np.count_nonzero(mask[y, max(0, x-radius):min(w, x+radius+1)]) > radius
                                for y, x in zip(ys, xs)], dtype=bool)
            runs = _runs(sampled)
            on_runs = [length for on, length in runs if on and length >= max(4, h*.025)]
            off_runs = [length for on, length in runs[1:-1] if not on and length >= max(4, h*.025)]
            coverage = float(sampled.mean())
            if coverage >= .85 and not off_runs:
                classification = 'solid'
            elif len(on_runs) >= 3 and len(off_runs) >= 2 and .2 <= coverage <= .8:
                classification = 'dashed'
            details.update(coverage=coverage, runs=runs,
                           points=[[int(xs[0]), int(ys[0])], [int(xs[-1]), int(ys[-1])]])
        observations.append(Observation(frame.key, side+'_boundary', classification, details=details))
    return tuple(observations)


def observe_boundaries(frame, *, allow_solid_extension=True, allow_dashed_extension=False, parallel=False):
    hsv = cv2.cvtColor(frame.bgr, cv2.COLOR_BGR2HSV)
    if parallel and allow_solid_extension:
        with frame_tasks() as tasks:
            candidate = tasks.submit(lane_candidate.observe_boundaries, frame, hsv=hsv)
            baseline = _baseline(frame, hsv=hsv)
            if all(o.value != 'unknown' for o in baseline):
                # The original path never needed candidate on this branch.
                # Join it, but do not promote an unused exception to failure.
                return baseline
            proposed = candidate.result()
    else:
        baseline = _baseline(frame, hsv=hsv)
        if not allow_solid_extension:
            return tuple(Observation(o.key, o.kind, o.value, o.applicable, o.fresh,
                                     dict(o.details, solid_extension_disabled_reason='uncalibrated_cropped_road',
                                          new_dashed_permission_enabled=False)) for o in baseline)
        if all(o.value != 'unknown' for o in baseline):
            return baseline
        proposed = lane_candidate.observe_boundaries(frame, hsv=hsv)
    if (len(proposed) != 2 or any(old.key != new.key or old.kind != new.kind
                                  for old, new in zip(baseline, proposed))):
        raise ValueError('cross-frame or mismatched lane proposal')
    output = []
    for old, new in zip(baseline, proposed):
        eligible = old.value == 'unknown' and new.fresh and new.applicable
        adopted = eligible and new.value == 'solid'
        chain = new.details.get('dash_chain')
        complete_chain = (new.details.get('dash_chain_certified') is True and isinstance(chain, dict)
                          and isinstance(chain.get('blocks'), list) and len(chain['blocks']) >= 4)
        dashed_adopted = (allow_dashed_extension and eligible and new.value == 'dashed' and complete_chain)
        selected = new if adopted or dashed_adopted else old
        details = dict(selected.details, baseline_value=old.value, diagnostic_candidate=new.value,
                       solid_extension_adopted=adopted, new_dashed_permission_enabled=dashed_adopted,
                       dashed_extension_adopted=dashed_adopted,
                       raw_dashed_proposals=new.details.get('raw_dashed_proposals', 0))
        if dashed_adopted:
            details.update(method='complete_paint_block_center_chain', points=chain['points'])
        output.append(Observation(selected.key, selected.kind, selected.value, selected.applicable,
                                  selected.fresh, details))
    return tuple(output)
