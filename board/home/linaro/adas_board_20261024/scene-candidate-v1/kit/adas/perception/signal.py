"""Associate colored arrow/round symbols, never a neighboring green by x alone."""
from collections import Counter
from functools import lru_cache
import math

import cv2
import numpy as np

from ..contracts import Observation


def _normalize(mask):
    points = cv2.findNonZero(mask)
    if points is None:
        return None
    x, y, w, h = cv2.boundingRect(points)
    if w < 3 or h < 3:
        return None
    return cv2.resize(mask[y:y+h, x:x+w], (48, 48), interpolation=cv2.INTER_NEAREST) != 0


@lru_cache(maxsize=1)
def _templates():
    values = []
    for name, start, end in (('left', (51, 32), (13, 32)), ('right', (13, 32), (51, 32)),
                             ('straight', (32, 51), (32, 13))):
        for thickness in (3, 5, 7, 9, 11):
            for tip in (.35, .5, .65):
                mask = np.zeros((64, 64), np.uint8)
                cv2.arrowedLine(mask, start, end, 255, thickness, tipLength=tip)
                values.append((name, _normalize(mask)))
    for thickness in (-1, 3, 5, 7):
        mask = np.zeros((64, 64), np.uint8)
        cv2.circle(mask, (32, 32), 18, 255, thickness)
        values.append(('round', _normalize(mask)))
    return values


def _symbol(roi, color):
    if roi.size == 0 or color not in ('red', 'green', 'yellow'):
        return 'unknown', 0.
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    hue, saturation, value = cv2.split(hsv)
    hues = {'red': (hue < 13) | (hue > 166), 'green': (hue >= 35) & (hue <= 95),
            'yellow': (hue >= 15) & (hue < 35)}[color]
    mask = (hues & (saturation >= 85) & (value >= 85)).astype(np.uint8)*255
    # A dim arrow shaft may be separated from its head by 1–2 source pixels.
    # Close only inside the detected lamp; do not grow arbitrary scene colors.
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
    if count < 2:
        return 'unknown', 0.
    component = 1+int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    if stats[component, cv2.CC_STAT_AREA] < 8:
        return 'unknown', 0.
    normalized = _normalize((labels == component).astype(np.uint8)*255)
    if normalized is None:
        return 'unknown', 0.
    scores = {name: 0. for name in ('left', 'right', 'straight', 'round')}
    for name, template in _templates():
        score = np.logical_and(normalized, template).sum()/max(1, np.logical_or(normalized, template).sum())
        scores[name] = max(scores[name], float(score))
    ranked = sorted(scores, key=scores.get, reverse=True)
    best = scores[ranked[0]]
    return (ranked[0] if best >= .45 and best-scores[ranked[1]] >= .08 else 'unknown'), best


def _repair_prohibited_symbol(roi, color, baseline_score):
    """Recover dim edges near CURRENT strong pixels, only to strengthen STOP.

    Unrestricted weak-color flood fill absorbs the dark lamp housing and can
    turn an arrow into a circle. Growth is limited to two source-pixel steps;
    round/green and lower-confidence repairs are deliberately not accepted.
    """
    if roi.size == 0 or color not in ('red', 'yellow'):
        return None
    hue, saturation, value = cv2.split(cv2.cvtColor(roi, cv2.COLOR_BGR2HSV))
    hues = ((hue < 13) | (hue > 166)) if color == 'red' else ((hue >= 15) & (hue < 35))
    seed = (hues & (saturation >= 85) & (value >= 85)).astype(np.uint8)
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(seed*255, cv2.MORPH_CLOSE, kernel)
    weak = cv2.morphologyEx((hues & (saturation >= 45) & (value >= 45)).astype(np.uint8)*255,
                            cv2.MORPH_CLOSE, kernel)
    for _ in range(2):
        mask = cv2.dilate(mask, kernel) & weak
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
    if count < 2:
        return None
    seeds = np.bincount(labels.ravel(), weights=seed.ravel(), minlength=count)
    seeds[0] = 0
    component = int(seeds.argmax())
    if seeds[component] < 8:
        return None
    normalized = _normalize((labels == component).astype(np.uint8)*255)
    if normalized is None:
        return None
    scores = {name: 0. for name in ('left', 'right', 'straight', 'round')}
    for name, template in _templates():
        score = np.logical_and(normalized, template).sum()/max(1, np.logical_or(normalized, template).sum())
        scores[name] = max(scores[name], float(score))
    ranked = sorted(scores, key=scores.get, reverse=True)
    best = scores[ranked[0]]
    if ranked[0] == 'round' or best < max(.45, baseline_score) or best-scores[ranked[1]] < .08:
        return None
    return dict(symbol=ranked[0], score=best, strong_seed_pixels=int(seeds[component]),
                connected_pixels=int(stats[component, cv2.CC_STAT_AREA]), source_growth_steps=2,
                green_release_allowed=False)


def _repair_connected_red_symbol(roi, baseline_score):
    """Recover pink red-arrow edges anchored by CURRENT strict red evidence.

    This is a last-resort prohibited-direction repair, not a new color detector.
    It cannot remove raw UNKNOWN or its veto on a neighboring green signal.
    """
    if roi.size == 0:
        return None
    hue, saturation, value = cv2.split(cv2.cvtColor(roi, cv2.COLOR_BGR2HSV))
    seed = (((hue < 13) | (hue > 166)) & (saturation >= 85) & (value >= 85)).astype(np.uint8)
    weak = (((hue < 13) | (hue >= 145)) & (saturation >= 45) & (value >= 45)).astype(np.uint8)*255
    weak = cv2.morphologyEx(weak, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    count, labels, stats, _ = cv2.connectedComponentsWithStats(weak)
    seeds = np.bincount(labels.ravel(), weights=seed.ravel(), minlength=count)
    seeds[0] = 0
    anchored = np.flatnonzero(seeds >= 8)
    if len(anchored) != 1:
        return None
    component = int(anchored[0])
    x, y, width, height, area = (int(v) for v in stats[component])
    if x <= 0 or y <= 0 or x+width >= roi.shape[1] or y+height >= roi.shape[0]:
        return None  # A clipped support region is not a complete arrow.
    normalized = _normalize((labels == component).astype(np.uint8)*255)
    if normalized is None:
        return None
    scores = {name: 0. for name in ('left', 'right', 'straight', 'round')}
    for name, template in _templates():
        score = np.logical_and(normalized, template).sum()/max(1, np.logical_or(normalized, template).sum())
        scores[name] = max(scores[name], float(score))
    ranked = sorted(scores, key=scores.get, reverse=True)
    best = scores[ranked[0]]
    if ranked[0] == 'round' or best < max(.45, baseline_score) or best-scores[ranked[1]] < .08:
        return None
    return dict(symbol=ranked[0], score=best, strong_seed_pixels=int(seeds[component]),
                connected_pixels=area, support_bbox_xyxy=[x, y, x+width, y+height],
                source_growth_steps=None, support_mode='current_seed_connected_red', green_release_allowed=False)


class SignalAssociation:
    """Retain prohibited-symbol geometry briefly, NEVER an old color or green.

    Decoder PTS measures source evidence age, not optical latency. Missing boxes,
    colors, errors and discontinuities break the association. Stills are stateless.
    """
    def __init__(self, ttl_us=150000):
        if type(ttl_us) is not int or not 0 < ttl_us <= 150000:
            raise ValueError('symbol association TTL must be 1..150000 us')
        self.ttl_us = ttl_us
        self._cameras = {}

    def reset(self, camera=None):
        if camera is None:
            self._cameras.clear()
        else:
            self._cameras.pop(camera, None)

    @staticmethod
    def _iou(a, b):
        width = max(0., min(a[2], b[2])-max(a[0], b[0]))
        height = max(0., min(a[3], b[3])-max(a[1], b[1]))
        intersection = width*height
        union = (a[2]-a[0])*(a[3]-a[1])+(b[2]-b[0])*(b[3]-b[1])-intersection
        return intersection/max(1., union)

    def resolve(self, frame, lamps):
        camera, pts = frame.key.camera_id, frame.pts_us
        previous = self._cameras.pop(camera, None)
        if frame.pts_kind != 'decoder_reported' or type(pts) is not int or pts < 0:
            return
        shape = frame.bgr.shape[:2]
        consecutive = (previous is not None and previous['epoch'] == frame.key.epoch and
                       previous['frame_id']+1 == frame.key.frame_id and previous['shape'] == shape and
                       0 < pts-previous['pts'] <= self.ttl_us)
        history = previous['lamps'] if consecutive else []
        # Reciprocal uniqueness: an old lamp cannot donate geometry to two new
        # candidates. Include current classified/weak candidates as competitors,
        # not just those that would benefit from a hold.
        matches_by_lamp = [[i for i, old in enumerate(history) if old['color'] == lamp['color'] and
                            0 < pts-old['confirmed_pts'] <= self.ttl_us and
                            self._iou(old['bbox_xyxy'], lamp['bbox_xyxy']) >= .6] for lamp in lamps]
        donors = Counter(i for matches in matches_by_lamp for i in matches)
        current = []
        for lamp, matches in zip(lamps, matches_by_lamp):
            if lamp['color'] not in ('red', 'yellow') or lamp['symbol_score'] <= 0:
                continue
            confirmed_pts = pts
            if lamp['symbol'] == 'unknown':
                if len(matches) != 1 or donors[matches[0]] != 1:
                    continue
                old = history[matches[0]]
                lamp.update(symbol=old['symbol'], symbol_source='recent_same_color_geometry',
                            association_age_us=pts-old['confirmed_pts'])
                confirmed_pts = old['confirmed_pts']  # Held evidence cannot renew itself.
            current.append(dict(color=lamp['color'], bbox_xyxy=lamp['bbox_xyxy'],
                                symbol=lamp['symbol'], confirmed_pts=confirmed_pts))
        self._cameras[camera] = dict(epoch=frame.key.epoch, frame_id=frame.key.frame_id,
                                    shape=shape, pts=pts, lamps=current)


def observe_signals(frame, detections, lane, *, required=False, tracker=None):
    if lane not in ('left', 'straight', 'right'):
        raise ValueError('left/straight/right lane required')
    if any(d.key != frame.key for d in detections):
        raise ValueError('cross-frame detection')
    h, w = frame.bgr.shape[:2]
    lamps = [d for d in detections if d.label in ('red', 'green', 'yellow', 'black')
             and d.bbox_xyxy[1] < h*.7]
    details = []
    for lamp in lamps:
        x0, y0, x1, y1 = lamp.bbox_xyxy
        roi = frame.bgr[max(0, math.floor(y0)):min(h, math.ceil(y1)),
                        max(0, math.floor(x0)):min(w, math.ceil(x1))]
        symbol, score = _symbol(roi, lamp.label)
        item = dict(color=lamp.label, raw_symbol=symbol, symbol=symbol, symbol_score=score,
                    bbox_xyxy=list(lamp.bbox_xyxy), symbol_source='current_pixels', association_age_us=0)
        if symbol == 'unknown':
            repair = _repair_prohibited_symbol(roi, lamp.label, score)
            if repair is None and lamp.label == 'red':
                repair = _repair_connected_red_symbol(roi, score)
            if repair is not None:
                # Keep raw_symbol UNKNOWN: a recovered neighbor direction must
                # not remove the current unknown-red veto and enable green.
                item.update(symbol=repair['symbol'], symbol_source='current_color_reconstruction', repair=repair)
        details.append(item)
    if tracker is not None:
        tracker.resolve(frame, details)
    relevant = []
    for item in details:
        item['matched_lane'] = item['symbol'] in (lane, 'round')
        if item['matched_lane']:
            relevant.append(item['color'])
    colors = set(relevant)
    color = next(iter(colors)) if len(colors) == 1 and 'black' not in colors else 'unknown'
    unresolved_prohibition = any(item['color'] in ('red', 'yellow', 'black') and
                                 item['raw_symbol'] == 'unknown' for item in details)
    if color == 'green' and unresolved_prohibition:
        # History may strengthen a STOP, never remove a CURRENT unknown-red veto
        # and thereby enable green. Neighbor exclusion must be current pixels.
        color = 'unknown'
    return Observation(frame.key, 'signal', color, applicable=required or bool(lamps),
                       details=dict(lamps=details, association='color_symbol_not_x_order',
                                    unresolved_prohibition=unresolved_prohibition))
