"""Automatic light-theme navigation screen/card and dark-arrow geometry.

Pixel-only prototype: no paths, timestamps, GT tables, hand ROI or trained model.
Reads the prominent upcoming maneuver, not distance-triggered physical steering.
Unsupported themes/symbols/ambiguous instructions deliberately remain UNKNOWN.
"""
from functools import lru_cache
from copy import deepcopy
from dataclasses import replace
import math

import cv2
import numpy as np

from ..contracts import Observation


def validate_primary_region(region):
    """Optional operator geometry relative to an automatically found screen."""
    if (not isinstance(region, (list, tuple)) or len(region) != 4 or
            any(type(v) not in (int, float) or not math.isfinite(v) or not 0 <= v <= 1 for v in region) or
            not (region[0] < region[2] and region[1] < region[3])):
        raise ValueError('primary region requires normalized x0 y0 x1 y1 geometry')
    return tuple(float(v) for v in region)


def restrict_primary_region(observation, region):
    """Restrict current proposals, never invent a direction or real-road state.

    Screen geometry and applicability remain the original automatic results.
    Rejected proposals stay visible in diagnostics. The default path never calls
    this function, retaining support for arbitrarily translated prompt cards.
    """
    region = validate_primary_region(region)
    if observation.kind != 'navigation':
        raise ValueError('primary region applies only to navigation observations')
    details = deepcopy(observation.details)
    raw = details.get('candidates', [])
    accepted, rejected = [], []
    for candidate in raw:
        sx0, sy0, sx1, sy1 = candidate['screen_bbox_xyxy']
        x0, y0, x1, y1 = candidate['bbox_xyxy']
        if sx1 <= sx0 or sy1 <= sy0:
            raise ValueError('candidate screen rectangle must be nonempty')
        rx0, ry0, rx1, ry1 = region
        inside = (sx0 + rx0 * (sx1 - sx0) <= x0 < x1 <= sx0 + rx1 * (sx1 - sx0) and
                  sy0 + ry0 * (sy1 - sy0) <= y0 < y1 <= sy0 + ry1 * (sy1 - sy0))
        if inside:
            accepted.append(candidate)
        else:
            rejected.append(dict(candidate, reason='outside_operator_primary_prompt_geometry'))
    accepted.sort(key=lambda c: c['component_area'], reverse=True)
    prominent = [c for c in accepted if c['component_area'] >= accepted[0]['component_area'] * .55] if accepted else []
    values = {c['value'] for c in prominent}
    value = next(iter(values)) if len(values) == 1 and 'uturn' not in values else 'unknown'
    # Geometry is a veto, not extra positive perception. Removing a conflict
    # cannot manufacture a new direction or upgrade an original UNKNOWN.
    if value != observation.value:
        value = 'unknown'
    details.update(candidates=accepted, raw_candidates=raw,
                   raw_navigation_value=observation.value, direction_promotion_enabled=False,
                   operator_primary_region=list(region), primary_region_rejected=rejected,
                   primary_region_scope='operator_calibrated_geometry_not_a_direction_answer')
    return replace(observation, value=value, details=details)


def _normal(mask):
    points = cv2.findNonZero(mask)
    if points is None:
        return None
    x, y, w, h = cv2.boundingRect(points)
    if min(w, h) < 3:
        return None
    return cv2.resize(mask[y:y+h, x:x+w], (48, 48), interpolation=cv2.INTER_NEAREST) > 0


@lru_cache(maxsize=1)
def _templates():
    templates = []
    for thickness in (5, 7, 9, 11):
        for tip in (.35, .5, .65):
            for kind in ('left', 'right', 'straight', 'uturn'):
                for bend in (0, 1):
                    image = np.zeros((110, 110), np.uint8)
                    if kind == 'straight':
                        points = np.array([(50, 95), (50, 20)])
                    elif kind == 'uturn':
                        points = np.array([(25, 95), (25, 25), (70, 25), (70, 80)])
                    else:
                        points = np.array([(25, 95), (25, 45), (70, 45)] if not bend else
                                          [(25, 95), (25, 65), (42, 42), (70, 20)])
                        if kind == 'left':
                            points[:, 0] = 95-points[:, 0]
                    cv2.polylines(image, [points], False, 255, thickness)
                    cv2.arrowedLine(image, tuple(points[-2]), tuple(points[-1]), 255, thickness, tipLength=tip)
                    templates.append((kind, _normal(image)))
                    if kind in ('left', 'right') and not bend:
                        # Light-theme turn glyphs may include a thin incoming
                        # continuation behind the elbow. Represent both mirrored
                        # shapes instead of lowering the global match threshold.
                        elbow = tuple(points[1])
                        for length in (12, 20, 28):
                            incoming = image.copy()
                            tail = (elbow[0] + (length if kind == 'left' else -length), elbow[1])
                            cv2.line(incoming, tail, elbow, 255, max(1, thickness // 3))
                            templates.append((kind, _normal(incoming)))
                    if kind == 'uturn':
                        templates.append((kind, _normal(cv2.flip(image, 1))))
    # Digits are rejection competitors, not navigation templates/training data.
    for character in '0123456789':
        for thickness in (2, 3, 4):
            image = np.zeros((100, 100), np.uint8)
            cv2.putText(image, character, (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 2.5, 255, thickness)
            templates.append(('text', _normal(image)))
    return templates


def _symbol(mask):
    normalized = _normal(mask)
    if normalized is None:
        return 'unknown', 0., 0.
    scores = dict.fromkeys(('left', 'right', 'straight', 'uturn', 'text'), 0.)
    for kind, template in _templates():
        score = np.count_nonzero(normalized & template)/max(1, np.count_nonzero(normalized | template))
        scores[kind] = max(scores[kind], score)
    ranked = sorted(scores, key=scores.get, reverse=True)
    best, margin = scores[ranked[0]], scores[ranked[0]]-scores[ranked[1]]
    kind = ranked[0] if best >= .46 and margin >= .055 and ranked[0] != 'text' else 'unknown'
    return kind, float(best), float(margin)


def _screens(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    hue, saturation, value = cv2.split(hsv)
    mask = ((hue >= 70) & (hue <= 125) & (saturation >= 15) &
            (saturation <= 145) & (value >= 95)).astype(np.uint8)*255
    height, width = image.shape[:2]
    regions = []
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # Color alone also connects sky, buildings and windshield reflections.
    # First require an actual large, convex screen-shaped image boundary.
    edges = cv2.Canny(cv2.GaussianBlur(gray, (5, 5), 0), 35, 110)
    edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    for binary in (edges, (gray < 90).astype(np.uint8)*255):
        contours, _ = cv2.findContours(binary, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < .055*width*height:
                continue
            polygon = cv2.approxPolyDP(contour, .018*cv2.arcLength(contour, True), True)
            if len(polygon) != 4 or not cv2.isContourConvex(polygon):
                continue
            x, y, w, h = cv2.boundingRect(polygon)
            if (not 1.05 <= w/h <= 3.2 or area/(w*h) < .75 or
                    x <= 1 or y <= 1 or x+w >= width-1 or y+h >= height-1):
                continue
            inside = np.zeros_like(mask)
            cv2.fillConvexPoly(inside, polygon, 255)
            if np.count_nonzero(mask & inside)/max(1, np.count_nonzero(inside)) >= .45:
                regions.append((x, y, w, h))
    # A display may be cut by the camera edge. Color-based fallback must show
    # a dark hardware rim on both a horizontal and a vertical side.
    connected = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((13, 13), np.uint8))
    connected = cv2.morphologyEx(connected, cv2.MORPH_OPEN, np.ones((7, 7), np.uint8))
    _, _, stats, _ = cv2.connectedComponentsWithStats(connected)
    for x, y, w, h, area in stats[1:]:
        if area < .055*width*height or not 1.05 <= w/h <= 3.2 or area/(w*h) < .57:
            continue
        border = max(5, round(min(w, h)*.035))
        bands = [gray[max(0, y-border):y, x:x+w], gray[y+h:min(height, y+h+border), x:x+w],
                 gray[y:y+h, max(0, x-border):x], gray[y:y+h, x+w:min(width, x+w+border)]]
        dark = [band.size > 0 and float(np.mean(band < 90)) >= .45 for band in bands]
        if any(dark[:2]) and any(dark[2:]):
            regions.append((int(x), int(y), int(w), int(h)))
    # Multiple contours can describe the same physical screen.
    unique = []
    for region in sorted(regions, key=lambda r: r[2]*r[3], reverse=True):
        x, y, w, h = region
        if not any(max(0, min(x+w, ox+ow)-max(x, ox))*max(0, min(y+h, oy+oh)-max(y, oy)) /
                   max(1, min(w*h, ow*oh)) > .85 for ox, oy, ow, oh in unique):
            unique.append(region)
    return unique[:4]


def _distance_row(stats, component):
    """Find proportionate, horizontally aligned distance-character components.

    A rectangle alone is not reliable under reflection. Conversely, two arbitrary
    right-hand neighbors can be stacked texture, toolbar words or clipped edges.
    This is layout validation, NOT OCR or a physical turn-distance estimate.
    """
    x, y, w, h, area = (int(v) for v in stats[component])
    others = stats[1:]
    nx, ny, nw, nh, na = others.T
    # This same predicate used to convert all five integers of every other
    # component in Python for every glyph. Keep the exact bounds/order; only
    # materialize the few matching neighbors after the NumPy selection.
    eligible = ((np.arange(1, len(stats)) != component) &
                (x+w < nx) & (nx < x+w+4*h) &
                (y-.3*h < ny) & (ny < y+.65*h) &
                (.25*h <= nh) & (nh <= .85*h) &
                (.18 <= nw/np.maximum(1, nh)) & (nw/np.maximum(1, nh) <= 1.8) &
                (na >= max(3, .025*area)))
    possible = [tuple(map(int, stat[:4])) for stat in others[eligible]]
    possible.sort()
    for first in possible:
        fx, fy, fw, fh = first
        if fx > x+w+1.8*h:
            continue
        row = [first]
        for second in possible:
            nx, ny, nw, nh = second
            px, py, pw, ph = row[-1]
            if (0 <= nx-px-pw <= max(.8*min(ph, nh), .3*h) and
                    .7 <= nh/fh <= 1.4 and abs(ny+nh-fy-fh) <= max(2, .3*min(fh, nh))):
                row.append(second)
        if len(row) >= 2 and row[-1][0]+row[-1][2]-fx >= .6*h:
            return row
    return []


def _prompt_background(screen, box, row):
    """Check the automatically proposed prompt, without requiring closed edges.

    This prototype supports pale light-theme cards only. Ignore the darkest35%
    (glyphs/text) when estimating background spread; camera texture is not a card
    simply because its connected components have a plausible relative layout.
    """
    x, y, w, h = box
    top = min(y, *(ny for _, ny, _, _ in row))
    bottom = max(y+h, *(ny+nh for _, ny, _, nh in row))
    right = max(nx+nw for nx, _, nw, _ in row)
    x0, y0 = max(0, round(x-.25*h)), max(0, round(top-.25*h))
    x1, y1 = min(screen.shape[1], round(right+.25*h)), min(screen.shape[0], round(bottom+.25*h))
    patch = screen[y0:y1, x0:x1]
    gray = cv2.cvtColor(patch, cv2.COLOR_BGR2GRAY)
    hsv = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV)
    background = gray[gray >= np.percentile(gray, 35)]
    spread = float(np.std(background))
    value, saturation = float(np.median(hsv[:, :, 2])), float(np.median(hsv[:, :, 1]))
    if value < 140 or saturation > 125 or spread > 12:
        return None
    return dict(median_value=value, median_saturation=saturation, background_gray_std=spread)


def observe_navigation(frame, *, required=False, primary_region=None):
    if primary_region is not None:
        primary_region = validate_primary_region(primary_region)
    image = frame.bgr
    if (not isinstance(image, np.ndarray) or image.dtype != np.uint8 or image.ndim != 3 or
            image.shape[2] != 3 or min(image.shape[:2]) < 1):
        raise ValueError('navigation needs nonempty uint8 BGR frame')
    height, width = image.shape[:2]
    scale = min(1., 1280/width)
    small = cv2.resize(image, (max(1, round(width*scale)), max(1, round(height*scale))))
    regions = _screens(small)
    candidates = []
    for sx, sy, sw, sh in regions:
        roi = small[sy:sy+sh, sx:sx+sw]
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        # Reflection raises all channels together; fixed V<110 split real arrow
        # shafts from heads. Local contrast extracts both from the same card.
        dark = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                     cv2.THRESH_BINARY_INV, 31, 10)
        dark = cv2.morphologyEx(dark, cv2.MORPH_CLOSE, np.ones((2, 2), np.uint8))
        count, components, stats, _ = cv2.connectedComponentsWithStats(dark)
        for component in range(1, count):
            x, y, w, h, area = (int(v) for v in stats[component])
            if (x <= 0 or y <= 0 or x+w >= sw or y+h >= sh or
                    h < 8 or h > min(140, sh*.32) or not .25 <= w/h <= 1.5 or
                    area < 16 or area/(w*h) < .12 or area/(w*h) > .82):
                continue
            # A maneuver arrow has nearby distance/text on its right. A standalone
            # road arrow, map route or secondary icon is not sufficient evidence.
            row = _distance_row(stats, component)
            if not row:
                continue
            background = _prompt_background(roi, (x, y, w, h), row)
            if background is None:
                continue
            kind, score, margin = _symbol((components[y:y+h, x:x+w] == component).astype(np.uint8)*255)
            if kind == 'unknown':
                continue
            box = [round((sx+x)/scale), round((sy+y)/scale),
                   round((sx+x+w)/scale), round((sy+y+h)/scale)]
            candidates.append(dict(value=kind, score=score, margin=margin, bbox_xyxy=box,
                                   screen_bbox_xyxy=[round(sx/scale), round(sy/scale),
                                                     round((sx+sw)/scale), round((sy+sh)/scale)],
                                   component_area=area, distance_text_neighbors=len(row),
                                   distance_row_xyxy=[[round((sx+nx)/scale), round((sy+ny)/scale),
                                                      round((sx+nx+nw)/scale), round((sy+ny+nh)/scale)]
                                                     for nx, ny, nw, nh in row],
                                   context='aligned_distance_row_on_light_low_texture_prompt',
                                   prompt_background=background))
    # Prefer a clearly larger primary card arrow over small lane/afterward icons;
    # comparable conflicting arrows stay UNKNOWN rather than picking one by x.
    candidates.sort(key=lambda c: c['component_area'], reverse=True)
    prominent = [c for c in candidates if c['component_area'] >= candidates[0]['component_area']*.55] if candidates else []
    values = {c['value'] for c in prominent}
    value = next(iter(values)) if len(values) == 1 and 'uturn' not in values else 'unknown'
    primary = candidates[0] if candidates else None
    screen = primary['screen_bbox_xyxy'] if primary else (
        [round(regions[0][0]/scale), round(regions[0][1]/scale),
         round((regions[0][0]+regions[0][2])/scale), round((regions[0][1]+regions[0][3])/scale)]
        if regions else None)
    screen_area = ((screen[2]-screen[0])*(screen[3]-screen[1])/(width*height)) if screen else 0.
    observation = Observation(frame.key, 'navigation', value, applicable=required or bool(candidates),
                       details=dict(method='automatic_light_screen_dark_arrow_geometry',
                                    crop_origin='automatic_navigation_locator', candidates=candidates,
                                    screen_bbox_xyxy=screen, screen_area_fraction=screen_area,
                                    view='display_only' if screen_area >= .6 else 'road_with_display' if screen else 'unknown',
                                    supported=('left', 'straight', 'right'),
                                    instruction_scope='upcoming_hint_not_distance_triggered_steering'))
    return observation if primary_region is None else restrict_primary_region(observation, primary_region)
