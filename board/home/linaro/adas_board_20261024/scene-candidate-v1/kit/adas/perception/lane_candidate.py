"""Pixel proposals with a separate complete-block DASHED certificate.

Raw ray-based DASHED values remain diagnostic, not driving permission. The
production adapter may explicitly opt in to the stronger centre-chain result.
"""
import cv2
import numpy as np

from ..contracts import Observation
from .segments import boolean_runs as _runs


def _paint_components(image, *, include_contrast=False, hsv=None):
    """Keep narrow locally bright paint, not all bright asphalt/car panels."""
    h, w = image.shape[:2]
    if hsv is None:
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    hue, saturation, value = cv2.split(hsv)
    white = (value >= 165) & (saturation <= 90)
    yellow = (hue >= 15) & (hue <= 38) & (saturation >= 90) & (value >= 100)
    gray = cv2.GaussianBlur(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), (5, 5), 0)
    background = cv2.morphologyEx(gray, cv2.MORPH_OPEN, np.ones((1, max(15, round(w*.04))), np.uint8))
    contrast = cv2.subtract(gray, background)
    # A broad car panel is not a short dash. Thin connected paint may be either
    # one long solid stripe or one of several genuinely separate dash blocks.
    output, components = np.zeros((h, w), np.int32), {}
    for color, colored in (('white', white & ~yellow & (contrast >= 18)),
                           ('yellow', yellow & (contrast >= 10))):
        mask = colored.astype(np.uint8)
        mask[:int(.52*h)] = 0
        mask[int(.94*h):] = 0
        count, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
        for component in range(1, count):
            if stats[component, cv2.CC_STAT_AREA] < 5:
                continue
            x, y, cw, ch = stats[component, :4]
            # Any rotated-rectangle side is at most the point-set diameter,
            # itself at most this integer Manhattan span. Strictly less only:
            # equality and diagonal paint still go through the original fit.
            if int(cw)+int(ch)-2 < max(5, h*.005):
                continue
            selected = labels[y:y+ch, x:x+cw] == component
            points = cv2.findNonZero(selected.astype(np.uint8))
            short, long = sorted(cv2.minAreaRect(points)[1])
            if long >= max(5, h*.005) and long/max(1., short) >= 1.6 and short <= max(5, w*.03):
                index = len(components)+1
                vx, vy = cv2.fitLine(points, cv2.DIST_L2, 0, .01, .01).ravel()[:2]
                components[index] = (color, float(vx), float(vy), long)
                output[y:y+ch, x:x+cw][selected] = index
    return (output, components, contrast) if include_contrast else (output, components)


def _prepare_profile(components, height):
    """One ID table per frame; zero is background, never compatible paint."""
    vx = np.zeros(len(components)+1, np.float64)
    vy = np.zeros_like(vx)
    alignment = np.ones_like(vx)
    colors = {color: np.zeros(len(vx), bool) for color in ('white', 'yellow')}
    for index, (paint, x, y, length) in components.items():
        vx[index], vy[index] = x, y
        # Preserve the original long/short alignment thresholds exactly.
        alignment[index] = .97 if length >= height*.08 else .90
        colors[paint][index] = True
    return vx, vy, alignment, colors


def _compatible_profile(prepared, slope, color):
    vx, vy, alignment, colors = prepared
    return colors[color] & (np.abs(slope*vx+vy)/np.hypot(slope, 1.) >= alignment)


def _sample_ray(labels, slope, intercept):
    h, w = labels.shape
    ys = np.arange(round(h*.55), round(h*.94))
    xs = np.rint(slope*ys+intercept).astype(int)
    radius = max(2, round(w*.0025))
    columns = xs[:, None]+np.arange(-radius, radius+1)
    valid = (columns >= 0) & (columns < w)
    sampled_ids = labels[ys[:, None], columns.clip(0, w-1)]
    return ys, xs, sampled_ids, valid


def _profile(labels, components, slope, intercept, color, *, prepared=None, sampling=None):
    h, w = labels.shape
    ys, xs, sampled_ids, valid = _sample_ray(labels, slope, intercept) if sampling is None else sampling
    compatible = _compatible_profile(_prepare_profile(components, h) if prepared is None else prepared, slope, color)
    hits = compatible[sampled_ids] & valid
    sampled = np.any(hits, axis=1)
    # Close tiny pixel holes only, not the gaps that define a dashed boundary.
    offset = 0
    for on, length in _runs(sampled):
        if not on and offset and offset+length < len(sampled) and length <= max(1, round(h*.003)):
            sampled[offset:offset+length] = True
        offset += length
    offset = 0
    for on, length in _runs(sampled):
        if on and length < max(3, round(h*.006)):
            sampled[offset:offset+length] = False
        offset += length
    visible = np.flatnonzero(sampled)
    if not len(visible):
        return None
    start, end = int(visible[0]), int(visible[-1])+1
    if ys[end-1] < .85*h:
        return None  # No near-road support: car/fence fragments alone are not a boundary.
    runs = _runs(sampled[start:end])
    span, coverage = end-start, float(sampled[start:end].mean())
    on_runs = [length for on, length in runs if on]
    off_runs = [length for on, length in runs if not on and length >= max(4, h*.008)]
    block_ids, position = [], start
    for on, length in runs:
        if on:
            identifiers = sampled_ids[position:position+length][hits[position:position+length]]
            counts = np.bincount(identifiers, minlength=len(components)+1)
            block_ids.append(int(np.argmax(counts)))
        position += length
    gaps = [length for on, length in runs if not on]
    # A ray grazing one solid stripe can produce many ON/OFF runs. Permission
    # needs distinct paint blocks and plausible perspective progression, not
    # scattered vehicle/fence fragments or one huge occlusion between hits.
    distinct_blocks = len(block_ids) >= 3 and len(set(block_ids)) == len(block_ids)
    progressing = (all(b >= a*.65 for a, b in zip(on_runs, on_runs[1:])) and
                   all(b >= a*.65 for a, b in zip(gaps, gaps[1:])) and
                   all(gap <= 4*max(on_runs[i:i+2]) for i, gap in enumerate(gaps)))
    classification = 'unknown'
    raw_value = 'unknown'
    if on_runs[-1] >= h*.15 or (span >= h*.15 and coverage >= .88 and not off_runs):
        classification = raw_value = 'solid'
    elif span >= h*.12 and len(on_runs) >= 3 and len(off_runs) >= 2 and .15 <= coverage <= .8:
        raw_value = 'dashed'
        if distinct_blocks and progressing:
            classification = 'dashed'
    return dict(value=classification, raw_value=raw_value, paint_color=color, coverage=coverage, runs=runs, span_px=span,
                points=[[int(xs[start]), int(ys[start])], [int(xs[end-1]), int(ys[end-1])]],
                support_pixels=int(sampled.sum()), block_ids=block_ids,
                distinct_blocks=distinct_blocks, perspective_progression=progressing)


def _dash_chains(labels, components, contrast):
    """Fit centres of complete paint blocks, not tangents on a single block.

    This is a separate certificate; old ray-based DASHED proposals do not earn
    it. Four disjoint, growing blocks and gaps must agree on one boundary.
    """
    h, w = labels.shape
    white = np.zeros(len(components)+1, np.uint8)
    for index, (color, *_rest) in components.items():
        white[index] = color == 'white'
    count, block_labels, stats, centers = cv2.connectedComponentsWithStats(white[labels])
    blocks, competitors = [], []
    result = {'left': [], 'right': []}
    for index in range(1, count):
        x, y, bw, bh, area = (int(v) for v in stats[index])
        cx, cy = (float(v) for v in centers[index])
        if not .62*h <= cy <= .92*h or area < 12:
            continue  # The ROI must not manufacture an endpoint in a long stripe.
        local = block_labels[y:y+bh, x:x+bw] == index
        points = cv2.findNonZero(local.astype(np.uint8))
        short, long = sorted(cv2.minAreaRect(points)[1])
        # Perspective shortens far blocks along the road. Reuse the paint
        # component elongation floor; the whole chain supplies the evidence.
        if (long < h*.008 or short < 1. or long/short < 1.6 or
                area/max(1., short*long) < .55):
            continue
        vx, vy = cv2.fitLine(points, cv2.DIST_L2, 0, .01, .01).ravel()[:2]
        if abs(vy) < .3:
            continue
        block = dict(id=index, center=[cx, cy], bbox=[x, y, x+bw, y+bh],
                     width=float(short), length=float(long), area=area,
                     axis=[float(vx), float(vy)])
        # Unclassified continuous paint is still boundary evidence. Do not
        # throw it away just because it is too long/clipped to be a dash block.
        if bh >= h*.04:
            competitors.append(block)
        if long > h*.09 or y+bh >= int(.94*h)-1:
            continue
        # Cull only centres unreachable by ANY later-admissible hypothesis.
        # At .62h all such rays are in [.15,.85]w; at .90h they are in
        # [.04,.96]w. Their affine outer envelope includes both sides, with
        # the SAME centre-fit tolerance used below. Beyond .90h the top
        # coefficient is negative, so reverse its extrema for extrapolation.
        # Keep competitors above: an unresolved near solid must still veto.
        t=(cy-.62*h)/(.28*h)
        low=((1-t)*(.15 if t<=1 else .85)+t*.04)*w
        high=((1-t)*(.85 if t<=1 else .15)+t*.96)*w
        tolerance=max(3.,w*.003)
        if not low-tolerance<=cx<=high+tolerance:
            continue
        blocks.append(block)
        if len(blocks) > 64:
            # Bound quadratic hypotheses, without selecting an arbitrary first
            # subset that could hide a nearer competing boundary.
            return result, 'candidate_overload'
    if len(blocks) < 4:
        return result, 'insufficient_complete_blocks'
    xy = np.asarray([b['center'] for b in blocks])
    axes = np.asarray([b['axis'] for b in blocks])
    seen = set()
    for i, first in enumerate(blocks):
        for j in range(i+1, len(blocks)):
            dx, dy = xy[j]-xy[i]
            if abs(dy) < .04*h:
                continue
            slope = dx/dy
            if not .12 <= abs(slope) <= 4.:
                continue
            intercept = xy[i, 0]-slope*xy[i, 1]
            bottom, top = slope*.90*h+intercept, slope*.62*h+intercept
            side = ('left' if slope < 0 and .04*w < bottom < .5*w else
                    'right' if slope > 0 and .5*w < bottom < .96*w else None)
            if side is None or not .15*w <= top <= .85*w:
                continue
            aligned = np.abs(slope*axes[:, 0]+axes[:, 1])/np.hypot(slope, 1.) >= .9
            close = np.abs(xy[:, 0]-slope*xy[:, 1]-intercept) <= max(3., w*.003)
            indices = sorted(np.flatnonzero(aligned & close), key=lambda k: xy[k, 1])
            for start in range(len(indices)-3):
                ids = tuple(indices[start:start+4])
                if ids in seen:
                    continue
                seen.add(ids)
                chain = [blocks[k] for k in ids]
                points = xy[list(ids)]
                if points[-1, 1] < .82*h or np.ptp(points[:, 1]) < .07*h:
                    continue
                lengths = np.array([b['length'] for b in chain])
                widths = np.array([b['width'] for b in chain])
                areas = np.array([b['area'] for b in chain])
                intervals = np.diff(points[:, 1])
                gaps = np.array([b['bbox'][1]-a['bbox'][3] for a, b in zip(chain, chain[1:])])
                if (np.any(gaps < max(3., h*.004)) or np.any(gaps[1:] < gaps[:-1]*.75) or
                        np.any(lengths[1:] < lengths[:-1]*.75) or np.any(areas[1:] < areas[:-1]*.65) or
                        np.any(intervals[1:] < intervals[:-1]*.75) or
                        lengths[-1] < lengths[0]*1.5 or widths[-1] < widths[0]*1.35 or
                        intervals[-1] < intervals[0]*1.25):
                    continue
                a, b = np.polyfit(points[:, 1], points[:, 0], 1)
                error = float(np.max(np.abs(points[:, 0]-a*points[:, 1]-b)))
                if error > max(2., w*.0015):
                    continue
                # Absolute white thresholding cuts even intact paint in shade.
                # Relative stripe contrast in the supposed gaps is a veto, not
                # proof that a dark region is empty road.
                shadowed_paint = False
                for far, near in zip(chain, chain[1:]):
                    gy = np.arange(far['bbox'][3]+2, near['bbox'][1]-2)
                    if len(gy) < 3:
                        continue
                    gx = np.rint(a*gy+b).astype(int)
                    radius = max(1, round(min(far['width'], near['width'])/4))
                    columns = (gx[:, None]+np.arange(-radius, radius+1)).clip(0, w-1)
                    stripe = np.median(contrast[gy[:, None], columns], axis=1)
                    if np.mean(stripe >= 8) >= .6:
                        shadowed_paint = True
                        break
                if shadowed_paint:
                    continue
                chain_bottom = a*.90*h+b
                nearer_boundary = False
                member_ids = {block['id'] for block in chain}
                for competitor in competitors:
                    if competitor['id'] in member_ids:
                        continue
                    x0, y0, x1, y1 = competitor['bbox']
                    if min(y1, chain[-1]['bbox'][3])-max(y0, chain[0]['bbox'][1]) < .02*h:
                        continue
                    vx, vy = competitor['axis']
                    cm = vx/vy
                    if not .12 <= abs(cm) <= 4. or (cm < 0) != (side == 'left'):
                        continue
                    cx, cy = competitor['center']
                    cb = cx-cm*cy
                    competitor_bottom = cm*.90*h+cb
                    nearer = (chain_bottom+w*.01 < competitor_bottom < .5*w if side == 'left' else
                              .5*w < competitor_bottom < chain_bottom-w*.01)
                    if nearer:
                        nearer_boundary = True
                        break
                if nearer_boundary:
                    continue
                result[side].append(dict(blocks=chain, center_fit_x_from_y=[float(a), float(b)],
                                         max_center_error_px=error, gaps_px=gaps.tolist(),
                                         relative_gap_paint_veto_passed=True,
                                         nearer_unresolved_paint_veto_passed=True,
                                         points=[[round(a*chain[0]['bbox'][1]+b), chain[0]['bbox'][1]],
                                                 [round(a*chain[-1]['bbox'][3]+b), chain[-1]['bbox'][3]]]))
    return result, 'evaluated'


def observe_boundaries(frame, *, hsv=None):
    image = frame.bgr
    h, w = image.shape[:2]
    labels, components, contrast = _paint_components(image, include_contrast=True, hsv=hsv)
    prepared = _prepare_profile(components, h)
    dash_chains, dash_chain_status = _dash_chains(labels, components, contrast)
    mask = (labels > 0).astype(np.uint8)*255
    lines = cv2.HoughLinesP(mask, 1, np.pi/180, threshold=max(10, h//60),
                           minLineLength=max(6, h*.01), maxLineGap=max(2, h*.007))
    candidates = {'left': [], 'right': []}
    if lines is not None:
        for x0, y0, x1, y1 in lines[:, 0]:
            if abs(int(y1)-int(y0)) < max(4, h*.008):
                continue
            slope = (float(x1)-x0)/(float(y1)-y0)
            if not .05 <= abs(slope) <= 4.:
                continue
            intercept = x0-slope*y0
            bottom = slope*(.90*h)+intercept
            top = slope*(.62*h)+intercept
            if not .15*w <= top <= .85*w:
                continue
            if .02*w < bottom < .5*w and slope < 0:
                candidates['left'].append((slope, intercept, bottom))
            elif .5*w < bottom < .98*w and slope > 0:
                candidates['right'].append((slope, intercept, bottom))
    observations = []
    for side in ('left', 'right'):
        group = candidates[side]
        details = dict(method='local_paint_segment_visible_span', coverage=0., runs=[], points=[],
                       candidate_count=len(group), classified_candidates=0, raw_dashed_proposals=0,
                       permission_source=False, dash_chain_certified=False, dash_chain_status=dash_chain_status)
        classification = 'unknown'
        if group:
            credible, seen = [], set()
            for slope, intercept, bottom in group:
                signature = (round((slope*.62*h+intercept)/max(1, w*.005)), round(bottom/max(1, w*.005)))
                if signature in seen:
                    continue
                seen.add(signature)
                sampling = _sample_ray(labels, slope, intercept)
                for color in ('white', 'yellow'):
                    profile = _profile(labels, components, slope, intercept, color, prepared=prepared, sampling=sampling)
                    if profile is not None and profile['raw_value'] == 'dashed':
                        details['raw_dashed_proposals'] += 1
                    if profile is not None and profile['value'] != 'unknown':
                        credible.append((bottom, profile))
            details['classified_candidates'] = len(credible)
            if credible:
                # Keep a real supported line, never independently median slope
                # and intercept from different objects into a fictitious ray.
                bottom, profile = (max(credible, key=lambda c: c[0]) if side == 'left' else
                                   min(credible, key=lambda c: c[0]))
                classification = profile['value']
                details.update({k: v for k, v in profile.items() if k != 'value'})
        if dash_chains[side]:
            # A certificate is still diagnostic until the production adapter
            # explicitly opts in after semantic/negative and control validation.
            def boundary_rank(chain):
                slope, intercept = chain['center_fit_x_from_y']
                bottom = slope*.90*h+intercept
                return ((-bottom if side == 'left' else bottom), chain['max_center_error_px'])
            chain = min(dash_chains[side], key=boundary_rank)
            details.update(dash_chain_certified=True, dash_chain=chain)
            if classification == 'unknown':
                classification = 'dashed'
                details['points'] = chain['points']
        observations.append(Observation(frame.key, side+'_boundary', classification, details=details))
    return tuple(observations)
