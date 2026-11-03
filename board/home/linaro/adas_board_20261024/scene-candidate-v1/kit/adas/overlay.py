"""Frame-bound visual actions. No pixels are written back into source images."""
import cv2

from .media import fit_image
from .status import perception_status
from .direction_indicators import draw_direction_indicators


def render(frame, decision, observations, detections=(), *, size=(1280, 720), indicator_clock_ns=None):
    if (decision.key != frame.key or any(o.key != frame.key for o in observations)
            or any(d.key != frame.key for d in detections)):
        raise ValueError('refuse to overlay stale/cross-frame result')
    if size[0] < 640 or size[1] < 240:
        raise ValueError('status panel requires at least 640x240')
    # Reserve a panel rather than covering traffic lights in the source image.
    road, transform = fit_image(frame.bgr, (size[0], size[1]-138))
    import numpy as np
    canvas = np.full((size[1], size[0], 3), 18, np.uint8)
    canvas[138:] = road
    transform['top'] += 138
    sx, sy, left, top = (transform[k] for k in ('scale_x', 'scale_y', 'left', 'top'))
    calibrated = [region for o in observations if o.kind == 'signal'
                  for region in o.details.get('reference_profile', {}).get('regions', [])]
    for detection in detections:
        if calibrated and detection.label in ('red','green','yellow','black'):
            continue  # Keep NN outputs in logs; show only the calibrated lamp source here.
        box = detection.bbox_xyxy
        p1 = (round(box[0]*sx+left), round(box[1]*sy+top))
        p2 = (round(box[2]*sx+left), round(box[3]*sy+top))
        color = (0, 0, 255) if detection.label == 'red' else (0, 220, 0)
        cv2.rectangle(canvas, p1, p2, color, 2)
        cv2.putText(canvas, f'{detection.label} {detection.confidence:.2f}', (p1[0], max(16, p1[1]-5)),
                    cv2.FONT_HERSHEY_SIMPLEX, .5, color, 1, cv2.LINE_AA)
    geometric = [head for o in observations if o.kind == 'signal'
                 for head in o.details.get('lamp_head_geometry', {}).get('heads', [])]
    for region in calibrated:
        box = region['bbox_xyxy']
        p1 = (round(box[0]*sx+left), round(box[1]*sy+top))
        p2 = (round(box[2]*sx+left), round(box[3]*sy+top))
        cv2.rectangle(canvas,p1,p2,(255,210,0),2)
        cv2.putText(canvas,'ROI '+region['color'].upper(),(p1[0],max(154,p1[1]-5)),
                    cv2.FONT_HERSHEY_SIMPLEX,.43,(255,210,0),1,cv2.LINE_AA)
    geometry_labels = []
    for head in geometric:
        box = head['bbox_xyxy']
        p1 = (round(box[0]*sx+left), round(box[1]*sy+top))
        p2 = (round(box[2]*sx+left), round(box[3]*sy+top))
        cv2.rectangle(canvas, p1, p2, (0, 190, 255), 2)
        label = 'GEOM '+head['color'].upper()
        (tw, th), baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, .43, 1)
        tx = max(0, min(p1[0], size[0]-tw-2))
        ty = max(154, p1[1]-5)
        while ty+baseline < size[1]:
            rectangle = (tx, ty-th, tx+tw, ty+baseline)
            if not any(rectangle[0] <= r[2]+2 and r[0] <= rectangle[2]+2 and
                       rectangle[1] <= r[3]+2 and r[1] <= rectangle[3]+2 for r in geometry_labels):
                geometry_labels.append(rectangle)
                cv2.putText(canvas, label, (tx, ty), cv2.FONT_HERSHEY_SIMPLEX,
                            .43, (0, 190, 255), 1, cv2.LINE_AA)
                break
            ty += th+baseline+4
        # If too crowded, retain every box and the panel's geometry provenance,
        # rather than drawing clipped/overlapping labels over UNKNOWN status.
    navigation = next((o for o in observations if o.kind == 'navigation'), None)
    if navigation is not None:
        # Other components are raw diagnostics, not accepted route instructions.
        # Drawing them all as "route hint" would contradict the selected state.
        candidates = navigation.details.get('candidates', [])
        selected = candidates[:1] if navigation.value != 'unknown' else []
        for candidate in selected:
            if candidate['value'] != navigation.value:
                continue
            x0, y0, x1, y1 = candidate['bbox_xyxy']
            p1 = (round(x0*sx+left), round(y0*sy+top))
            p2 = (round(x1*sx+left), round(y1*sy+top))
            cv2.rectangle(canvas, p1, p2, (255, 220, 0), 2)
            cv2.putText(canvas, 'route hint: '+candidate['value'].upper(),
                        (p1[0], max(154, p1[1]-7)), cv2.FONT_HERSHEY_SIMPLEX,
                        .52, (255, 220, 0), 1, cv2.LINE_AA)
    perception = perception_status(observations, decision)
    state_color = {'UNKNOWN': (0, 150, 210), 'INVALID': (0, 0, 210), 'KNOWN': (45, 90, 45)}[perception['state']]
    cv2.rectangle(canvas, (0, 0), (size[0]-1, 35), state_color, -1)
    cv2.putText(canvas, 'PERCEPTION: '+perception['state'], (16, 26),
                cv2.FONT_HERSHEY_SIMPLEX, .78, (255, 255, 255), 2, cv2.LINE_AA)
    draw_direction_indicators(canvas, decision, frame.key, indicator_clock_ns)
    fields = ' | '.join(f'{kind}={value}' for kind, value in perception['fields'].items()) or 'no observations=UNKNOWN'
    # Fit every field on the status line rather than silently dropping unknowns.
    field_scale = min(.43, (size[0]-32)/max(1, cv2.getTextSize(fields, cv2.FONT_HERSHEY_SIMPLEX, 1, 1)[0][0]))
    cv2.putText(canvas, fields, (16, 54), cv2.FONT_HERSHEY_SIMPLEX, field_scale, (230, 230, 230), 1, cv2.LINE_AA)
    color = (0, 0, 255) if decision.alarm_reasons else ((0, 190, 250) if decision.simulated_action == 'stop' else (80, 240, 80))
    title = f'PC SIMULATION | allowed: {decision.allowed_action.upper()} | simulated: {decision.simulated_action.upper()}'
    title_scale = min(.67, (size[0]-32)/max(1, cv2.getTextSize(title, cv2.FONT_HERSHEY_SIMPLEX, 1, 2)[0][0]))
    cv2.putText(canvas, title, (16, 81), cv2.FONT_HERSHEY_SIMPLEX, title_scale, color, 2, cv2.LINE_AA)
    reasons = 'ALARM: ' + ', '.join(decision.alarm_reasons) if decision.alarm_reasons else 'Blocked: ' + ', '.join(decision.blocked_reasons)
    cv2.putText(canvas, reasons[:130], (16, 105), cv2.FONT_HERSHEY_SIMPLEX, .51, color, 1, cv2.LINE_AA)
    identity = f'cam={frame.key.camera_id} epoch={frame.key.epoch} frame={frame.key.frame_id} | W/S move/stop L/R change F force C cancel'
    tracked = [lamp['association_age_us'] for observation in observations if observation.kind == 'signal'
               for lamp in observation.details.get('lamps', [])
               if lamp.get('symbol_source') == 'recent_same_color_geometry' and lamp.get('matched_lane')]
    if tracked:
        identity = (f'cam={frame.key.camera_id} epoch={frame.key.epoch} frame={frame.key.frame_id} | '
                    f'SYMBOL RECENT {max(tracked)/1000:.0f}ms (current UNKNOWN); current red/yellow required')
    pending = [(o.kind[0].upper(), o.details['temporal']) for o in observations
               if o.kind in ('left_boundary', 'right_boundary') and
               o.details.get('temporal', {}).get('status') == 'pending']
    if pending:
        states = ' '.join(f'{side}:{t["count"]}/{t["required"]}' for side, t in pending)
        identity = (f'cam={frame.key.camera_id} epoch={frame.key.epoch} frame={frame.key.frame_id} | '
                    'LANE CURRENT DASHED PENDING '+states)
        if tracked:
            identity += f' | SYMBOL RECENT {max(tracked)/1000:.0f}ms (current UNKNOWN)'
    repaired = any(lamp.get('symbol_source') == 'current_color_reconstruction'
                   for observation in observations if observation.kind == 'signal'
                   for lamp in observation.details.get('lamps', []))
    if repaired:
        if not tracked and not pending:
            identity = f'cam={frame.key.camera_id} epoch={frame.key.epoch} frame={frame.key.frame_id}'
        identity += ' | SYMBOL CURRENT REPAIR (raw UNKNOWN; no green release)'
    if geometric:
        identity += ' | SIGNAL GEOM EXPERIMENTAL (CURRENT)'
    if calibrated:
        identity += ' | REFERENCE ROI CALIBRATED (CURRENT)'
    if navigation is not None and navigation.details.get('operator_primary_region') is not None:
        identity += ' | NAV PROMPT ROI CALIBRATED (CURRENT)'
    identity_scale = min(.48, (size[0]-32)/max(1, cv2.getTextSize(identity, cv2.FONT_HERSHEY_SIMPLEX, 1, 1)[0][0]))
    cv2.putText(canvas, identity, (16, 127), cv2.FONT_HERSHEY_SIMPLEX, identity_scale, (215, 215, 215), 1, cv2.LINE_AA)
    center = (size[0]//2, size[1]-35)
    if decision.simulated_action != 'stop':
        end = {'left': (center[0]-95, center[1]-55), 'right': (center[0]+95, center[1]-55),
               'straight': (center[0], center[1]-100)}[decision.simulated_action]
        cv2.arrowedLine(canvas, center, end, color, 10, tipLength=.45)
    if decision.alarm_reasons:
        cv2.rectangle(canvas, (2, 2), (size[0]-3, size[1]-3), (0, 0, 255), 6)
    return canvas
