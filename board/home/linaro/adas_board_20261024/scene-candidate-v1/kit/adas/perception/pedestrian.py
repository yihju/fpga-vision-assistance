"""Person conflict in normalized road corridors, NOT calibrated distance/TTC."""
import cv2
import numpy as np

from ..contracts import Observation


def observe_pedestrians(frame, detections, *, detector_ok=True):
    if any(d.key != frame.key for d in detections):
        raise ValueError('cross-frame detection')
    h, w = frame.bgr.shape[:2]
    corridors = {
        'path_clear': np.float32([[.43*w, .48*h], [.57*w, .48*h], [.82*w, h], [.18*w, h]]),
        'left_clear': np.float32([[.25*w, .48*h], [.43*w, .48*h], [.18*w, h], [0, h]]),
        'right_clear': np.float32([[.57*w, .48*h], [.75*w, .48*h], [w, h], [.82*w, h]])}
    outputs = []
    for kind, polygon in corridors.items():
        conflicts = []
        for detection in detections:
            if detection.label != 'person':
                continue
            x0, y0, x1, y1 = detection.bbox_xyxy
            foot = ((x0+x1)/2, y1)
            if y1 >= .5*h and cv2.pointPolygonTest(polygon, foot, False) >= 0:
                conflicts.append(list(detection.bbox_xyxy))
        value = ('blocked' if conflicts else 'clear') if detector_ok else 'unknown'
        outputs.append(Observation(frame.key, kind, value,
                       details=dict(conflicts=conflicts, scope='person_only_not_vehicle_or_generic_obstacle',
                                    distance_or_TTC_calibrated=False)))
    return tuple(outputs)
