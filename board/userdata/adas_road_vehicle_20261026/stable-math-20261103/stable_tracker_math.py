"""Same-value math for the stable tracker, with original general-array fallback.

Only the validated float64 rectangle path avoids NumPy's small-array reductions.
Affine prediction keeps the original NumPy matrix product and extrema order.
This module does not alter association thresholds, state or optical-flow inputs.
"""
import math
from pathlib import Path
import hashlib
import json

import numpy as np


def _original_iou(a, b):
    overlap = np.prod(np.maximum(0, np.minimum(a[2:], b[2:]) - np.maximum(a[:2], b[:2])))
    return float(overlap / max(np.prod(a[2:] - a[:2]) + np.prod(b[2:] - b[:2]) - overlap, 1e-9))


def exact_iou(a, b):
    if (type(a) is not np.ndarray or type(b) is not np.ndarray
            or a.dtype != np.float64 or b.dtype != np.float64
            or a.shape != (4,) or b.shape != (4,)):
        return _original_iou(a, b)
    ax0, ay0, ax1, ay1 = a.tolist()
    bx0, by0, bx1, by1 = b.tolist()
    if not all(map(math.isfinite, (ax0, ay0, ax1, ay1, bx0, by0, bx1, by1))):
        return _original_iou(a, b)

    # NumPy minimum/maximum select the second operand on equal values. Keep
    # that detail, including signed zero, rather than using Python min/max.
    left = ax0 if ax0 > bx0 else bx0
    top = ay0 if ay0 > by0 else by0
    right = ax1 if ax1 < bx1 else bx1
    bottom = ay1 if ay1 < by1 else by1
    width = right - left
    height = bottom - top
    width = width if width >= 0. else 0.
    height = height if height >= 0. else 0.
    overlap = width * height
    first_area = (ax1 - ax0) * (ay1 - ay0)
    second_area = (bx1 - bx0) * (by1 - by0)
    denominator = first_area + second_area - overlap
    if not math.isfinite(denominator) or not math.isfinite(overlap):
        return _original_iou(a, b)
    return overlap / max(denominator, 1e-9)


def exact_prediction(box, matrix, scale):
    corners = np.array([[box[0], box[1]], [box[2], box[1]],
                        [box[2], box[3]], [box[0], box[3]]])
    transformed = corners @ matrix[:, :2].T + matrix[:, 2]
    return np.concatenate((transformed.min(axis=0), transformed.max(axis=0)))


def install_verified_math(tracker_module, root):
    """Activate only the helper and original tracker bound to the RK replay."""
    proof = json.loads((Path(root) / 'tracking-validation.json').read_text())
    helper_sha = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    tracker_sha = hashlib.sha256(Path(tracker_module.__file__).read_bytes()).hexdigest()
    if (type(proof.get('frames')) is not int or proof['frames'] < 72
            or any(proof.get(key) is not True for key in ('fresh_nn', 'boxes_exact', 'complete_reports_exact'))
            or proof.get('helper_sha256') != helper_sha
            or proof.get('original_tracker_sha256') != tracker_sha):
        raise ValueError('same-frame exact tracker proof required')
    tracker_module._iou, tracker_module._prediction = exact_iou, exact_prediction
    return dict(backend='same_value_float64_tracker_math', helper_sha256=helper_sha,
                original_tracker_sha256=tracker_sha, validated_frames=proof['frames'])
