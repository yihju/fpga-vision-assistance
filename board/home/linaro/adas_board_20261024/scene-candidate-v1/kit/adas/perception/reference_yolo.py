"""Six-class reference YOLOv5; explicit output domains and custom anchors."""
from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
import time

import cv2
import numpy as np

from ..contracts import FrameKey

LABELS = ('zebra', 'person', 'red', 'green', 'black', 'yellow')
ROOT = Path(__file__).resolve().parents[2]


@dataclass(frozen=True)
class Detection:
    key: FrameKey
    label: str
    confidence: float
    bbox_xyxy: tuple[float, float, float, float]


def preprocess(image):
    if (not isinstance(image, np.ndarray) or image.dtype != np.uint8 or image.ndim != 3 or
            image.shape[2] != 3 or min(image.shape[:2]) < 1):
        raise ValueError('nonempty UINT8 BGR required')
    h, w = image.shape[:2]
    ratio = min(640/w, 640/h)
    rw, rh = int(w*ratio), int(h*ratio)
    if min(rw, rh) < 1:
        raise ValueError('image aspect too extreme')
    left, top = (640-rw)//2, (640-rh)//2
    canvas = np.zeros((640, 640, 3), np.uint8)
    canvas[top:top+rh, left:left+rw] = cv2.resize(image, (rw, rh))
    tensor = np.ascontiguousarray((canvas[:, :, ::-1].astype(np.float32)/255).transpose(2, 0, 1)[None])
    return tensor, dict(r=ratio, left=left, top=top)


def _iou(a, b):
    intersection = max(0., min(a[2], b[2])-max(a[0], b[0])) * max(0., min(a[3], b[3])-max(a[1], b[1]))
    union = (a[2]-a[0])*(a[3]-a[1]) + (b[2]-b[0])*(b[3]-b[1]) - intersection
    return intersection/union if union > 0 else 0.


def decode_heads(outputs, anchors, *, domain, threshold=.25, nms_iou=.4):
    anchors = np.asarray(anchors, np.float32)
    if (domain not in ('probabilities', 'logits') or len(outputs) != 3 or anchors.shape != (9, 2) or
            not np.isfinite(anchors).all() or np.any(anchors <= 0) or
            not 0 < threshold < 1 or not 0 < nms_iou < 1):
        raise ValueError('explicit domain, 9 positive custom anchors and valid thresholds required')
    candidates = []
    for head_index, (raw, n) in enumerate(zip(outputs, (80, 40, 20))):
        if (not isinstance(raw, np.ndarray) or raw.shape != (1, 33, n, n) or
                raw.dtype != np.float32 or not np.isfinite(raw).all()):
            raise ValueError('finite float32[1,33,80/40/20,80/40/20] required')
        if domain == 'probabilities':
            if np.any(raw < 0) or np.any(raw > 1):
                raise ValueError('probabilities outside [0,1]')
            value = raw
        else:
            value = 1/(1+np.exp(-np.clip(raw, -80, 80)))
        value = value.reshape(3, 11, n, n).transpose(0, 2, 3, 1)
        classes = value[..., 5:].argmax(axis=-1)
        scores = value[..., 4] * np.max(value[..., 5:], axis=-1)
        stride = 640/n
        for anchor, y, x in np.argwhere(scores >= threshold):
            row = value[anchor, y, x]
            cx, cy = (row[:2]*2-.5 + [x, y])*stride
            width, height = (row[2:4]*2)**2 * anchors[head_index*3+anchor]
            candidates.append(dict(label=LABELS[int(classes[anchor, y, x])],
                                   confidence=float(scores[anchor, y, x]),
                                   bbox_xyxy=[float(cx-width/2), float(cy-height/2),
                                              float(cx+width/2), float(cy+height/2)]))
    candidates.sort(key=lambda c: -c['confidence'])
    kept = []
    for candidate in candidates:
        if all(candidate['label'] != other['label'] or
               _iou(candidate['bbox_xyxy'], other['bbox_xyxy']) <= nms_iou for other in kept):
            kept.append(candidate)
    return kept


class ReferenceDetector:
    def __init__(self, *, threads=4):
        import onnxruntime as ort
        self.manifest = json.loads((ROOT/'adas/assets/reference-yolo.json').read_text(encoding='utf-8'))
        folder = (ROOT/self.manifest['directory_relative_to_project']).resolve()
        for kind, digest in (('model', 'model_sha256'), ('anchors', 'anchor_sha256')):
            path = folder/self.manifest[kind]
            with path.open('rb') as stream:
                if hashlib.file_digest(stream, 'sha256').hexdigest() != self.manifest[digest]:
                    raise ValueError('reference asset SHA mismatch: ' + str(path))
        self.anchors = np.fromstring((folder/self.manifest['anchors']).read_text(encoding='utf-8')
                                    .replace(',', ' '), sep=' ', dtype=np.float32).reshape(9, 2)
        options = ort.SessionOptions()
        options.intra_op_num_threads = threads
        options.inter_op_num_threads = 1
        self.session = ort.InferenceSession(str(folder/self.manifest['model']), options,
                                           providers=['CPUExecutionProvider'])
        inputs, outputs = self.session.get_inputs(), self.session.get_outputs()
        if (len(inputs) != 1 or inputs[0].shape != [1, 3, 640, 640] or inputs[0].type != 'tensor(float)' or
                [o.shape for o in outputs] != [[1, 33, n, n] for n in (80, 40, 20)]):
            raise ValueError('reference tensor contract mismatch')
        self.name = inputs[0].name
        self.last_inference_ms = None

    def detect(self, frame):
        tensor, transform = preprocess(frame.bgr)
        started = time.perf_counter_ns()
        outputs = self.session.run(None, {self.name: tensor})
        self.last_inference_ms = (time.perf_counter_ns()-started)/1e6
        rows = decode_heads(outputs, self.anchors, domain=self.manifest['onnx_output_domain'])
        ratio, left, top = (transform[k] for k in ('r', 'left', 'top'))
        h, w = frame.bgr.shape[:2]
        detections = []
        for row in rows:
            x0, y0, x1, y1 = row['bbox_xyxy']
            box = (max(0., (x0-left)/ratio), max(0., (y0-top)/ratio),
                   min(float(w), (x1-left)/ratio), min(float(h), (y1-top)/ratio))
            if box[0] < box[2] and box[1] < box[3]:
                detections.append(Detection(frame.key, row['label'], row['confidence'], box))
        return detections
