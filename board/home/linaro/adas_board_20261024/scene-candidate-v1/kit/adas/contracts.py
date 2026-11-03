"""Frame identity shared by media, perception, decisions and overlays."""
from dataclasses import dataclass, field
from typing import Any


@dataclass(frozen=True)
class FrameKey:
    epoch: int
    frame_id: int
    camera_id: int = 0

    def __post_init__(self):
        if (any(type(v) is not int or v < 0 for v in (self.epoch, self.frame_id, self.camera_id)) or
                self.camera_id > 3):
            raise ValueError('nonnegative epoch/frame and camera_id 0..3 required')


@dataclass(frozen=True)
class Frame:
    key: FrameKey
    pts_us: int | None
    decoded_ns: int
    bgr: Any = field(repr=False, compare=False)
    pts_kind: str = 'decoder_reported'


@dataclass(frozen=True)
class Control:
    lane: str = 'straight'
    requested_motion: bool = False
    turn: str | None = None
    force_enabled: bool = False
    epoch: int = 0

    def __post_init__(self):
        if (self.lane not in ('left', 'straight', 'right') or self.turn not in (None, 'left', 'right') or
                type(self.requested_motion) is not bool or type(self.force_enabled) is not bool or
                type(self.epoch) is not int or self.epoch < 0):
            raise ValueError('invalid simulated controls')


@dataclass(frozen=True)
class Observation:
    key: FrameKey
    kind: str
    value: str
    applicable: bool = True
    fresh: bool = True
    details: dict = field(default_factory=dict, compare=False)


@dataclass(frozen=True)
class Decision:
    key: FrameKey
    requested_action: str
    allowed_action: str
    simulated_action: str
    alarm_reasons: tuple[str, ...]
    blocked_reasons: tuple[str, ...]
    valid: bool
