"""Pure, deterministic rules. Inputs are perception, never filenames or GT."""
from dataclasses import replace

from .contracts import Control, Decision


def decide(observations, control, key, *, input_valid=True):
    by_kind = {}
    invalid = not input_valid or control.epoch != key.epoch
    for observation in observations:
        if observation.key != key or observation.kind in by_kind:
            invalid = True
        by_kind[observation.kind] = observation
    if invalid:
        return Decision(key, 'stop', 'stop', 'stop', (), ('invalid_or_mixed_frame',), False)

    def current(kind):
        value = by_kind.get(kind)
        return value.value if value is not None and value.fresh else 'unknown'

    signal = by_kind.get('signal')
    nav = by_kind.get('navigation')
    signal_required = signal is not None and signal.applicable
    nav_required = nav is not None and nav.applicable
    navigation = current('navigation') if nav_required else None
    requested = control.turn or (navigation if navigation in ('left', 'straight', 'right') else None)
    requested = requested or (control.lane if signal_required else 'straight')
    reasons, stop = [], False
    path = current('path_clear')
    if path != 'clear':
        reasons.append('pedestrian_conflict' if path == 'blocked' else 'path_unknown')
        stop = True
    if signal_required:
        color = current('signal')
        if color != 'green':
            reasons.append('red_signal' if color == 'red' else 'signal_' + color)
            stop = True
    if nav_required and navigation not in ('left', 'straight', 'right'):
        reasons.append('navigation_unknown')
        stop = True

    changing_lane = control.turn is not None or (nav_required and navigation in ('left', 'right'))
    if changing_lane:
        side = requested
        line = current(side + '_boundary')
        if line != 'dashed':
            reasons.append(('solid_line_' if line == 'solid' else 'boundary_unknown_') + side)
        if current(side + '_clear') != 'clear':
            reasons.append('target_not_clear_' + side)
    allowed = 'stop' if stop else ('straight' if reasons else requested)
    if not control.requested_motion:
        return Decision(key, 'stop', 'stop', 'stop', (), tuple(reasons), True)
    simulated = requested if control.force_enabled else allowed
    alarms = tuple(reasons) if control.force_enabled and simulated != allowed else ()
    return Decision(key, requested, allowed, simulated, alarms, tuple(reasons), True)


class Controls:
    def __init__(self, epoch=0):
        self.value = Control(epoch=epoch)

    def reset(self, epoch):
        self.value = Control(epoch=epoch)

    def press(self, key):
        key = key.lower()
        value = self.value
        if key in ('1', '2', '3'):
            value = replace(value, lane={'1': 'left', '2': 'straight', '3': 'right'}[key])
        elif key == 'w':
            value = replace(value, requested_motion=True)
        elif key == 's':
            value = replace(value, requested_motion=False, force_enabled=False, turn=None)
        elif key in ('l', 'r'):
            value = replace(value, turn='left' if key == 'l' else 'right')
        elif key == 't':
            value = replace(value, turn=None)
        elif key == 'f':
            value = replace(value, force_enabled=True)
        elif key == 'c':
            value = replace(value, force_enabled=False)
        self.value = value
        return key != '\x1b'
