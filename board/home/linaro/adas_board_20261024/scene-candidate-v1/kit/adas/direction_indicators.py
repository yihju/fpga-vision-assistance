"""Three vehicle-decision indicators, shared patterns for a future hardware sink."""
import time

import cv2

DIRECTIONS=('left','straight','right')
PATTERNS=('red_flash','red_steady','green_steady')


def indicator_patterns(decision,key):
    if decision.key!=key:raise ValueError('refuse cross-frame direction indicators')
    stopped=('red_flash',)*3
    if not decision.valid:return stopped
    if decision.alarm_reasons:return ('red_steady',)*3
    if (decision.allowed_action not in DIRECTIONS or
            decision.simulated_action!=decision.allowed_action):return stopped
    return tuple('green_steady' if direction==decision.allowed_action else 'red_flash' for direction in DIRECTIONS)


def indicator_colors(patterns,clock_ns):
    if (type(clock_ns) is not int or clock_ns<0 or len(patterns)!=3 or
            any(value not in PATTERNS for value in patterns) or patterns.count('green_steady')>1):
        raise ValueError('three safe patterns and a nonnegative integer monotonic clock required')
    on=clock_ns%1000000000<500000000
    return tuple('green' if pattern=='green_steady' else
                 'red' if pattern=='red_steady' or on else 'off' for pattern in patterns)


def draw_direction_indicators(canvas,decision,key,clock_ns=None):
    """Draw inside the existing top band; neither source road nor policy changes."""
    clock_ns=time.perf_counter_ns() if clock_ns is None else clock_ns
    colors=indicator_colors(indicator_patterns(decision,key),clock_ns)
    start=canvas.shape[1]-300
    palette={'red':(0,0,255),'green':(0,220,0),'off':(45,45,45)}
    for index,(label,color) in enumerate(zip(DIRECTIONS,colors)):
        x=start+index*96
        cv2.rectangle(canvas,(x-3,2),(x+91,33),(12,12,12),-1)
        cv2.circle(canvas,(x+9,18),9,(115,115,115),1)
        cv2.circle(canvas,(x+9,18),8,palette[color],-1)
        cv2.putText(canvas,label.upper(),(x+23,23),cv2.FONT_HERSHEY_SIMPLEX,.38,
                    (240,240,240),1,cv2.LINE_AA)
