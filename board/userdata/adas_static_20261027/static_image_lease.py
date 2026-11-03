"""STATIC simulation GUI-health lease. This is not a video freshness bypass.

It is only used by the separate still-image application, never by SceneLease.
No serialized log or camera packet can enter this in-process state contract.
"""
from dataclasses import dataclass
from adas.contracts import Frame,Decision

OFF=0x4c440000


@dataclass(frozen=True)
class StaticSnapshot:
    frame:Frame
    source_sha256:str
    scenario:str
    lane:str
    context_revision:int
    control_revision:int
    decision:Decision
    mode:int


def snapshot(state):
    _,decision=state.view()
    return StaticSnapshot(state.frame,state.source_sha256,state.scenario,state.controls.value.lane,
                          state.context_revision,state.control_revision,decision,state.lamp_mode())


class StaticImageLease:
    def __init__(self,state,*,health_ms=250):
        if type(health_ms) is not int or health_ms!=250:raise ValueError('explicit 250ms static GUI health required')
        frame=state.frame
        if (not isinstance(frame,Frame) or state.session.frame is not frame or
                frame.pts_kind!='not_applicable_image' or frame.pts_us is not None):
            raise ValueError('only an explicit immutable still-image session may renew static health')
        self.state=state;self.frame=frame;self.source_sha256=state.source_sha256
        self.health_ns=health_ms*1_000_000;self.mode=0;self.deadline_ns=0;self.finished=False
        self.last_clock_ns=-1;self.last_sequence=-1;self.last_submitted_ns=-1;self.last_painted_ns=-1

    def _clock(self,now_ns):
        if type(now_ns) is not int or now_ns<0 or now_ns<self.last_clock_ns:
            self.revoke(finished=True);return False
        self.last_clock_ns=now_ns;return True

    def revoke(self,*,finished=False):
        self.mode=0;self.deadline_ns=0;self.finished=self.finished or finished

    def accept(self,value,*,sequence,painted_ns,submitted_ns,now_ns,gui_submitted):
        self.mode=0;self.deadline_ns=0
        if not self._clock(now_ns) or self.finished:return OFF
        try:
            if (type(sequence) is not int or sequence<=self.last_sequence or gui_submitted is not True or
                    type(painted_ns) is not int or type(submitted_ns) is not int or
                    not 0<=painted_ns<=submitted_ns<=now_ns or
                    painted_ns<=self.last_painted_ns or submitted_ns<=self.last_submitted_ns):
                raise ValueError('new actual draw and same-clock GUI submission required')
            self.last_sequence=sequence
            state=self.state
            if (not isinstance(value,StaticSnapshot) or value.frame is not self.frame or state.frame is not self.frame or
                    state.session.frame is not self.frame or self.frame.pts_kind!='not_applicable_image' or self.frame.pts_us is not None or
                    value.source_sha256!=self.source_sha256 or state.source_sha256!=self.source_sha256 or
                    value.scenario!=state.scenario or value.lane!=state.controls.value.lane or
                    value.context_revision!=state.context_revision or value.control_revision!=state.control_revision or
                    state.session.ended or state.needs_analysis):raise ValueError('static source/context changed')
            _,current=state.view()
            if (not isinstance(value.decision,Decision) or value.decision!=current or value.decision.key!=self.frame.key or
                    value.decision.valid is not True or type(value.mode) is not int or value.mode not in range(5) or
                    value.mode!=state.lamp_mode()):raise ValueError('decision/lamp does not match the displayed static context')
            deadline=painted_ns+self.health_ns
            if now_ns>=deadline:raise ValueError('static GUI health expired')
            self.last_submitted_ns=submitted_ns;self.last_painted_ns=painted_ns
            self.deadline_ns=deadline;self.mode=value.mode
            return self.word(now_ns)
        except (AttributeError,KeyError,TypeError,ValueError,OverflowError):return OFF

    def word(self,now_ns):
        if not self._clock(now_ns) or self.finished or self.mode not in (1,2,3,4):return OFF
        lease_ms=min(200,(self.deadline_ns-now_ns)//1_000_000-10)
        if lease_ms<=0:self.mode=0;return OFF
        return OFF|(lease_ms<<8)|self.mode
