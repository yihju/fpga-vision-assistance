"""Asynchronous static-image controls; perception remains worker-owned."""
from collections import deque
from adas.background import AnalysisResult
from adas.image_controls import ImageSession
from adas.board_keys import commands_for_event
try:
    from .board_control import EVENTS
except ImportError:
    from board_control import EVENTS


class _ResetProxy:
    def reset(self):pass
    def analyze(self,*args,**kwargs):
        raise RuntimeError('static analysis must come from worker')


class _AnalysisStub:
    def __init__(self,analysis):self.analysis=analysis
    def reset(self):pass
    def analyze(self,*args,**kwargs):return self.analysis


class StaticImageState:
    def __init__(self,frame,source_sha256,*,scenario='intersection',lane='straight',move=False):
        if (not isinstance(source_sha256,str) or len(source_sha256)!=64 or
                any(c not in '0123456789abcdefABCDEF' for c in source_sha256)):
            raise ValueError('source SHA256 required')
        if type(move) is not bool:raise ValueError('explicit move boolean required')
        self.session=ImageSession(frame,_ResetProxy(),scenario=scenario,lane=lane)
        self.frame=self.session.frame
        self.source_sha256=source_sha256.lower()
        self.context_revision=0;self.control_revision=0
        self.last_control=None;self.control_history=deque(maxlen=3)
        self.physical_force_motion=False;self._muted=False;self.error=None
        if move:self.controls.press('w')

    @property
    def controls(self):return self.session.controls
    @property
    def scenario(self):return self.session.scenario
    @property
    def needs_analysis(self):return self.error is None and self.session.needs_analysis

    def press(self,key):
        if self.session.ended:return False
        if not isinstance(key,str):return True
        event=EVENTS.get(key)
        key=key.lower()
        if event is None and key not in ('1','2','3','w','s','l','r','t','f','c','i','v','n','\x1b'):return True
        before=(self.scenario,self.controls.value.lane)
        commands=commands_for_event(event,self.controls.value,self.scenario) if event else (key,)
        # A held physical FORCE owns its inserted W until a boundary or release.
        if self.physical_force_motion and (event!='KEY1_LONG_START' and
                any(c in ('s','c','i','v','n','1','2','3','l','r','t','\x1b') for c in commands)):
            self.controls.press('s');self.physical_force_motion=False
        if event=='KEY1_LONG_START':self.physical_force_motion=True
        self.control_revision+=1;self.last_control=event or key
        self.control_history.append(self.last_control)
        if self.scenario=='intersection':self._muted=True
        active=True
        for command in commands:
            if not self.session.press(command):active=False;break
        after=(self.scenario,self.controls.value.lane)
        if before!=after:
            self.context_revision+=1
            self.physical_force_motion=False
            self.error=None
            # Only a different task re-arms ordinary indications on this source.
            if before[0]!=after[0]:self._muted=False
        if not active:
            self.context_revision+=1;self.physical_force_motion=False
        if self.error is not None:
            self.controls.press('s');self.physical_force_motion=False
        return active

    def mark_submitted(self):
        if self.session.ended or self.error is not None:raise RuntimeError('static session is closed or faulted')
        self.session.analysis_calls+=1

    def accept(self,result):
        if (self.session.ended or self.error is not None or not isinstance(result,AnalysisResult) or
                result.frame is not self.frame or result.frame.key!=self.frame.key or
                result.frame.pts_us!=self.frame.pts_us or
                result.frame.pts_kind!=self.frame.pts_kind or
                result.frame.decoded_ns!=self.frame.decoded_ns or
                result.lane!=self.controls.value.lane or
                type(result.revision) is not int or result.revision!=self.context_revision):
            return False
        # A fresh validator avoids overwriting valid cached evidence on a bad result.
        validator=ImageSession(self.frame,_AnalysisStub(result.analysis),
                               scenario=self.scenario,lane=self.controls.value.lane)
        analysis=validator.analyze()
        if analysis.errors:
            self.fail('; '.join(analysis.errors));return False
        self.session.analysis=analysis
        return True

    def view(self):return self.session.view()

    def lamp_mode(self):
        analysis,decision=self.view()
        if analysis.errors or not decision.valid:return 0
        if decision.alarm_reasons:return 4
        if self._muted:return 0
        if self.scenario=='lane' and self.controls.value.turn is None:return 0
        if self.controls.value.requested_motion and decision.simulated_action=='stop':return 4
        return {'left':1,'straight':2,'right':3}.get(decision.simulated_action,0)

    def finish(self):
        if not self.session.ended:
            self.context_revision+=1;self.control_revision+=1
        self.physical_force_motion=False;self.session.finish()

    def fail(self,reason):
        self.context_revision+=1;self.control_revision+=1
        self.error=str(reason)
        self.controls.press('s');self.session.analysis=self.session._unknown(self.error)
        self.physical_force_motion=False

