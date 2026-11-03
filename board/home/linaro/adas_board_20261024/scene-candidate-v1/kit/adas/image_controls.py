"""Explicit examiner-operated STATIC IMAGE simulation, never a live-video hold."""
import cv2
import numpy as np
from .contracts import Frame,FrameKey,Observation
from .engine import Analysis
from .overlay import render
from .policy import Controls,decide


class ImageSession:
    def __init__(self,frame,engine,*,scenario='intersection',lane='straight'):
        if (not isinstance(frame,Frame) or not isinstance(frame.key,FrameKey) or
                frame.pts_kind!='not_applicable_image' or frame.pts_us is not None or
                not isinstance(frame.bgr,np.ndarray) or frame.bgr.dtype!=np.uint8 or
                frame.bgr.ndim!=3 or frame.bgr.shape[2]!=3 or min(frame.bgr.shape[:2])<1):
            raise ValueError('decoded static BGR image with explicit non-video identity required')
        if scenario not in ('intersection','lane','navigation') or lane not in ('left','straight','right'):
            raise ValueError('invalid task/lane')
        pixels=frame.bgr.copy();pixels.flags.writeable=False
        self.frame=Frame(frame.key,None,frame.decoded_ns,pixels,'not_applicable_image')
        self.engine,self.scenario=engine,scenario
        self.controls=Controls(epoch=frame.key.epoch)
        self.controls.press({'left':'1','straight':'2','right':'3'}[lane])
        self.analysis=None;self.analysis_calls=0;self.control_count=0;self.ended=False

    @property
    def needs_analysis(self):return self.analysis is None and not self.ended

    def press(self,key):
        if self.ended:return False
        key=key.lower()
        if key not in ('1','2','3','w','s','l','r','t','f','c','i','v','n','\x1b'):return True
        self.control_count+=1
        if key=='\x1b':self.finish();return False
        before=(self.scenario,self.controls.value.lane)
        if key in ('i','v','n'):self.scenario={'i':'intersection','v':'lane','n':'navigation'}[key]
        else:self.controls.press(key)
        if before!=(self.scenario,self.controls.value.lane):
            self.controls.press('s');self.analysis=None;self.engine.reset()
        return True

    def _unknown(self,error):
        kinds=['signal','left_boundary','right_boundary','path_clear','left_clear','right_clear']
        if self.scenario=='navigation':kinds.append('navigation')
        return Analysis(tuple(Observation(self.frame.key,kind,'unknown',fresh=False) for kind in kinds),(),(error,))

    def analyze(self):
        if self.ended:raise RuntimeError('static session is closed')
        if self.analysis is not None:return self.analysis
        self.analysis_calls+=1
        try:
            self.engine.reset()
            result=self.engine.analyze(self.frame,self.controls.value.lane,scenario=self.scenario)
            if not isinstance(result,Analysis):raise ValueError('invalid analysis type')
            kinds=[o.kind for o in result.observations]
            required={'signal','left_boundary','right_boundary','path_clear','left_clear','right_clear'}
            if self.scenario=='navigation':required.add('navigation')
            if (result.errors or len(set(kinds))!=len(kinds) or not required.issubset(kinds) or
                    any(o.key!=self.frame.key for o in (*result.observations,*result.detections)) or
                    any(not isinstance(o,Observation) or not isinstance(o.value,str) or
                        type(o.fresh) is not bool or type(o.applicable) is not bool for o in result.observations)):
                raise ValueError('invalid/mixed/incomplete analysis: '+str(result.errors))
            signal=next(o for o in result.observations if o.kind=='signal')
            if self.scenario=='intersection' and not signal.applicable:
                raise ValueError('signal requirement does not match task')
            if self.scenario=='navigation' and not next(o for o in result.observations if o.kind=='navigation').applicable:
                raise ValueError('navigation requirement missing')
            self.analysis=result
        except Exception as exc:
            self.controls.press('s')
            self.analysis=self._unknown(type(exc).__name__+': '+str(exc))
        return self.analysis

    def view(self):
        analysis=self.analysis if self.analysis is not None and not self.ended else self._unknown('closed' if self.ended else 'waiting_analysis')
        decision=decide(analysis.observations,self.controls.value,self.frame.key,input_valid=not analysis.errors)
        return analysis,decision

    def finish(self):
        self.controls.press('s');self.analysis=None;self.ended=True


def preview(session):
    analysis,decision=session.view()
    pixels=render(session.frame,decision,analysis.observations,analysis.detections,size=(640,480))
    pixels=cv2.copyMakeBorder(pixels,0,96,0,0,cv2.BORDER_CONSTANT,value=(18,18,18))
    notes=[f'STATIC IMAGE / NOT LIVE | TASK: {session.scenario.upper()} | no video timing',
           'I:intersection V:lane N:navigation | 1/2/3 lane | switch=STOP, then W',
           f'source images=1 | analyses={session.analysis_calls} controls={session.control_count} | redraw is NOT new input']
    for i,text in enumerate(notes):
        scale=min(.48,(pixels.shape[1]-24)/max(1,cv2.getTextSize(text,cv2.FONT_HERSHEY_SIMPLEX,1,1)[0][0]))
        cv2.putText(pixels,text,(12,pixels.shape[0]-73+32*i),cv2.FONT_HERSHEY_SIMPLEX,scale,(80,220,255),1,cv2.LINE_AA)
    return pixels
