"""Warning-only, in-process GTK receipt; deliberately separate from SceneLease.

No green or driving permission is representable here. A serialized old probe
log is not a producer. The caller owns a current live Frame and actual commit.
"""
import hashlib,json,time
from pathlib import Path
from adas.contracts import Frame,FrameKey
try:from .led_sink import DisplayLease,STOP
except ImportError:from led_sink import DisplayLease,STOP

SOURCE_SHA256='9e5954117a9357826332d92fb8da9a34fc089c7ea286e57ec72329d43475d9c4'


class PaintReceipt:
    """One GTK main-thread image generation, acknowledged after default draw."""
    def __init__(self,*,clock=time.perf_counter_ns):
        self.clock=clock;self.sequence=0;self.pending=None;self.drawn=None;self.drawn_ns=None

    def begin(self,key):
        if not isinstance(key,FrameKey):raise ValueError('typed source key required')
        self.sequence+=1;self.pending=(self.sequence,key);self.drawn=None;self.drawn_ns=None
        return self.sequence

    def painted(self,*unused):
        if self.pending is not None:self.drawn=self.pending;self.drawn_ns=self.clock()
        return False

    def acknowledged(self,token,key):
        return self.drawn_ns if self.pending==self.drawn==(token,key) else None

    def revoke(self):self.pending=None;self.drawn=None;self.drawn_ns=None


def cleanup_resources(stopping,lease,board,lock,window,worker):
    """One failed output/close must not skip draining or releasing other owners."""
    stopping.set();lease.revoke(finished=True);errors=[];sent=False;drained=worker is None
    def attempt(label,action):
        try:action()
        except Exception as exc:errors.append(label+': '+type(exc).__name__+': '+str(exc))
    if board is not None:
        try:board.write(STOP);sent=True
        except Exception as exc:errors.append('off: '+type(exc).__name__+': '+str(exc))
    if window is not None:attempt('window_close',window.close)
    if worker is not None:
        attempt('worker_join',lambda:worker.join(8));drained=not worker.is_alive()
        if not drained:errors.append('active NPU/decoder owner did not drain')
    if board is not None:attempt('board_close',board.close)
    if lock is not None:attempt('lock_close',lock.close)
    return dict(off_command_sent=sent,worker_drained=drained,errors=errors)


def validate_bundle(root):
    root=Path(root);record=json.loads((root/'vehicle-fp16-export.json').read_text())
    proof=json.loads((root/'vehicle-fused-probe.json').read_text())
    required=dict(source_sha256=SOURCE_SHA256,target='rk3568',fp16=True,int8=False,training=False,
                  pruning=False,segmentation_removed=True,outputs=['det_out'])
    if any(record.get(k)!=v or type(record.get(k)) is not type(v) for k,v in required.items()):
        raise ValueError('fixed FP16 vehicle graph required')
    if (proof.get('status')!='vehicle_numeric_probe_pass' or proof.get('export')!=record or
            proof.get('native_input') is not True or proof.get('fused') is not True):
        raise ValueError('exact fused native validation required')
    rows=proof.get('rows',[])
    if len(rows)!=3 or {r['case']['index'] for r in rows}!={0,1,2}:raise ValueError('three fixed numerical cases required')
    for row in rows:
        gate=row['gate']
        if (gate.get('passed') is not True or gate.get('native_input_exact') is not True or
                gate.get('limits')!=dict(confidence=.5,vehicle_iou=.9,score_difference=.03)):
            raise ValueError('original vehicle numerical limits required')
    for file,expected in [('yolop-vehicle-only-fp16.rknn',record['rknn_sha256']),('libvehicle_npu_fused.so',proof['bridge_sha256'])]:
        if hashlib.sha256((root/file).read_bytes()).hexdigest()!=expected:raise ValueError('validated asset changed: '+file)
    return dict(source_sha256=SOURCE_SHA256,rknn_sha256=record['rknn_sha256'],bridge_sha256=proof['bridge_sha256'])


class RiskLease(DisplayLease):
    def __init__(self,camera_roles,*,freshness_ms):
        super().__init__(freshness_ms=freshness_ms)
        if (type(camera_roles) is not dict or not camera_roles or
                any(type(k) is not int or k not in range(4) or v not in ('left','front','right','rear') for k,v in camera_roles.items())):
            raise ValueError('explicit camera roles required')
        self.roles=camera_roles.copy();self.seen={};self.high_epoch=-1;self.reason='waiting_analysis'

    def revoke(self,*,finished=False):
        self.mode=0;self.deadline_ns=0;self.finished=self.finished or finished;self.reason='revoked'

    def consume(self,event,now_ns):
        # The legacy display-log receiver can represent GO. It is deliberately
        # not an entry point for this in-process, warning-only adapter.
        self.revoke();self._clock(now_ns);return STOP

    def word(self,now_ns):
        if self.mode not in (0,9,10,12,15):self.revoke()
        return super().word(now_ns)

    def _deadline(self,frame,source_due_ns,now_ns):
        if (self.finished or not isinstance(frame,Frame) or type(frame.pts_us) is not int or frame.pts_us<0 or
                type(source_due_ns) is not int or type(frame.decoded_ns) is not int or type(now_ns) is not int or
                min(source_due_ns,frame.decoded_ns)<0 or max(source_due_ns,frame.decoded_ns)>now_ns or now_ns<self.last_clock_ns):
            raise ValueError('frame clocks not current')
        key=frame.key;previous=self.seen.get((key.epoch,key.camera_id),(-1,-1))
        if (key.camera_id not in self.roles or key.epoch<self.high_epoch or key.frame_id<=previous[0] or frame.pts_us<=previous[1]):
            raise ValueError('source/camera not current')
        deadline=min(frame.decoded_ns,source_due_ns)+self.freshness_ms*1_000_000
        if now_ns>=deadline:raise ValueError('expired warning frame')
        return deadline

    def _evidence_mode(self,frame,report):
        if type(report) is not dict or report.get('key')!=frame.key or report.get('pts_us')!=frame.pts_us:
            raise ValueError('cross-frame warning')
        risks=report.get('risks')
        if type(risks) not in (list,tuple):raise ValueError('explicit warning states required')
        if not risks:return 0
        role=self.roles[frame.key.camera_id]
        if report.get('status')!='observed' or list(risks)!=[role]:raise ValueError('unvalidated risk direction')
        background=report.get('background',{})
        if background.get('valid') is not True or background.get('inliers',0)<20:raise ValueError('background not supported')
        if not any(t.get('approaching') is True and type(t.get('observations')) is int and t['observations']>=4 and
                   t.get('background_compensated') is True and t.get('rotation_supported') is True for t in report.get('tracks',[])):
            raise ValueError('current multi-frame track missing')
        return {'left':9,'front':10,'right':12,'rear':15}[role]

    def preview_mode(self,frame,report,*,source_due_ns,now_ns):
        """Readonly precommit eligibility; never consumes a key or grants a lease."""
        try:
            deadline=self._deadline(frame,source_due_ns,now_ns)
            return self._evidence_mode(frame,report) if now_ns<deadline-20_000_000 else 0
        except (ValueError,TypeError,KeyError,OverflowError):return 0

    def accept(self,frame,report,*,source_due_ns,submitted_ns,now_ns,gui_submitted):
        self.mode=0;self.reason='invalid_current_warning'
        if not self._clock(now_ns) or self.finished:return STOP
        try:
            deadline=self._deadline(frame,source_due_ns,now_ns)
            if (gui_submitted is not True or type(submitted_ns) is not int or submitted_ns<self.last_submitted_ns or
                    not max(source_due_ns,frame.decoded_ns)<=submitted_ns<=now_ns):raise ValueError('frame/clock/GUI mismatch')
            key=frame.key;owner=(key.epoch,key.camera_id)
            self.high_epoch=max(self.high_epoch,key.epoch);self.seen[owner]=(key.frame_id,frame.pts_us)
            self.mode=self._evidence_mode(frame,report);self.deadline_ns=deadline;self.last_submitted_ns=submitted_ns
            self.reason=report.get('status','unknown_provider')
            if not self.mode:return STOP
            role=self.roles[key.camera_id]
            self.reason=role+'_visual_approach'
            return self.word(now_ns)
        except (ValueError,TypeError,KeyError,OverflowError):
            self.mode=0;self.reason='rejected_warning';return STOP
