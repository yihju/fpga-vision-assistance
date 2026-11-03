"""Independent one-image board simulator; GUI health is never video freshness."""
import argparse,hashlib,json,os,sys,time
from pathlib import Path
from dataclasses import asdict,is_dataclass

BASE=Path('/home/linaro/adas_board_20261024')
KIT=BASE/'scene-candidate-v1/kit'
DEPLOY=Path('/userdata/adas_static_20261027')


def bootstrap():
    for path in (BASE,KIT):
        if str(path) not in sys.path:sys.path.insert(0,str(path))
    from arm_npu import install
    install(BASE)


if os.environ.get('ADAS_STATIC_NPU')=='1':bootstrap()

try:
    from .static_image_lease import StaticImageLease,snapshot,OFF
    from .approach_output import PaintReceipt
except ImportError:
    from static_image_lease import StaticImageLease,snapshot,OFF
    from approach_output import PaintReceipt


def serial(value):
    if is_dataclass(value):return asdict(value)
    if hasattr(value,'tolist'):return value.tolist()
    return str(value)


def picture(state):
    import cv2,numpy as np
    from adas.overlay import render
    analysis,decision=state.view()
    road=render(state.frame,decision,analysis.observations,analysis.detections,size=(832,468))
    footer=np.zeros((72,832,3),np.uint8);c=state.controls.value
    lines=[f'STATIC IMAGE / NOT LIVE | {state.scenario} | SELECTED {c.lane}',
           f'REQUEST {c.requested_motion} {c.turn or "none"} | FORCE {c.force_enabled} | LED {state.lamp_mode()} | LAST KEY {state.last_control}',
           'RECEIPTS '+ ' / '.join(state.control_history)+(' | ERROR (see log)' if state.error else '')]
    for i,line in enumerate(lines):
        width=cv2.getTextSize(line,cv2.FONT_HERSHEY_SIMPLEX,1,1)[0][0]
        scale=min(.45,816/max(1,width))
        cv2.putText(footer,line,(8,20+i*23),cv2.FONT_HERSHEY_SIMPLEX,scale,(240,240,240),1,cv2.LINE_AA)
    return np.vstack((road,footer))


def run(*,state,window,worker,board,output,max_seconds=None,render=None,poll_seconds=.005):
    """Run the real control loop with injectable resource owners and draw receipt.

    A custom render returns an actual paint timestamp or None. Production uses
    GTK connect_after(draw), polls keys throughout waiting, and never captures X11.
    """
    output=Path(output);output.mkdir(parents=True,exist_ok=True)
    stream=(output/'events.jsonl').open('a',encoding='utf-8')
    def emit(kind,**values):
        stream.write(json.dumps(dict(kind=kind,now_ns=time.perf_counter_ns(),**values),default=serial)+'\n');stream.flush()
    lease=StaticImageLease(state);paint=PaintReceipt();sequence=0;active=True
    began=time.monotonic();ready_logged=False;rejected=0;cleanup_errors=[];failure=None
    final_off_readback=None;final_off_write_succeeded=False
    def write(word,reason):
        readback=board.write(word);emit('led',word=word,mode=word&15,readback=readback,reason=reason)
        return readback
    def poll():
        nonlocal active
        key=window.poll()
        if key is None or key==-1:return
        # Revoke before applying even an unrecognized key.
        lease.revoke();write(OFF,'input')
        key=chr(key) if isinstance(key,int) else key
        active=state.press(key) and active
        emit('control',key=key,last_control=state.last_control,control_revision=state.control_revision,
             context_revision=state.context_revision,controls=state.controls.value)
    try:
        write(OFF,'start')
        emit('source',source_sha256=state.source_sha256,decoded_sha256=hashlib.sha256(state.frame.bgr.tobytes()).hexdigest(),
             key=state.frame.key,pts_us=state.frame.pts_us,pts_kind=state.frame.pts_kind,decoded_ns=state.frame.decoded_ns)
        if render is None:window.image.connect_after('draw',paint.painted)
        while active and window.is_open() and (max_seconds is None or time.monotonic()-began<max_seconds):
            poll()
            if not active:break
            if not worker.alive:raise RuntimeError('analysis worker exited')
            result=worker.poll()
            if worker.ready and not ready_logged:emit('worker_ready',manifest=worker.manifest);ready_logged=True
            if result is not None:
                accepted=state.accept(result);rejected+=not accepted
                emit('result',accepted=accepted,request_id=result.request_id,key=result.frame.key,lane=result.lane,
                     context_revision=result.revision,analysis=result.analysis,worker_ms=result.worker_ms)
            if state.needs_analysis and worker.ready and not worker.busy:
                workload='intersection_light' if state.scenario=='intersection' else 'default'
                if worker.submit(state.frame,state.controls.value.lane,state.context_revision,state.scenario,workload=workload):
                    state.mark_submitted();emit('submit',key=state.frame.key,context_revision=state.context_revision,
                                                lane=state.controls.value.lane,scenario=state.scenario,workload=workload)
            value=snapshot(state);sequence+=1
            if render is not None:painted=render(state,value,window)
            else:
                token=paint.begin(state.frame.key);window.show(picture(state));painted=None
                until=time.perf_counter_ns()+80_000_000
                while active and window.is_open() and time.perf_counter_ns()<until:
                    poll()
                    painted=paint.acknowledged(token,state.frame.key)
                    if painted is not None:break
                    time.sleep(min(poll_seconds,.002))
            # Mandatory second sampling after render/draw, before any lamp write.
            poll();submitted=time.perf_counter_ns()
            word=OFF
            if active and window.is_open() and painted is not None:
                word=lease.accept(value,sequence=sequence,painted_ns=painted,submitted_ns=submitted,
                                  now_ns=time.perf_counter_ns(),gui_submitted=True)
            else:lease.revoke()
            emit('display_candidate',key=value.frame.key,context_revision=value.context_revision,control_revision=value.control_revision,
                 painted_ns=painted,submitted_ns=submitted,mode=word&15,decision=value.decision)
            # Logging may block longer than the GUI-health deadline. Re-sample
            # input and recompute immediately at the hardware boundary, with no
            # serialization or flush between this clock and the MMIO write.
            poll()
            if not active or not window.is_open():lease.revoke()
            write_ns=time.perf_counter_ns();word=lease.word(write_ns)
            readback=board.write(word)
            emit('display',key=value.frame.key,context_revision=value.context_revision,control_revision=value.control_revision,
                 painted_ns=painted,submitted_ns=submitted,write_ns=write_ns,mode=word&15,word=word,
                 readback=readback,decision=value.decision)
            time.sleep(poll_seconds)
    except Exception as exc:
        failure=f'{type(exc).__name__}: {exc}';state.fail(failure);emit('error',error=failure)
    finally:
        state.finish();lease.revoke(finished=True)
        try:
            final_off_readback=write(OFF,'final_stop');final_off_write_succeeded=True
        except Exception as exc:cleanup_errors.append('off: '+str(exc))
        for name,action in [('last_stop',lambda:save_stop(state,output)),
                            ('worker_close',worker.close),('window_close',window.close),('board_close',board.close)]:
            try:action()
            except Exception as exc:cleanup_errors.append(name+': '+str(exc))
        summary=dict(input_frames=1,analysis_calls=state.session.analysis_calls,ai_fps=None,video_latency=None,optical_latency=None,
                     rejected_results=rejected,error=failure,cleanup_errors=cleanup_errors,final_requested_mode=0,
                     final_off_write_succeeded=final_off_write_succeeded,final_off_readback=final_off_readback,
                     physical_off_verified=None,
                     key=state.frame.key,pts_us=state.frame.pts_us,pts_kind=state.frame.pts_kind,decoded_ns=state.frame.decoded_ns)
        emit('final_stop',**summary);stream.close()
        (output/'summary.json').write_text(json.dumps(summary,default=serial,indent=2),encoding='utf-8')
    return summary


def save_stop(state,output):
    import cv2
    if not cv2.imwrite(str(output/'last-stop.jpg'),picture(state)):raise OSError('last STOP image write failed')


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('image',type=Path);parser.add_argument('--output',required=True,type=Path)
    parser.add_argument('--scenario',choices=('intersection','lane','navigation'),default='intersection')
    parser.add_argument('--lane',choices=('left','straight','right'),default='straight')
    parser.add_argument('--move',action='store_true',help='explicit initial simulated motion')
    parser.add_argument('--max-seconds',type=float)
    args=parser.parse_args(argv)
    if args.max_seconds is not None and (not 0<args.max_seconds<float('inf')):parser.error('positive finite max-seconds required')
    if args.image.suffix.lower() not in ('.jpg','.jpeg','.png','.bmp','.webp','.tif','.tiff'):parser.error('single still image required')
    os.environ['ADAS_STATIC_NPU']='1';bootstrap()
    import cv2,fcntl,signal
    from adas.contracts import Frame,FrameKey
    try:from .static_image_state import StaticImageState
    except ImportError:from static_image_state import StaticImageState
    from generic_worker import GenericFrameWorker
    from gtk_window import GtkWindow
    from board_control import window_class
    from board_key_source import BoardKeySource
    from scene_key_audit import AuditedKeys
    from scene_led_sink import ScenePcieLed
    cv2.setNumThreads(1)
    data=args.image.resolve().read_bytes()
    import numpy as np
    pixels=cv2.imdecode(np.frombuffer(data,np.uint8),cv2.IMREAD_COLOR)
    if pixels is None:raise ValueError('image decode failed')
    state=StaticImageState(Frame(FrameKey(0,0),None,time.perf_counter_ns(),pixels,'not_applicable_image'),
                           hashlib.sha256(data).hexdigest(),scenario=args.scenario,lane=args.lane,move=args.move)
    args.output.mkdir(parents=True,exist_ok=True)
    window=worker=board=None
    with open('/run/lock/adas-led.lock','a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        try:
            board=ScenePcieLed();board.write(OFF)
            window=window_class(GtkWindow)(task_switching=True,source_factory=lambda:AuditedKeys(BoardKeySource,args.output/'board-key-audit.jsonl'))
            for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):signal.signal(sig,lambda *_:window.push('\x1b'))
            worker=GenericFrameWorker(engine_kwargs=dict(threads=4,parallel_perception=True))
            summary=run(state=state,window=window,worker=worker,board=board,output=args.output,max_seconds=args.max_seconds)
            return 1 if summary['error'] or summary['cleanup_errors'] else 0
        finally:
            for resource in (worker,window,board):
                if resource is not None:
                    try:resource.close()
                    except Exception:pass


if __name__=='__main__':raise SystemExit(main())
