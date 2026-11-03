"""Independent single-front visual warning check; no driving permissions.

Uses the actual latest native source, fixed FP16 graph, two slots and current
GTK commits. No old log replay, no answer timeline, no Flash or DMA changes.
"""
from pathlib import Path
from dataclasses import asdict,is_dataclass
import argparse,collections,importlib.util,json,signal,sys,threading,time
import cv2,numpy as np


def load(name,path):
    spec=importlib.util.spec_from_file_location(name,path);module=importlib.util.module_from_spec(spec)
    sys.modules[name]=module;spec.loader.exec_module(module);return module


def marker_for(run_tag,frame_id):
    bits=np.unpackbits(np.frombuffer(((run_tag<<32)|frame_id).to_bytes(8,'big'),dtype=np.uint8)).reshape(8,8)
    colors=np.array([[35,60,95],[210,120,25]],dtype=np.uint8)
    return np.repeat(np.repeat(colors[bits],4,axis=0),4,axis=1)


def desktop_matches(window,marker,deadline_ns,stopping,*,full_capture=False):
    """Read actual X11 pixels, not a queued GtkImage or an intended frame."""
    until=min(deadline_ns-20_000_000,time.perf_counter_ns()+100_000_000)
    allocation=window.image.get_allocation();origin=tuple(window.image.get_window().get_origin())
    if len(origin)!=3 or not origin[0]:return None,None
    x=origin[1]+allocation.x+790;y=origin[2]+allocation.y+3
    if not (0<=x<=1024-32 and 0<=y<=600-32):return None,None
    def active():return not stopping.is_set() and window.is_open() and not getattr(window,'quit_requested',False)
    while active() and time.perf_counter_ns()<until:
        window._pump();gdk=window.Gdk;root=gdk.get_default_root_window()
        capture=gdk.pixbuf_get_from_window(root,0,0,1024,600) if full_capture else gdk.pixbuf_get_from_window(root,x,y,32,32)
        if capture is not None:
            channels=capture.get_n_channels();width=capture.get_width();height=capture.get_height()
            pixels=np.frombuffer(capture.get_pixels(),np.uint8).reshape(height,capture.get_rowstride())[:,:width*channels]
            pixels=pixels.reshape(height,width,channels)[:,:,:3][:,:,::-1].copy()
            actual=pixels[y:y+32,x:x+32] if full_capture else pixels
            matches=actual.shape==marker.shape and np.array_equal(actual,marker)
            captured=time.perf_counter_ns()
            if matches and active() and captured<until:
                return pixels,dict(captured_ns=captured,tag_max_difference=0,tag_position=(x,y),
                                   capture_kind='root_desktop' if full_capture else 'root_marker_roi')
        stopping.wait(.008)
    return None,None


def wait_painted(window,receipt,token,key,deadline_ns,stopping):
    """A draw event is a software paint receipt, never an optical timestamp."""
    until=min(deadline_ns-20_000_000,time.perf_counter_ns()+100_000_000)
    while not stopping.is_set() and window.is_open() and not window.quit_requested and time.perf_counter_ns()<until:
        window._pump();painted=receipt.acknowledged(token,key);now=time.perf_counter_ns()
        if stopping.is_set() or not window.is_open() or window.quit_requested:return None
        if painted is not None and now<until:return painted
        stopping.wait(.002)
    return None


def picture(frame,report,*,warning,stale=False,marker=None):
    # Display-only linear resampling. Native AREA model input and LK pixels
    # keep their original validated interpolation; this never touches them.
    image=cv2.resize(frame.bgr,(832,468),interpolation=cv2.INTER_LINEAR)
    scale_x=832/frame.bgr.shape[1];scale_y=468/frame.bgr.shape[0]
    if not stale:
        for row in report['tracks']:
            x1,y1,x2,y2=row['xyxy'];color=(0,190,255) if warning and row['approaching'] else (180,180,80)
            a=(int(x1*scale_x),int(y1*scale_y));b=(int(x2*scale_x),int(y2*scale_y))
            cv2.rectangle(image,a,b,color,2)
            cv2.putText(image,'vehicle #'+str(row['id']),a,cv2.FONT_HERSHEY_SIMPLEX,.4,color,1,cv2.LINE_AA)
    header=np.zeros((72,832,3),np.uint8)
    cv2.putText(header,'SINGLE FRONT | VISUAL APPROACH | FUNCTION CHECK 600ms',(12,23),cv2.FONT_HERSHEY_SIMPLEX,.56,(0,220,255),1,cv2.LINE_AA)
    status='STALE / NO CURRENT WARNING' if stale else ('FRONT APPROACH - CAUTION' if warning else report['status'].upper()+' / NOT A SAFETY GUARANTEE')
    cv2.putText(header,status,(12,46),cv2.FONT_HERSHEY_SIMPLEX,.59,(0,190,255) if warning else (225,225,225),2,cv2.LINE_AA)
    cv2.putText(header,'source frame '+str(frame.key.frame_id)+' | PTS '+format(frame.pts_us/1e6,'.3f')+'s | ESC close',(12,67),cv2.FONT_HERSHEY_SIMPLEX,.49,(210,210,210),1,cv2.LINE_AA)
    if marker is not None and not stale:header[3:35,790:822]=marker
    return np.vstack((header,image))


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--root',required=True,type=Path)
    p.add_argument('--output',required=True,type=Path);p.add_argument('--led',action='store_true');p.add_argument('--native-tracking',action='store_true')
    p.add_argument('--exact-postprocess',action='store_true',help='use independently validated same-output head scan and appearance histogram')
    p.add_argument('--shared-tracking-input',action='store_true',help='reuse validated same-frame area sums for original tracking pixels')
    p.add_argument('--exact-tracking-math',action='store_true',help='use same-frame validated tracker math without temporary reductions')
    args=p.parse_args();root=args.root;args.output.mkdir(parents=True,exist_ok=False)
    board_root=Path('/home/linaro/adas_board_20261024')
    sys.path[:0]=[str(root),str(board_root),str(board_root/'scene-candidate-v1/kit')]
    from approach_output import RiskLease,PaintReceipt,validate_bundle,cleanup_resources,STOP
    from scene_led_sink import ScenePcieLed
    from gtk_window import GtkWindow
    from async_media import AsyncLatestReader
    from road_vehicle_npu import VehicleSession,decode_vehicles
    from road_vehicle_probe import Mapping
    exact_identity=None
    if args.exact_postprocess:
        optimization=root/'stable-exact-20261103';sys.path.insert(0,str(optimization))
        from stable_sparse_decode import StableSparseDecoder
        from native_appearance import NativeAppearance
        decode_vehicles=StableSparseDecoder(optimization)
        appearance=NativeAppearance(optimization)
        exact_identity=dict(decoder='validated_native_stable_selection',appearance='validated_same_cv2_HSV_histogram')
    tracker_module=load('adas.approach_probe',root/'approach_probe.py')
    if exact_identity is not None:tracker_module._appearance=appearance
    pipeline=load('adas.approach_pipeline',root/'approach_pipeline.py')
    identity=validate_bundle(root)  # before any GPIO/BAR output or NPU call
    gray_transform=None
    if args.native_tracking:
        from tracking_native import NativeTrackingGray
        gray_transform=NativeTrackingGray(root)
    if args.shared_tracking_input:
        if not args.native_tracking or not args.exact_postprocess:raise ValueError('shared input requires validated original tracking and postprocessing')
        shared_root=root/'stable-dual-20261103';sys.path.insert(0,str(shared_root))
        from stable_dual_input import StableDualSession
        from stable_tracking_cache import StableTrackingCache
        gray_transform=StableTrackingCache(gray_transform)
        OriginalVehicleSession=VehicleSession
        def shared_session(model_root,*,native=False,fused=False):
            if not native or not fused:raise ValueError('fixed stable fused model required')
            return StableDualSession(shared_root,model_root,gray_transform,base_class=OriginalVehicleSession)
        VehicleSession=shared_session
        exact_identity=dict(exact_identity,shared_tracking_input=True)
    if args.exact_tracking_math:
        if not args.exact_postprocess:raise ValueError('exact tracking math requires validated stable postprocessing')
        math_root=root/'stable-math-20261103';sys.path.insert(0,str(math_root))
        from stable_tracker_math import install_verified_math
        exact_identity=dict(exact_identity,tracking_math=install_verified_math(tracker_module,math_root))
    cv2.setNumThreads(2);AsyncLatestReader.live_mode=True;AsyncLatestReader.demand_driven=False;AsyncLatestReader.reports=[]
    stopping=threading.Event();finished=threading.Event();mutex=threading.Lock();latest=[];errors=[]
    counts=collections.Counter();display_ages=[];records_lock=threading.Lock()
    records=(args.output/'events.jsonl').open('x',encoding='utf-8')
    def record(value):
        with records_lock:
            records.write(json.dumps(value,default=lambda x:asdict(x) if is_dataclass(x) else str(x),allow_nan=False)+'\n');records.flush()
    def produce():
        try:
            source=board_root/'media/video_20260929_113343.mp4'
            tracker=tracker_module.ApproachProbe({0:'front'},flow_width=320,gray_transform=gray_transform)
            with VehicleSession(root,native=True,fused=True) as session,AsyncLatestReader(source) as reader:
                origin=time.perf_counter_ns()
                def read():
                    while not reader.ready:
                        if stopping.wait(.001):return None
                    return None if stopping.is_set() else reader.read()
                def neural(frame):
                    if frame.bgr.shape!=(1080,1920,3):raise ValueError('fixed native1080p frame required')
                    counts['neural_started']+=1;started=time.perf_counter_ns();raw=session.run_bgr3(frame.bgr)
                    boxes=decode_vehicles(raw,Mapping(1920,1080,640/1920,0,140,640,360))
                    counts['neural_completed']+=1
                    return started,boxes
                def track(frame,result):
                    started,boxes=result;report=tracker.update(frame,boxes);end=time.perf_counter_ns()
                    due=origin+frame.pts_us*1000
                    record(dict(kind='analysis',key=frame.key,pts_us=frame.pts_us,decoded_ns=frame.decoded_ns,
                                completed_ns=end,source_due_ns=due,report=report,boxes=boxes,total_ms=(end-started)/1e6))
                    counts['analysis']+=1;counts['analysis_warning']+=bool(report['risks'])
                    return frame,report,due,end
                iterator=pipeline.run(read,neural,track,limit=80)
                try:
                    for item in iterator:
                        if stopping.is_set():counts['not_published_after_stop']+=1;break
                        with mutex:
                            if latest:counts['gui_slot_overwritten']+=1
                            latest[:]=[item]
                finally:iterator.close()
        except Exception as exc:
            errors.append(type(exc).__name__+': '+str(exc));record(dict(kind='error',error=errors[-1]))
        finally:finished.set()
    lock=None;board=None;window=None;worker=None
    lease=RiskLease({0:'front'},freshness_ms=600);last=None;last_deadline=0;last_stale=False;previous=-1;confirmed=set();snapshot=None
    paint=PaintReceipt()
    start=time.perf_counter_ns()
    run_tag=start&0xffffffff
    def stop(*unused):stopping.set();lease.revoke(finished=True);paint.revoke()
    for sig in (signal.SIGTERM,signal.SIGINT,signal.SIGHUP):signal.signal(sig,stop)
    try:
        if args.led:
            import fcntl
            lock=open('/run/lock/adas-led.lock','a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
            board=ScenePcieLed();board.write(STOP)
        window=GtkWindow();window.root.set_title('RK3568 - single front visual approach - functional600ms')
        window.image.connect_after('draw',paint.painted)
        window.show(np.zeros((540,832,3),np.uint8));window.poll()
        record(dict(kind='start',assets=identity,tracking_backend=gray_transform.identity if gray_transform else 'opencv',single_camera_role='front',validity_ms=600,led_enabled=args.led,run_tag=run_tag,
                    driving_permissions=False,original_scene_changed=False,optical_latency_ms=None,exact_postprocess=exact_identity))
        worker=threading.Thread(target=produce,name='approach-two-slot-producer');worker.start()
        while not stopping.is_set():
            if window.poll()==27:break
            with mutex:item=latest.pop() if latest else None
            if item is not None:
                frame,report,due,completed=item;deadline=min(frame.decoded_ns,due)+600_000_000
                render_start=time.perf_counter_ns()
                warning=bool(lease.preview_mode(frame,report,source_due_ns=due,now_ns=render_start))
                lease.revoke()
                paint.revoke()
                if board is not None:board.write(STOP)
                marker=marker_for(run_tag,frame.key.frame_id) if warning else None
                # show_before also checks AFTER commit; a successful receipt
                # therefore retains >20ms for validation and the 10ms TTL guard.
                submitted_pixels=picture(frame,report,warning=warning,marker=marker)
                token=paint.begin(frame.key)
                shown=window.show_before(submitted_pixels,deadline-20_000_000)
                committed=time.perf_counter_ns();painted_ns=None
                if shown and warning:
                    painted_ns=wait_painted(window,paint,token,frame.key,deadline,stopping)
                    shown=painted_ns is not None
                submitted=time.perf_counter_ns()
                if stopping.is_set() or not window.is_open() or window.quit_requested:
                    lease.revoke(finished=True);shown=False
                lease.accept(frame,report,source_due_ns=due,submitted_ns=submitted,now_ns=submitted,gui_submitted=shown)
                counts['gui_accepted' if shown else 'gui_expired']+=1
                if shown:
                    counts['gui_warning']+=bool(lease.mode)
                    display_ages.append((submitted-due)/1e6)
                last=(frame,report);last_deadline=deadline;last_stale=not shown or warning and not lease.mode
                if last_stale and window.is_open():
                    paint.revoke();window.show(picture(frame,report,warning=False,stale=True))
                if lease.mode and painted_ns is not None and snapshot is None:
                    snapshot=submitted_pixels
                record(dict(kind='approach_display',key=frame.key,pts_us=frame.pts_us,decoded_ns=frame.decoded_ns,source_due_ns=due,
                            committed_ns=committed,gtk_painted_ns=painted_ns,submitted_ns=submitted,gui_submitted=shown,desktop_tag_verified=False,
                            run_tag=run_tag,
                            image_age_ms=(submitted-frame.decoded_ns)/1e6,
                            nominal_source_age_ms=(submitted-due)/1e6,render_commit_ms=(submitted-render_start)/1e6,
                            mode=lease.mode,reason=lease.reason,deadline_ns=lease.deadline_ns,neutralized=bool(last_stale)))
            now=time.perf_counter_ns();word=lease.word(now);mode=word&15
            if stopping.is_set() or not window.is_open() or window.quit_requested:lease.revoke(finished=True);word=STOP;mode=0
            readback=board.write(word) if board is not None else None
            if mode!=previous:
                record(dict(kind='approach_led',now_ns=now,mode=mode,command=hex(word),readback=readback));previous=mode
            if mode and readback is not None and readback[3]==mode and mode not in confirmed:
                confirmed.add(mode);record(dict(kind='approach_led_confirmed',now_ns=now,mode=mode,readback=readback))
            if last is not None and not last_stale and time.perf_counter_ns()>=last_deadline:
                paint.revoke();window.show(picture(*last,warning=False,stale=True));last_stale=True
            if finished.is_set() and item is None:
                with mutex:tail_pending=bool(latest)
                if not tail_pending:break
            stopping.wait(.008)
    finally:
        cleanup=cleanup_resources(stopping,lease,board,lock,window,worker);errors.extend(cleanup['errors'])
        with mutex:counts['gui_slot_left_on_exit']=len(latest)
        record(dict(kind='end',cleanup=cleanup,errors=errors))
        if not cleanup['worker_drained']:raise RuntimeError('active NPU/decoder owner did not drain; logger retained')
        records.close()
        # This second image is explicitly the submitted buffer, not a whole
        # desktop photograph or optical HDMI timing evidence.
        submitted_saved=snapshot is not None and cv2.imwrite(str(args.output/'submitted-warning.png'),snapshot,[cv2.IMWRITE_PNG_COMPRESSION,1])
        summary=dict(counts=dict(counts),tracking_backend=gray_transform.identity if gray_transform else 'opencv',native_tracking_calls=gray_transform.native_calls if gray_transform else 0,software_source_to_gui_ms=dict(p50=float(np.percentile(display_ages,50)),p95=float(np.percentile(display_ages,95))) if display_ages else None,
                     led_modes_confirmed=sorted(confirmed),gtk_draw_receipt=True,desktop_marker_saved=False,submitted_image_saved=bool(submitted_saved),
                     complete_desktop_saved=False,errors=errors,wall_seconds=(time.perf_counter_ns()-start)/1e9,
                     decoder=AsyncLatestReader.reports,single_front=True,functional_ms=600,latency_target_ms=300,
                     preview_interpolation='display-only-linear',optical_latency_ms=None,default_scene_changed=False,flash_written=False,exact_postprocess=exact_identity)
        (args.output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n',encoding='utf-8')
    return 1 if errors else 0


if __name__=='__main__':raise SystemExit(main())
