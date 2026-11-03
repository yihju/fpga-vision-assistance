"""Opt-in consistency across bounded CURRENT analyzed samples, not held classes.

Skipped raw video frames remain unobserved. Decoder PTS and monotonic decode
timestamps both bound history; neither is a physical/optical latency proof.
"""
from ..contracts import Observation
from .lane_sequence import LaneConsistency


class SampledLaneConsistency(LaneConsistency):
    def filter(self,frame,boundaries,*,domain='full'):
        camera=frame.key.camera_id
        if (domain!='full' or len(boundaries)!=2 or
                [o.kind for o in boundaries]!=['left_boundary','right_boundary'] or
                any(o.key!=frame.key or o.value not in ('solid','dashed','unknown') for o in boundaries)):
            self.reset(camera)
            raise ValueError('sampled lane requires same-frame full-domain boundaries')
        pts,decoded,shape=frame.pts_us,frame.decoded_ns,frame.bgr.shape[:2]
        previous=self._cameras.pop(camera,None)
        still=frame.pts_kind=='not_applicable_image' and pts is None
        timed=(frame.pts_kind=='decoder_reported' and type(pts) is int and pts>=0 and
               type(decoded) is int and decoded>0)
        linked=(timed and previous is not None and previous['epoch']==frame.key.epoch and
                frame.key.frame_id>previous['frame_id'] and previous['shape']==shape and
                0<pts-previous['pts']<=300000 and 0<decoded-previous['decoded']<=300000000)
        history=previous['sides'] if linked else {}
        current,output={},[]
        for observed in boundaries:
            value,count,span,decode_span=observed.value,0,None,None
            status='current_non_dashed';evidence=[]
            if still:status='still_stateless'
            elif observed.value=='dashed':
                value='unknown';status='no_sample_timestamp' if not timed else 'current_unavailable'
                if timed and observed.fresh is True and observed.applicable is True:
                    geometry=self._geometry(observed,shape);status='geometry_unknown'
                    if geometry is not None:
                        older=[p for p in history.get(observed.kind,[])[-2:]
                               if 0<pts-p['pts']<=600000 and 0<decoded-p['decoded']<=600000000]
                        if not all(self._same_boundary(p['geometry'],geometry,shape) for p in older):older=[]
                        evidence=[*older,dict(geometry=geometry,pts=pts,decoded=decoded,frame_id=frame.key.frame_id)][-3:]
                        current[observed.kind]=evidence;count=len(evidence)
                        span=pts-evidence[0]['pts'];decode_span=(decoded-evidence[0]['decoded'])/1e6
                        status='confirmed' if count==3 else 'pending'
                        if count==3:value='dashed'
            temporal=dict(raw_value=observed.value,status=status,count=count,required=3,
                source_span_us=span,source_clock='decoder_PTS_and_monotonic_decode_not_optical',
                historical_value_substitution=False,mode='bounded_current_samples',
                step_limit_ms=300,span_limit_ms=600,decoded_span_ms=decode_span,
                evidence_frame_ids=[p['frame_id'] for p in evidence],
                evidence_pts_us=[p['pts'] for p in evidence],evidence_decoded_ns=[p['decoded'] for p in evidence])
            output.append(Observation(observed.key,observed.kind,value,observed.applicable,observed.fresh,
                                      dict(observed.details,temporal=temporal)))
        if timed:self._cameras[camera]=dict(epoch=frame.key.epoch,frame_id=frame.key.frame_id,
            shape=shape,pts=pts,decoded=decoded,sides=current)
        return tuple(output)
