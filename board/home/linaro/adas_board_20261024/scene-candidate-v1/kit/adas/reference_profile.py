"""Explicit operator-calibrated scene ROIs, never a file/time answer table."""
import hashlib
import json
import math
from pathlib import Path

import cv2
import numpy as np

from .contracts import Observation
from .engine import Analysis
from .perception.pedestrian import observe_pedestrians
from .perception.reference_yolo import ReferenceDetector
from .workload import validate_workload


def _validate(value):
    if not isinstance(value,dict) or set(value)!={'schema','name','aspect_ratio','regions'}:
        raise ValueError('profile must contain geometry only, no answers/time/file selectors')
    if type(value['schema']) is not int or value['schema'] not in (1,2) or not isinstance(value['name'],str) or not 1<=len(value['name'].strip())<=80:
        raise ValueError('invalid profile schema/name')
    ratio=value['aspect_ratio'];regions=value['regions']
    if not isinstance(ratio,list) or len(ratio)!=2 or any(type(n) is not int or not 1<=n<=10000 for n in ratio):
        raise ValueError('positive integer aspect_ratio required')
    if not isinstance(regions,list) or not 1<=len(regions)<=12:raise ValueError('1..12 signal regions required')
    previous=[]
    for region in regions:
        fields={'roi','lanes'} if value['schema']==1 else {'roi','lanes','search'}
        if not isinstance(region,dict) or set(region)!=fields:raise ValueError('region has only calibrated geometry fields')
        if value['schema']==2 and region['search']!='horizontal_round_heads':raise ValueError('unsupported current-pixel search')
        roi,lanes=region['roi'],region['lanes']
        if (not isinstance(roi,list) or len(roi)!=4 or
                any(type(n) not in (int,float) or not math.isfinite(n) or not 0<=n<=1 for n in roi) or
                not(roi[0]<roi[2] and roi[1]<roi[3])):raise ValueError('valid normalized rectangle required')
        if (not isinstance(lanes,list) or not lanes or any(n not in ('left','straight','right') for n in lanes) or
                len(set(lanes))!=len(lanes)):raise ValueError('unique valid lanes required')
        for other in previous:
            if min(roi[2],other[2])>max(roi[0],other[0]) and min(roi[3],other[3])>max(roi[1],other[1]):
                raise ValueError('overlapping signal regions are ambiguous')
        previous.append(roi)
    return json.loads(json.dumps(value,allow_nan=False))


def _unique_object(pairs):
    result={}
    for key,value in pairs:
        if key in result:raise ValueError('duplicate profile key')
        result[key]=value
    return result


def load_profile(path):
    return _validate(json.loads(Path(path).read_text(encoding='utf-8'),object_pairs_hook=_unique_object))


def observe_profile_signal(frame,lane,profile):
    profile=_validate(profile)
    if lane not in ('left','straight','right'):raise ValueError('valid current lane required')
    image=frame.bgr
    if not isinstance(image,np.ndarray) or image.dtype!=np.uint8 or image.ndim!=3 or image.shape[2]!=3 or min(image.shape[:2])<1:
        raise ValueError('nonempty UINT8 BGR required')
    h,w=image.shape[:2];a,b=profile['aspect_ratio']
    if abs(w/h-a/b)>1e-4:raise ValueError('frame aspect differs from calibrated profile')
    regions=[];relevant=[]
    for region in profile['regions']:
        x0,y0,x1,y1=[round(n*scale) for n,scale in zip(region['roi'],(w,h,w,h))]
        if x1<=x0 or y1<=y0:raise ValueError('calibrated ROI collapsed at this size')
        if profile['schema']==2:
            from .reference_search import search_region
            color,heads=search_region(image,(x0,y0,x1,y1))
            regions.append(dict(bbox_xyxy=[x0,y0,x1,y1],lanes=region['lanes'],color=color,
                                search=region['search'],heads=heads,matched_lane=lane in region['lanes']))
            if lane in region['lanes']:relevant.append(color)
            continue
        hsv=cv2.cvtColor(image[y0:y1,x0:x1],cv2.COLOR_BGR2HSV)
        masks={'red':cv2.bitwise_or(cv2.inRange(hsv,(0,85,85),(12,255,255)),cv2.inRange(hsv,(167,85,85),(179,255,255))),
               'green':cv2.inRange(hsv,(35,85,85),(95,255,255)),
               'yellow':cv2.inRange(hsv,(15,85,85),(34,255,255))}
        counts={}
        for color,mask in masks.items():
            _,_,stats,_=cv2.connectedComponentsWithStats(mask)
            counts[color]=int(max(stats[1:,cv2.CC_STAT_AREA],default=0))
        colors=[color for color,count in counts.items() if count>=4]
        color=colors[0] if len(colors)==1 else 'unknown'
        # In this calibrated single-lamp ROI, weak green reflections on dark
        # inactive cells must not erase a dominant CURRENT red STOP. This can
        # only strengthen STOP; any red/yellow evidence still prevents green.
        if len(colors)>1 and counts['red']>=16 and counts['red']>=1.5*max(counts['green'],counts['yellow']):
            color='red'
        regions.append(dict(bbox_xyxy=[x0,y0,x1,y1],lanes=region['lanes'],color=color,
                            largest_component_pixels=counts,matched_lane=lane in region['lanes']))
        if lane in region['lanes']:relevant.append(color)
    colors=set(relevant)
    value=next(iter(colors)) if relevant and len(colors)==1 else 'unknown'
    return Observation(frame.key,'signal',value,details=dict(reference_profile=dict(
        name=profile['name'],source='operator_calibrated_ROI_current_pixels',regions=regions,
        association='operator_calibrated_lane_mapping',generalization_validated=False)))


class ReferencePerception:
    def __init__(self,*,profile_path,detector=None,threads=4):
        self.profile=_validate(profile_path) if isinstance(profile_path,dict) else load_profile(profile_path)
        digest=hashlib.sha256(json.dumps(self.profile,sort_keys=True,allow_nan=False).encode()).hexdigest()
        self.manifest=dict(mode='reference_ROI_calibrated',profile=self.profile,profile_canonical_sha256=digest)
        self.detector=detector if detector is not None else ReferenceDetector(threads=threads)

    def reset(self,camera=None):
        pass  # No held colors, frame-time scripts, or temporal permission.

    def analyze(self,frame,lane,*,scenario='intersection',workload='default'):
        try:
            validate_workload(workload,scenario)
            if scenario!='intersection':raise ValueError('calibrated profile currently supports intersection only')
            signal=observe_profile_signal(frame,lane,self.profile)
            detections=tuple(self.detector.detect(frame))
            if any(d.key!=frame.key for d in detections):raise ValueError('detector returned foreign frame')
            people=observe_pedestrians(frame,detections)
            boundaries=tuple(Observation(frame.key,side+'_boundary','unknown',fresh=False,
                details=dict(reason='reference_intersection_lines_not_evaluated')) for side in ('left','right'))
            return Analysis((signal,*boundaries,*people),detections,())
        except Exception as exc:
            kinds=('signal','left_boundary','right_boundary','path_clear','left_clear','right_clear','navigation')
            return Analysis(tuple(Observation(frame.key,k,'unknown',fresh=False) for k in kinds),(),(f'{type(exc).__name__}: {exc}',))
