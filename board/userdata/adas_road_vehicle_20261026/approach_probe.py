"""PC-only visual approach evidence. No physical distance or hardware output.

Four-camera roles are explicit. Missing/ambiguous tracks and unusable background
motion revoke evidence; an empty risk list is not a guarantee of a clear road.
"""
import math
import cv2,numpy as np
from .contracts import Frame


def _iou(a,b):
    overlap=np.prod(np.maximum(0,np.minimum(a[2:],b[2:])-np.maximum(a[:2],b[:2])))
    return float(overlap/max(np.prod(a[2:]-a[:2])+np.prod(b[2:]-b[:2])-overlap,1e-9))


def _gray(frame,max_width):
    h,w=frame.bgr.shape[:2];width=min(max_width,w);height=max(1,round(h*width/w))
    return cv2.cvtColor(cv2.resize(frame.bgr,(width,height),interpolation=cv2.INTER_AREA),cv2.COLOR_BGR2GRAY)


def _exclude(shape,boxes,native):
    mask=np.full(shape,255,np.uint8);sy,sx=shape[0]/native[0],shape[1]/native[1]
    for box in boxes:
        x0,y0,x1,y1=box['xyxy'];pad=max(3,(x1-x0)*.08)
        cv2.rectangle(mask,(round((x0-pad)*sx),round((y0-pad)*sy)),
                      (round((x1+pad)*sx),round((y1+pad)*sy)),0,-1)
    return mask


def _background(previous,current,boxes,shape):
    old=previous['gray'];mask=_exclude(old.shape,previous['boxes'],shape)
    scale=old.shape[1]/640.;window=max(5,round(21*scale));window+=1-window%2
    points=cv2.goodFeaturesToTrack(old,maxCorners=240,qualityLevel=.02,minDistance=max(4,round(9*scale)),mask=mask)
    unavailable=dict(valid=False,scale=None,inliers=0,reason='insufficient_background')
    if points is None or len(points)<20:return None,unavailable
    target,status,error=cv2.calcOpticalFlowPyrLK(old,current,points,None,winSize=(window,window),maxLevel=3)
    if target is None:return None,unavailable
    first=points.reshape(-1,2);second=target.reshape(-1,2)
    good=(status.ravel()==1)&(error.ravel()<20)&np.isfinite(second).all(axis=1)
    current_mask=_exclude(current.shape,boxes,shape)
    indices=np.rint(np.nan_to_num(second,nan=-1)).astype(int)
    inside=(indices[:,0]>=0)&(indices[:,0]<current.shape[1])&(indices[:,1]>=0)&(indices[:,1]<current.shape[0])
    good &= inside
    good &= current_mask[indices[:,1].clip(0,current.shape[0]-1),indices[:,0].clip(0,current.shape[1]-1)]>0
    if good.sum()<20:return None,unavailable
    matrix,inliers=cv2.estimateAffinePartial2D(first[good],second[good],method=cv2.RANSAC,ransacReprojThreshold=2.5*scale,maxIters=800,confidence=.99)
    if matrix is None or inliers is None or not np.isfinite(matrix).all():return None,unavailable
    scale=math.hypot(matrix[0,0],matrix[1,0]);rotation=math.atan2(matrix[1,0],matrix[0,0])
    count=int(inliers.sum());valid=count>=20 and count/int(good.sum())>=.55 and .85<=scale<=1.18 and abs(rotation)<=.15
    result=dict(valid=bool(valid),scale=float(scale),rotation_rad=float(rotation),inliers=count,reason='estimated' if valid else 'unstable_background')
    if not valid:return None,result
    # Gray dimensions retain native aspect; translation must map back as well.
    matrix=matrix.copy();matrix[0,2]*=shape[1]/old.shape[1];matrix[1,2]*=shape[0]/old.shape[0]
    return matrix,result


def _prediction(box,matrix,scale):
    corners=np.array([[box[0],box[1]],[box[2],box[1]],[box[2],box[3]],[box[0],box[3]]])
    transformed=corners@matrix[:,:2].T+matrix[:,2]
    return np.r_[transformed.min(axis=0),transformed.max(axis=0)]


def _appearance(image,box):
    h,w=image.shape[:2];a,b,c,d=box['xyxy']
    patch=image[max(0,int(math.floor(b))):min(h,int(math.ceil(d))),max(0,int(math.floor(a))):min(w,int(math.ceil(c)))]
    hsv=cv2.cvtColor(patch,cv2.COLOR_BGR2HSV)
    hsv[:,:,0][hsv[:,:,1]<32]=0  # Hue is undefined for white/gray vehicles.
    histogram=cv2.calcHist([hsv],[0,1,2],None,[8,4,4],[0,180,0,256,0,256])
    return (histogram/max(float(histogram.sum()),1.)).ravel()


class ApproachProbe:
    def __init__(self,camera_roles,*,flow_width=640,gray_transform=None):
        if type(flow_width) is not int or flow_width not in (320,640):raise ValueError('explicit 320/640 background flow width required')
        if not isinstance(camera_roles,dict) or not camera_roles or any(type(k) is not int or not 0<=k<=3 or
                v not in ('front','rear','left','right') for k,v in camera_roles.items()):raise ValueError('explicit camera roles required')
        if gray_transform is not None and not callable(gray_transform):raise ValueError('callable exact gray transform required')
        self.roles=dict(camera_roles);self.flow_width=flow_width;self.gray_transform=gray_transform;self.cameras={};self.high={};self.next_id=1

    def _boxes(self,boxes,shape):
        h,w=shape;result=[]
        if type(boxes) not in (list,tuple):raise ValueError('explicit detection list required')
        for box in boxes:
            if not isinstance(box,dict):raise ValueError('typed detection required')
            xy=np.asarray(box.get('xyxy'),np.float64);score=box.get('score');label=box.get('label')
            if xy.shape!=(4,) or not np.isfinite(xy).all() or type(score) not in (int,float) or not math.isfinite(score) or not 0<=score<=1:
                raise ValueError('finite boxes and probability required')
            if not (0<=xy[0]<xy[2]<=w and 0<=xy[1]<xy[3]<=h):raise ValueError('native box outside frame')
            if score>=.5 and label in ('vehicle','person'):result.append(dict(xyxy=xy.tolist(),score=score,label=label))
        if len(result)>100:raise ValueError('bounded detections required')
        return result

    def update(self,frame,boxes):
        if not isinstance(frame,Frame) or frame.key.camera_id not in self.roles:raise ValueError('configured frame source required')
        camera=frame.key.camera_id;owner=(frame.key.epoch,frame.key.frame_id)
        previous=self.cameras.pop(camera,None);prior=self.high.get(camera)
        report=dict(key=frame.key,pts_us=frame.pts_us,status='pending',risks=[],tracks=[],background=dict(valid=False,scale=None,inliers=0,reason='first_frame'))
        if prior is not None and owner[0]<prior[0]:raise ValueError('old source epoch')
        if prior is not None and owner[0]==prior[0] and owner[1]<=prior[1]:
            report['status']='replayed_frame';return report
        self.high[camera]=owner
        if (not isinstance(frame.bgr,np.ndarray) or frame.bgr.dtype!=np.uint8 or frame.bgr.ndim!=3 or
                frame.bgr.shape[2]!=3 or min(frame.bgr.shape[:2])<1):raise ValueError('native uint8 BGR frame required')
        shape=frame.bgr.shape[:2]
        if frame.pts_kind!='decoder_reported' or type(frame.pts_us) is not int or frame.pts_us<0:
            report['status']='unknown_timestamp';return report
        boxes=self._boxes(boxes,shape);gray=(self.gray_transform or _gray)(frame,self.flow_width);matrix=None
        expected_width=min(self.flow_width,shape[1]);expected_height=max(1,round(shape[0]*expected_width/shape[1]))
        if not isinstance(gray,np.ndarray) or gray.dtype!=np.uint8 or gray.shape!=(expected_height,expected_width):
            raise ValueError('exact native-aspect uint8 tracking gray image required')
        continuous=(previous is not None and previous['epoch']==owner[0] and previous['shape']==shape and 0<frame.pts_us-previous['pts']<=350000)
        if continuous:
            matrix,report['background']=_background(previous,gray,boxes,shape)
        elif previous is not None:report['status']='discontinuity'
        old=previous['tracks'] if continuous and matrix is not None else []
        appearances=[_appearance(frame.bgr,box) for box in boxes]
        affinities=np.zeros((len(old),len(boxes)),np.float64)
        if old:
            scale=report['background']['scale']
            for i,track in enumerate(old):
                prediction=_prediction(np.array(track['box']['xyxy']),matrix,scale)
                for j,box in enumerate(boxes):
                    if (track['box']['label']==box['label'] and
                            cv2.compareHist(track['appearance'],appearances[j],cv2.HISTCMP_BHATTACHARYYA)<=.45):
                        affinities[i,j]=_iou(prediction,np.array(box['xyxy']))
        # Ambiguous rows OR columns start new histories, rather than walking
        # one vehicle's evidence into another at a crossing/duplicate box.
        eligible=affinities>=.3
        ambiguous_rows=eligible.sum(axis=1)>1;ambiguous_cols=eligible.sum(axis=0)>1
        associations={j:i for i,j in zip(*np.where(eligible)) if not ambiguous_rows[i] and not ambiguous_cols[j]}
        tracks=[];h,w=shape
        for j,box in enumerate(boxes):
            found=old[associations[j]] if j in associations else None
            if found is None:identifier=self.next_id;self.next_id+=1;history=[]
            else:
                identifier=found['id'];scale=report['background']['scale']
                # Express the bounded history in CURRENT background coordinates;
                # avoid a drifting cumulative zoom over hours of playback.
                history=[dict(row,height=row['height']*scale,width=row['width']*scale,area=row['area']*scale*scale) for row in found['history']]
            xy=np.array(box['xyxy']);bh=xy[3]-xy[1];area=np.prod(xy[2:]-xy[:2])
            center=(xy[:2]+xy[2:])/2
            displacement=0.
            if found is not None:
                before=np.array(found['box']['xyxy']);predicted=_prediction(before,matrix,report['background']['scale'])
                displacement=history[-1]['displacement']+(xy[3]-predicted[3])/h
            rotation=abs(report['background'].get('rotation_rad',0.)) if matrix is not None else 0.
            history.append(dict(pts=frame.pts_us,height=bh,width=xy[2]-xy[0],area=area,displacement=float(displacement),rotation=rotation))
            history=[row for row in history if frame.pts_us-row['pts']<=1200000][-32:]
            tracks.append(dict(id=identifier,box=box,history=history,appearance=appearances[j]))
            span=(history[-1]['pts']-history[0]['pts'])/1e6
            ratio=history[-1]['height']/history[0]['height'];area_ratio=history[-1]['area']/history[0]['area']
            width_ratio=history[-1]['width']/history[0]['width']
            rotation_span=sum(row['rotation'] for row in history[1:])
            rotation_supported=rotation_span<=math.radians(2)
            rate=math.log(ratio)/span if span>0 else 0.
            increments=np.diff([row['height'] for row in history]);positive=float(np.mean(increments>0)) if len(increments) else 0.
            rates=[math.log(b['height']/a['height'])/((b['pts']-a['pts'])/1e6) for a,b in zip(history,history[1:])]
            median_rate=float(np.median(rates)) if rates else 0.
            near=area/(h*w)>=.015 and xy[3]>=.65*h and .28*w<=center[0]<=.72*w
            complete=xy[0]>.01*w and xy[2]<.99*w and xy[3]<.98*h
            approaching=(len(history)>=4 and span>=.45 and ratio>=1.2 and width_ratio>=1.15 and area_ratio>=1.25 and rate>=.25 and rotation_supported and
                         median_rate>=.15 and positive>=.75 and displacement-history[0]['displacement']>=.01 and near and complete and matrix is not None)
            row=dict(id=identifier,label=box['label'],xyxy=box['xyxy'],observations=len(history),span_seconds=span,
                compensated_height_ratio=ratio,compensated_width_ratio=width_ratio,compensated_log_height_rate=rate,median_pair_log_height_rate=median_rate,background_compensated=matrix is not None,
                rotation_supported=rotation_supported,rotation_span_degrees=math.degrees(rotation_span),
                near=bool(near),approaching=bool(approaching))
            report['tracks'].append(row)
            if approaching and self.roles[camera] not in report['risks']:report['risks'].append(self.roles[camera])
        if continuous:report['status']='observed' if matrix is not None else 'unknown_background'
        if any(t['near'] and not t['rotation_supported'] for t in report['tracks']):report['status']='unknown_rotation'
        self.cameras[camera]=dict(epoch=owner[0],frame_id=owner[1],pts=frame.pts_us,shape=shape,gray=gray,
                                  boxes=boxes,tracks=tracks)
        return report
