"""Vehicle box mapping, preprocessing and non-maximum suppression."""
from dataclasses import dataclass
import cv2
import numpy as np

SIZE=640


@dataclass(frozen=True)
class Mapping:
    native_width:int
    native_height:int
    ratio:float
    left:int
    top:int
    width:int
    height:int

    def native_boxes(self,boxes):
        result=np.asarray(boxes,np.float32).copy()
        result[:,[0,2]]=(result[:,[0,2]]-self.left)/self.ratio
        result[:,[1,3]]=(result[:,[1,3]]-self.top)/self.ratio
        result[:,[0,2]]=np.clip(result[:,[0,2]],0,self.native_width)
        result[:,[1,3]]=np.clip(result[:,[1,3]],0,self.native_height)
        return result

    def native_mask(self,logits):
        cropped=logits[0,:,self.top:self.top+self.height,self.left:self.left+self.width]
        mask=np.argmax(cropped,axis=0).astype(np.uint8)*255
        return cv2.resize(mask,(self.native_width,self.native_height),interpolation=cv2.INTER_NEAREST)


def prepare_rgb(image):
    if not isinstance(image,np.ndarray) or image.ndim!=3 or image.shape[2]!=3 or image.dtype!=np.uint8:
        raise ValueError('native uint8 BGR image required')
    h,w=image.shape[:2]
    if h<1 or w<1:raise ValueError('empty native image')
    ratio=min(SIZE/w,SIZE/h);width,height=round(w*ratio),round(h*ratio)
    if width<1 or height<1:raise ValueError('unsupported aspect ratio')
    left,top=(SIZE-width)//2,(SIZE-height)//2
    canvas=np.full((SIZE,SIZE,3),114,np.uint8)
    # Channel permutation commutes with per-channel INTER_AREA.
    small=cv2.resize(image,(width,height),interpolation=cv2.INTER_AREA)
    canvas[top:top+height,left:left+width]=cv2.cvtColor(small,cv2.COLOR_BGR2RGB)
    return canvas,Mapping(w,h,ratio,left,top,width,height)




def _nms(boxes,scores,threshold):
    order=np.argsort(-scores,kind='stable')[:3000];keep=[]
    areas=np.prod(np.maximum(0,boxes[:,2:]-boxes[:,:2]),axis=1)
    while len(order) and len(keep)<100:
        index=order[0];keep.append(index);rest=order[1:]
        overlap=np.prod(np.maximum(0,np.minimum(boxes[index,2:],boxes[rest,2:])-np.maximum(boxes[index,:2],boxes[rest,:2])),axis=1)
        iou=overlap/np.maximum(areas[index]+areas[rest]-overlap,1e-9)
        order=rest[iou<=threshold]
    return keep


def decode_vehicles(detections,mapping,*,confidence=.25,nms_iou=.45):
    detections=np.asarray(detections)
    if not 0<confidence<1 or not 0<nms_iou<1 or detections.ndim!=3 or detections.shape[0]!=1 or detections.shape[2]!=6:
        raise ValueError('single vehicle head shape required')
    if not np.isfinite(detections).all():raise ValueError('nonfinite vehicle output')
    rows=detections[0]
    if len(rows) and (np.any(rows[:,4:]<0) or np.any(rows[:,4:]>1)):raise ValueError('decoded probability head required')
    score=rows[:,4]*rows[:,5]
    valid=((score>=confidence)&(rows[:,2]>0)&(rows[:,3]>0)&
        (rows[:,0]>=mapping.left)&(rows[:,0]<mapping.left+mapping.width)&
        (rows[:,1]>=mapping.top)&(rows[:,1]<mapping.top+mapping.height))
    rows,score=rows[valid],score[valid]
    boxes=np.concatenate((rows[:,:2]-rows[:,2:4]/2,rows[:,:2]+rows[:,2:4]/2),axis=1)
    boxes=mapping.native_boxes(boxes)
    keep=_nms(boxes,score,nms_iou)
    return [dict(xyxy=boxes[i].tolist(),score=float(score[i]),label='vehicle') for i in keep]




