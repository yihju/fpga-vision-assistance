"""Opt-in current-pixel round heads; geometry is not neural confidence.

The frozen diagnostic remains immutable. No temporal state, model calls,
or production default changes live here.
"""
import math
import cv2
import numpy as np
from .reference_yolo import Detection
from .signal import observe_signals


def find_heads(image):
    if (not isinstance(image,np.ndarray) or image.dtype!=np.uint8 or image.ndim!=3 or
            image.shape[2]!=3 or min(image.shape[:2])<1):
        raise ValueError('nonempty UINT8 BGR required')
    height,width=image.shape[:2];band=math.ceil(height*.7)
    hsv=cv2.cvtColor(image[:band],cv2.COLOR_BGR2HSV);value=hsv[:,:,2]
    _,_,dark_stats,_=cv2.connectedComponentsWithStats(cv2.inRange(hsv,(0,0,0),(179,255,109)))
    # Bulk-reject component stats before Python/contour work. Same predicates,
    # label order and float64 arithmetic as the scalar reference.
    x,y,w,h,_=dark_stats[1:].astype(np.int64).T
    aspect=np.maximum(w,h)/np.minimum(w,h)
    keep=(x>0)&(y>0)&(x+w<width)&(y+h<band)&(aspect>=2)&(aspect<=6)
    dark_heads=[tuple(map(int,row[:4])) for row in dark_stats[1:][keep]]
    # inRange fuses the exact UINT8 hue/S/V predicates without split copies
    # and repeated full-band numpy temporaries. Nonzero masks remain identical.
    colors={'red':cv2.bitwise_or(cv2.inRange(hsv,(0,85,85),(12,255,255)),
                                cv2.inRange(hsv,(167,85,85),(179,255,255))),
            'green':cv2.inRange(hsv,(35,85,85),(95,255,255)),
            'yellow':cv2.inRange(hsv,(15,85,85),(34,255,255))}
    heads=[]
    for color,color_mask in colors.items():
        count,labels,stats,_=cv2.connectedComponentsWithStats(color_mask)
        x,y,w,h,area=stats[1:].astype(np.int64).T
        aspect=w/h;fill=area/(w*h)
        keep=((x>0)&(y>0)&(x+w<width)&(y+h<band)&(area>=12)&(area<=2500)&
              (w>=5)&(w<=80)&(h>=5)&(h<=80)&(aspect>=.65)&(aspect<=1.5)&(fill>=.5)&(fill<=.9))
        for component in np.flatnonzero(keep)+1:
            x,y,w,h,area=map(int,stats[component]);diameter=max(w,h)
            mask=(labels[y:y+h,x:x+w]==component).astype(np.uint8)
            contours,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
            contour=max(contours,key=cv2.contourArea);perimeter=cv2.arcLength(contour,True)
            circularity=4*math.pi*cv2.contourArea(contour)/max(1.,perimeter**2)
            if circularity<.65:continue
            cx,cy=x+(w-1)/2,y+(h-1)/2
            supports=[]
            for hx,hy,hw,hh in dark_heads:
                if not(hx<x and hy<y and x+w<hx+hw and y+h<hy+hh):continue
                horizontal=hw>=hh;length,thickness=(hw,hh) if horizontal else (hh,hw)
                if not(1.15<=thickness/diameter<=3 and 2.6<=length/diameter<=6.5):continue
                centers=([(hx+hh/2,hy+hh/2),(hx+hw/2,hy+hh/2),(hx+hw-hh/2,hy+hh/2)] if horizontal else
                         [(hx+hw/2,hy+hw/2),(hx+hw/2,hy+hh/2),(hx+hw/2,hy+hh-hw/2)])
                distances=[math.hypot(cx-a,cy-b) for a,b in centers];slot=int(np.argmin(distances))
                if distances[slot]>.4*diameter:continue
                fractions=[]
                for index,(a,b) in enumerate(centers):
                    if index==slot:continue
                    radius=.35*diameter
                    x0,x1=math.floor(a-radius),math.ceil(a+radius)+1
                    y0,y1=math.floor(b-radius),math.ceil(b+radius)+1
                    if x0<hx or y0<hy or x1>hx+hw or y1>hy+hh:break
                    yy,xx=np.ogrid[y0:y1,x0:x1];disk=(xx-a)**2+(yy-b)**2<=radius**2
                    fractions.append(float(np.mean(value[y0:y1,x0:x1][disk]<110)))
                if len(fractions)==2 and min(fractions)>=.55:
                    supports.append(dict(head_bbox_xyxy=[hx,hy,hx+hw,hy+hh],axis='horizontal' if horizontal else 'vertical',
                                         dark_cell_fractions=fractions,slot=slot))
            if len(supports)!=1:continue
            heads.append(dict(color=color,bbox_xyxy=[x,y,x+w,y+h],source='current_pixel_head_geometry',
                              area=area,circularity=circularity,fill=area/(w*h),**supports[0]))
    return sorted(heads,key=lambda row:(row['bbox_xyxy'][1],row['bbox_xyxy'][0],row['color']))


def refine_signal(frame,detections,lane,baseline):
    if (baseline.key!=frame.key or baseline.kind!='signal' or
            any(d.key!=frame.key for d in detections)):
        raise ValueError('current signal and same-frame detections required')
    if lane not in ('left','straight','right'):raise ValueError('valid lane required')
    if baseline.value!='unknown' or not baseline.fresh or not baseline.applicable:return baseline
    heads=find_heads(frame.bgr)
    if not heads:return baseline
    h,w=frame.bgr.shape[:2];geometry=[]
    for head in heads:
        x0,y0,x1,y1=head['bbox_xyxy']
        box=(max(0,x0-2),max(0,y0-2),min(w,x1+2),min(h,y1+2))
        geometry.append(Detection(frame.key,head['color'],head['circularity'],box))
    # Do NOT call/update the temporal tracker twice or add these to NN outputs.
    signal=observe_signals(frame,[*detections,*geometry],lane,required=True)
    signal.details['lamp_head_geometry']=dict(experimental=True,heads=heads,
        source='current_pixels_only',confidence_kind='geometric_circularity_not_neural',baseline_value=baseline.value)
    return signal
