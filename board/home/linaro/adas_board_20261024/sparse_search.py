# Generated from approved reference_search: only prefilter loops vectorized.
SOURCE_SHA256='58ad752843309ab076cb27060c6736635b9eaae4c6f38f9b86880367dc01755c'
"""Native bloom-tolerant housing search, only inside a calibrated corridor."""
import math
import cv2
import numpy as np


def search_region(image,bbox):
    value,heads=_search_region(image,bbox,reconstruct_red=False)
    if heads and value!='green':return value,heads
    # A strict green may coexist with a weak but seeded prohibited red. Never
    # let the successful green search bypass that current-frame veto check.
    repaired_value,repaired_heads=_search_region(image,bbox,reconstruct_red=True)
    if not repaired_heads:return value,heads
    if heads:return 'unknown',heads+repaired_heads
    return repaired_value,repaired_heads


def _search_region(image,bbox,*,reconstruct_red):
    x0,y0,x1,y1=bbox
    crop=image[y0:y1,x0:x1]
    height,width=crop.shape[:2];hsv=cv2.cvtColor(crop,cv2.COLOR_BGR2HSV)
    dark_mask=cv2.inRange(hsv,(0,0,0),(179,255,109))
    masks={'red':cv2.bitwise_or(cv2.inRange(hsv,(0,85,85),(12,255,255)),cv2.inRange(hsv,(167,85,85),(179,255,255))),
           'green':cv2.inRange(hsv,(35,85,85),(95,255,255)),
           'yellow':cv2.inRange(hsv,(15,85,85),(34,255,255))}
    strong_red=masks['red']
    if reconstruct_red:
        weak=cv2.inRange(hsv,(0,35,85),(12,255,255))|cv2.inRange(hsv,(167,35,85),(179,255,255))
        count,weak_labels,weak_stats,_=cv2.connectedComponentsWithStats(weak)
        seeds=np.bincount(weak_labels[strong_red>0],minlength=count)
        allowed=(seeds>=8)&(seeds>=.1*weak_stats[:,cv2.CC_STAT_AREA])
        allowed[0]=False
        repaired=np.where(allowed[weak_labels],255,0).astype(np.uint8)
        if not np.any(repaired):return 'unknown',[]
        masks={'red':repaired,'green':np.zeros_like(repaired),'yellow':np.zeros_like(repaired)}
    # Bloom can split the dark housing into two components. Joining only the
    # CURRENT colored pixels recovers its envelope; two inactive cells must
    # still be genuinely dark in the unmodified HSV, never in this union mask.
    union=dark_mask|masks['red']|masks['green']|masks['yellow']
    heads=[]
    for color,mask in masks.items():
        _,labels,stats,_=cv2.connectedComponentsWithStats(mask)
        for component in _component_ids(stats,width,height):
            x,y,w,h,area=map(int,stats[component]);diameter=max(w,h)
            if not (x>0 and y>0 and x+w<width and y+h<height and 12<=area<=2500 and
                    5<=w<=80 and 5<=h<=80 and .5<=w/h<=1.8 and .4<=area/(w*h)<=.95):continue
            local=(labels[y:y+h,x:x+w]==component).astype(np.uint8)
            contours,_=cv2.findContours(local,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
            contour=max(contours,key=cv2.contourArea);perimeter=cv2.arcLength(contour,True)
            circularity=4*math.pi*cv2.contourArea(contour)/max(1.,perimeter**2)
            if circularity<.45:continue
            housings,merged=_local_housings(dark_mask,union,x,y,w,h,diameter)
            cx,cy=x+(w-1)/2,y+(h-1)/2;slot={'red':0,'yellow':1,'green':2}[color];supports=[]
            # Prefer the unmodified dark envelope. Only fall back when it has
            # no supporting housing, never to bypass multiple supports.
            for index,(hx,hy,hw,hh) in enumerate(housings+merged):
                if index==len(housings) and supports:break
                if not(.7<=hh/diameter<=3 and 2.3<=hw/diameter<=6.5):continue
                margin=.3*diameter
                if not(hx-margin<=x and hy-margin<=y and x+w<=hx+hw+margin and y+h<=hy+hh+margin):continue
                centers=[(hx+hh/2,hy+hh/2),(hx+hw/2,hy+hh/2),(hx+hw-hh/2,hy+hh/2)]
                if math.hypot(cx-centers[slot][0],cy-centers[slot][1])>.65*diameter:continue
                fractions=[];middle_red=None;middle_unexplained=None
                for index,(a,b) in enumerate(centers):
                    if index==slot:continue
                    radius=.35*diameter
                    xa,xb=math.floor(a-radius),math.ceil(a+radius)+1
                    ya,yb=math.floor(b-radius),math.ceil(b+radius)+1
                    if xa<0 or ya<0 or xb>width or yb>height:break
                    yy,xx=np.ogrid[ya:yb,xa:xb];disk=(xx-a)**2+(yy-b)**2<=radius**2
                    dark=hsv[ya:yb,xa:xb,2]<110
                    fractions.append(float(np.mean(dark[disk])))
                    if color=='red' and index==1:
                        current_red=strong_red[ya:yb,xa:xb]>0
                        middle_red=float(np.mean(current_red[disk]))
                        middle_unexplained=float(np.mean((~dark & ~current_red)[disk]))
                strict=(circularity>=.65 and 1.15<=hh/diameter and
                        hx<x and hy<y and x+w<hx+hw and y+h<hy+hh)
                # Relaxed glow geometry needs stronger independent inactive-cell
                # evidence; a half-dark building window must not become green.
                minimum_dark=.55 if strict else .75
                # Some complete vehicle heads put a same-red countdown in the
                # middle cell. This is additional STOP evidence only: require
                # the original strict first-slot red circle and a dark last
                # cell; never relax green/yellow or weak-red reconstruction.
                countdown=(not reconstruct_red and color=='red' and strict and len(fractions)==2 and
                           min(fractions)<minimum_dark and fractions[1]>=.75 and
                           middle_red is not None and middle_red>=.15 and middle_unexplained<=.1)
                if len(fractions)==2 and (min(fractions)>=minimum_dark or countdown):
                    evidence=(dict(red_countdown_current_pixels=True,middle_red_fraction=middle_red,
                                   middle_unexplained_fraction=middle_unexplained) if countdown else {})
                    candidate=(hx,hy,hw,hh,fractions,evidence)
                    if not any(c[:4]==candidate[:4] for c in supports):supports.append(candidate)
            if len(supports)!=1:continue
            hx,hy,hw,hh,fractions,evidence=supports[0]
            heads.append(dict(color=color,bbox_xyxy=[x0+x,y0+y,x0+x+w,y0+y+h],
                head_bbox_xyxy=[x0+hx,y0+hy,x0+hx+hw,y0+hy+hh],axis='horizontal',slot=slot,
                area=area,circularity=circularity,fill=area/(w*h),dark_cell_fractions=fractions,
                source='calibrated_native_bloom_housing',search_scale=1,**evidence))
            if reconstruct_red:
                heads[-1].update(source='calibrated_native_seeded_red_reconstruction',
                    strong_seed_pixels=int(np.count_nonzero((labels[y:y+h,x:x+w]==component)&
                                                           (strong_red[y:y+h,x:x+w]>0))))
    colors={h['color'] for h in heads}
    return (next(iter(colors)) if len(colors)==1 else 'unknown'),heads

def _housing_rows(s,width,height):
    if not len(s):return s
    x,y,w,h,a=s.T
    return s[(x>0)&(y>0)&(x+w<width)&(y+h<height)&(w>=2*h)&(w<=6*h)]

def _component_ids(stats,width,height):
    if len(stats)<=1:return []
    x,y,w,h,a=stats[1:].T
    keep=(x>0)&(y>0)&(x+w<width)&(y+h<height)&(a>=12)&(a<=2500)&(w>=5)&(w<=80)&(h>=5)&(h<=80)&(w/h>=.5)&(w/h<=1.8)&(a/(w*h)>=.4)&(a/(w*h)<=.95)
    return np.flatnonzero(keep)+1

def _local_housings(dark,union,x,y,w,h,diameter):
    # Any eligible support has width<=6.5*d, height<=3*d and containment
    # margin .3*d. Therefore it is strictly inside this 7*d / 4*d window.
    # A connected component touching a newly cropped edge cannot be eligible
    # globally; reject it rather than manufacturing a truncated housing.
    height,width=dark.shape
    xa,xb=max(0,x-7*diameter),min(width,x+w+7*diameter)
    ya,yb=max(0,y-4*diameter),min(height,y+h+4*diameter)
    groups=[]
    for mask in (dark,union):
        stats=cv2.connectedComponentsWithStats(mask[ya:yb,xa:xb])[2][1:]
        groups.append([(int(hx)+xa,int(hy)+ya,int(hw),int(hh))
            for hx,hy,hw,hh,area in _housing_rows(stats,xb-xa,yb-ya)])
    return groups
