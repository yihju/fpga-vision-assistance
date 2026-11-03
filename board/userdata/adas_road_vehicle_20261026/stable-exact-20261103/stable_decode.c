/* Same probability head and selection order; original Python geometry/NMS remain. */
#include <math.h>
#include <stdint.h>
#include <stddef.h>

int stable_select(const float *rows,size_t count,float confidence,float left,float top,float width,float height,
                  uint32_t *indices,float *scores,size_t capacity){
    if(!rows||!indices||!scores||count>25200||capacity<count||!(confidence>0.f&&confidence<1.f)||
       !(width>0.f&&height>0.f))return -1;
    int kept=0;
    for(size_t i=0;i<count;i++){
        const float *r=rows+6*i;
        for(int k=0;k<6;k++)if(!isfinite(r[k]))return -2;
        if(r[4]<0.f||r[4]>1.f||r[5]<0.f||r[5]>1.f)return -3;
        float score=r[4]*r[5];
        if(score>=confidence&&r[2]>0.f&&r[3]>0.f&&r[0]>=left&&r[0]<left+width&&r[1]>=top&&r[1]<top+height){
            indices[kept]=(uint32_t)i;scores[kept]=score;kept++;
        }
    }
    return kept;
}
