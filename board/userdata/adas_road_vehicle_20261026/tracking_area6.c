/* Diagnostic exact integer-area kernel for the measured 1080p tracking path. */
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#if defined(__aarch64__)
#include <arm_neon.h>
#endif

/* Build the complete finite rounding map once, before dlopen returns. This
 * preserves OpenCV float-scale/ties-to-even results without 172800 libm calls. */
static uint8_t average36[36*255+1];
__attribute__((constructor)) static void initialize_average36(void){
    const float scale=1.0f/36.0f;
    for(int sum=0;sum<=36*255;++sum)average36[sum]=(uint8_t)lrintf((float)sum*scale);
}

int tracking_area6(const uint8_t *source,size_t bytes,uint8_t *target,size_t capacity){
    if(!source||!target||bytes!=1920u*1080u*3u||capacity!=320u*180u*3u)return -1;
    for(int y=0;y<180;++y){
#if defined(__aarch64__)
        uint16_t vertical[1920*3];
        const uint8_t *rows=source+(size_t)y*6*1920*3;
        for(int i=0;i<1920*3;i+=16){
            uint8x16_t a=vld1q_u8(rows+i),b=vld1q_u8(rows+1920*3+i);
            uint16x8_t low=vaddl_u8(vget_low_u8(a),vget_low_u8(b));
            uint16x8_t high=vaddl_u8(vget_high_u8(a),vget_high_u8(b));
            for(int dy=2;dy<6;++dy){
                uint8x16_t v=vld1q_u8(rows+(size_t)dy*1920*3+i);
                low=vaddw_u8(low,vget_low_u8(v));high=vaddw_u8(high,vget_high_u8(v));
            }
            vst1q_u16(vertical+i,low);vst1q_u16(vertical+i+8,high);
        }
#endif
        for(int x=0;x<320;++x){
        unsigned sums[3]={0,0,0};
#if defined(__aarch64__)
        const uint16_t *p=vertical+x*18;
        for(int c=0;c<3;++c)sums[c]=p[c]+p[c+3]+p[c+6]+p[c+9]+p[c+12]+p[c+15];
#else
        for(int dy=0;dy<6;++dy){
            const uint8_t *p=source+((size_t)(6*y+dy)*1920+6*x)*3;
            for(int dx=0;dx<6;++dx){sums[0]+=p[3*dx];sums[1]+=p[3*dx+1];sums[2]+=p[3*dx+2];}
        }
#endif
        const size_t offset=((size_t)y*320+x)*3;
        for(int c=0;c<3;++c)target[offset+c]=average36[sums[c]];
        }
    }
    return 0;
}
