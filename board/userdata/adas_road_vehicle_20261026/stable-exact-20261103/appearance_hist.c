/* Exact 8x4x4 histogram on the EXISTING installed OpenCV UINT8 HSV output.
 * Hue is mapped to zero for S<32 exactly as the original tracking code.
 * BGR->HSV stays in cv2: its rounding is specific to the installed backend.
 * No input writes, model, threshold, target geometry or tracking rule changes.
 */
#include <stdint.h>
#include <stddef.h>

static int process(const uint8_t *pixels,size_t width,size_t height,size_t stride,
                   uint8_t *bins,uint32_t *counts){
    if(!pixels||!width||!height||width>16777216U||height>16777216U||
       width>16777216U/height||stride<width*3U||(!bins&&!counts))return -1;
    if(counts)for(int i=0;i<128;++i)counts[i]=0;
    for(size_t y=0;y<height;++y){
        const uint8_t *row=pixels+y*stride;
        for(size_t x=0;x<width;++x){
            unsigned h=row[3*x],s=row[3*x+1],v=row[3*x+2];if(s<32)h=0;
            if(h>=180)return -2;
            unsigned index=((h*8U)/180U)*16U+(s>>6)*4U+(v>>6);
            if(bins)bins[y*width+x]=(uint8_t)index;
            if(counts)++counts[index];
        }
    }
    return 0;
}
int appearance_bins(const uint8_t *pixels,size_t width,size_t height,size_t stride,uint8_t *out,size_t capacity){
    if(!height||width>16777216U/height||capacity<width*height||!out)return -1;
    return process(pixels,width,height,stride,out,0);
}
int appearance_hist(const uint8_t *pixels,size_t width,size_t height,size_t stride,uint32_t *out,size_t capacity){
    if(!out||capacity!=128)return -1;return process(pixels,width,height,stride,0,out);
}
