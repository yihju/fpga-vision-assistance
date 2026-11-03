/* Exact area3 neural input and original area6 tracking RGB from the same sums. */
#include <math.h>
#if defined(__aarch64__)
#include <arm_neon.h>
#endif
static uint8_t dual_average36[36*255+1];
__attribute__((constructor)) static void initialize_dual_average36(void){
    const float scale=1.f/36.f;
    for(int i=0;i<=36*255;i++)dual_average36[i]=(uint8_t)lrintf((float)i*scale);
}
static void prepare_tracking_dual(vehicle_model *model,const uint8_t *bgr,uint8_t *small){
    for(int i=0;i<640*640;i++)for(int c=0;c<3;c++)model->input[i*3+c]=model->lookup[114][c];
    for(int y=0;y<180;y++){
        uint16_t sums[2][1920*3];
        for(int group=0;group<2;group++){
            const uint8_t *rows=bgr+(size_t)(y*6+group*3)*1920*3;
            for(int i=0;i<1920*3;i+=16){
#if defined(__aarch64__)
                uint8x16_t a=vld1q_u8(rows+i),b=vld1q_u8(rows+1920*3+i),c=vld1q_u8(rows+1920*3*2+i);
                vst1q_u16(sums[group]+i,vaddw_u8(vaddl_u8(vget_low_u8(a),vget_low_u8(b)),vget_low_u8(c)));
                vst1q_u16(sums[group]+i+8,vaddw_u8(vaddl_u8(vget_high_u8(a),vget_high_u8(b)),vget_high_u8(c)));
#else
                for(int j=0;j<16;j++)sums[group][i+j]=rows[i+j]+rows[1920*3+i+j]+rows[1920*3*2+i+j];
#endif
            }
        }
        for(int x=0;x<320;x++){
            unsigned v[2][2][3];
            for(int row=0;row<2;row++)for(int column=0;column<2;column++){
                const uint16_t *p=sums[row]+x*18+column*9;
                for(int c=0;c<3;c++)v[row][column][c]=p[c]+p[c+3]+p[c+6];
                __fp16 *target=model->input+((size_t)(y*2+140+row)*640+x*2+column)*3;
                target[0]=model->lookup[(v[row][column][2]+4)/9][0];
                target[1]=model->lookup[(v[row][column][1]+4)/9][1];
                target[2]=model->lookup[(v[row][column][0]+4)/9][2];
            }
            for(int c=0;c<3;c++)small[((size_t)y*320+x)*3+c]=dual_average36[v[0][0][c]+v[0][1][c]+v[1][0][c]+v[1][1][c]];
        }
    }
}
