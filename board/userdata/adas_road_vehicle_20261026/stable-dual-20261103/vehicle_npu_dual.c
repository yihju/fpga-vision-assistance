/* Synchronous one-output specialization; never supplies road/lane masks. */
#include "rknn_api.h"
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#if defined(VEHICLE_NEON_AREA3) && defined(__aarch64__)
#include <arm_neon.h>
#endif
typedef struct {rknn_context ctx;
#ifdef VEHICLE_NATIVE_INPUT
    __fp16 *input;__fp16 lookup[256][3];
#endif
} vehicle_model;
#ifdef VEHICLE_FUSED_TRACKING
#include "vehicle_tracking_dual.h"
#endif
void *vehicle_npu_open(const char *path){
    if(!path)return NULL;vehicle_model *model=calloc(1,sizeof(*model));if(!model)return NULL;
    if(rknn_init(&model->ctx,(void*)path,0,0,NULL)){free(model);return NULL;}
    rknn_input_output_num counts={0};rknn_tensor_attr out={0};
    if(rknn_query(model->ctx,RKNN_QUERY_IN_OUT_NUM,&counts,sizeof(counts))||counts.n_input!=1||counts.n_output!=1)goto fail;
    if(rknn_query(model->ctx,RKNN_QUERY_OUTPUT_ATTR,&out,sizeof(out)))goto fail;
    fprintf(stderr,"vehicle output fmt=%d dims=%u,%u,%u elems=%u\n",out.fmt,out.dims[0],out.dims[1],out.dims[2],out.n_elems);
    if(out.n_dims!=3||out.dims[0]!=1||out.dims[1]!=25200||out.dims[2]!=6||out.n_elems!=151200)goto fail;
#ifdef VEHICLE_NATIVE_INPUT
    rknn_tensor_attr input={0};
    if(rknn_query(model->ctx,RKNN_QUERY_INPUT_ATTR,&input,sizeof(input))||input.type!=RKNN_TENSOR_FLOAT16||
        input.fmt!=RKNN_TENSOR_NHWC||input.n_dims!=4||input.dims[0]!=1||input.dims[1]!=640||input.dims[2]!=640||input.dims[3]!=3||input.size_with_stride!=640*640*3*2)goto fail;
    model->input=malloc(640*640*3*2);if(!model->input)goto fail;
    const float mean[3]={.485f,.456f,.406f},std[3]={.229f,.224f,.225f};
    for(int i=0;i<256;++i)for(int c=0;c<3;++c)model->lookup[i][c]=(__fp16)(((float)i/255.f-mean[c])/std[c]);
#endif
    return model;
fail:
#ifdef VEHICLE_NATIVE_INPUT
    free(model->input);
#endif
    rknn_destroy(model->ctx);free(model);return NULL;
}
static int execute(vehicle_model *model,rknn_input *input,float *det,size_t capacity){
    int code=rknn_inputs_set(model->ctx,1,input);if(code)return code;
    code=rknn_run(model->ctx,NULL);if(code)return code;
    rknn_output output={0};output.want_float=1;output.is_prealloc=1;output.buf=det;output.size=capacity*sizeof(float);
    code=rknn_outputs_get(model->ctx,1,&output,NULL);if(!code)rknn_outputs_release(model->ctx,1,&output);return code;
}
int vehicle_npu_run(void *handle,const uint8_t *rgb,size_t bytes,float *det,size_t capacity){
    if(!handle||!rgb||!det||bytes!=640*640*3||capacity!=151200)return -1;
    vehicle_model *model=handle;rknn_input input={0};input.buf=(void*)rgb;input.size=bytes;
    input.type=RKNN_TENSOR_UINT8;input.fmt=RKNN_TENSOR_NHWC;input.pass_through=0;
#ifdef VEHICLE_NATIVE_INPUT
    for(int i=0;i<640*640;++i){model->input[i*3]=model->lookup[rgb[i*3]][0];model->input[i*3+1]=model->lookup[rgb[i*3+1]][1];model->input[i*3+2]=model->lookup[rgb[i*3+2]][2];}
    input.buf=model->input;input.size=640*640*3*2;input.type=RKNN_TENSOR_FLOAT16;input.pass_through=1;
#endif
    return execute(model,&input,det,capacity);
}
int vehicle_npu_run_bgr3(void *handle,const uint8_t *bgr,size_t bytes,float *det,size_t capacity){
#ifdef VEHICLE_NATIVE_INPUT
    if(!handle||!bgr||!det||bytes!=1920*1080*3||capacity!=151200)return -1;
    vehicle_model *model=handle;
    for(int i=0;i<640*640;++i)for(int c=0;c<3;++c)model->input[i*3+c]=model->lookup[114][c];
    for(int y=0;y<360;++y){
#if defined(VEHICLE_NEON_AREA3) && defined(__aarch64__)
        uint16_t vertical[1920*3];
        const uint8_t *rows=bgr+(size_t)y*3*1920*3;
        for(int i=0;i<1920*3;i+=16){
            uint8x16_t a=vld1q_u8(rows+i),b=vld1q_u8(rows+1920*3+i),c=vld1q_u8(rows+1920*3*2+i);
            vst1q_u16(vertical+i,vaddw_u8(vaddl_u8(vget_low_u8(a),vget_low_u8(b)),vget_low_u8(c)));
            vst1q_u16(vertical+i+8,vaddw_u8(vaddl_u8(vget_high_u8(a),vget_high_u8(b)),vget_high_u8(c)));
        }
#endif
      for(int x=0;x<640;++x){
        unsigned sums[3]={0,0,0};
#if defined(VEHICLE_NEON_AREA3) && defined(__aarch64__)
        const uint16_t *p=vertical+x*9;
        for(int c=0;c<3;c++)sums[c]=p[c]+p[c+3]+p[c+6];
#else
        for(int dy=0;dy<3;++dy)for(int dx=0;dx<3;++dx){
            const uint8_t *p=bgr+((size_t)(y*3+dy)*1920+x*3+dx)*3;
            sums[0]+=p[0];sums[1]+=p[1];sums[2]+=p[2];
        }
#endif
        /* Nine integer samples cannot lie exactly on a half-integer tie. */
        __fp16 *target=model->input+((size_t)(y+140)*640+x)*3;
        target[0]=model->lookup[(sums[2]+4)/9][0];target[1]=model->lookup[(sums[1]+4)/9][1];target[2]=model->lookup[(sums[0]+4)/9][2];
      }
    }
    rknn_input input={0};input.buf=model->input;input.size=640*640*3*2;
    input.type=RKNN_TENSOR_FLOAT16;input.fmt=RKNN_TENSOR_NHWC;input.pass_through=1;
    return execute(model,&input,det,capacity);
#else
    (void)handle;(void)bgr;(void)bytes;(void)det;(void)capacity;return -1;
#endif
}
#ifdef VEHICLE_FUSED_TRACKING
int vehicle_npu_run_bgr3_tracking(void *handle,const uint8_t *bgr,size_t bytes,float *det,size_t capacity,uint8_t *small,size_t small_capacity){
    if(!handle||!bgr||!det||!small||bytes!=1920*1080*3||capacity!=151200||small_capacity!=180*320*3)return -1;
    vehicle_model *model=handle;prepare_tracking_dual(model,bgr,small);
    rknn_input input={0};input.buf=model->input;input.size=640*640*3*2;
    input.type=RKNN_TENSOR_FLOAT16;input.fmt=RKNN_TENSOR_NHWC;input.pass_through=1;
    return execute(model,&input,det,capacity);
}
#endif
int vehicle_npu_copy_input(void *handle,void *target,size_t bytes){
#ifdef VEHICLE_NATIVE_INPUT
    if(!handle||!target||bytes!=640*640*3*2)return -1;
    vehicle_model *model=handle;memcpy(target,model->input,bytes);return 0;
#else
    (void)handle;(void)target;(void)bytes;return -1;
#endif
}
void vehicle_npu_close(void *handle){if(handle){vehicle_model *model=handle;
#ifdef VEHICLE_NATIVE_INPUT
    free(model->input);
#endif
    rknn_destroy(model->ctx);free(model);}}
