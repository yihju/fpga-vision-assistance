/* Candidate synchronous bridge: explicit native FP16 input conversion.
 * Only enable after same-model, same-frame output/box/class/score comparison.
 */
#include "rknn_api.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
typedef struct {
    rknn_context ctx;
    rknn_tensor_attr out[3];
    __fp16 *input;
    __fp16 lookup[2][256];
    double timing[3];
} model;
static double msnow(void){struct timespec t;clock_gettime(CLOCK_MONOTONIC,&t);return t.tv_sec*1000.+t.tv_nsec/1e6;}
void *adas_npu_open(const char *path){
    model *m=calloc(1,sizeof(*m));if(!m)return NULL;
    if(rknn_init(&m->ctx,(void*)path,0,0,NULL)){free(m);return NULL;}
    rknn_input_output_num n={0};rknn_tensor_attr input={0};
    if(rknn_query(m->ctx,RKNN_QUERY_IN_OUT_NUM,&n,sizeof(n))||n.n_input!=1||n.n_output!=3)goto fail;
    if(rknn_query(m->ctx,RKNN_QUERY_INPUT_ATTR,&input,sizeof(input))||input.type!=RKNN_TENSOR_FLOAT16||
       input.fmt!=RKNN_TENSOR_NHWC||input.n_dims!=4||input.dims[0]!=1||input.dims[1]!=640||
       input.dims[2]!=640||input.dims[3]!=3||input.size_with_stride!=640*640*3*2)goto fail;
    for(int i=0;i<3;i++){
        m->out[i].index=i;int size=80>>i;
        if(rknn_query(m->ctx,RKNN_QUERY_OUTPUT_ATTR,&m->out[i],sizeof(m->out[i]))||m->out[i].n_dims!=4||
           m->out[i].dims[0]!=1||m->out[i].dims[1]!=33||m->out[i].dims[2]!=size||m->out[i].dims[3]!=size)goto fail;
    }
    m->input=malloc(640*640*3*2);if(!m->input)goto fail;
    for(int i=0;i<256;i++){m->lookup[0][i]=(__fp16)i;m->lookup[1][i]=(__fp16)((float)i/255.f);}
    return m;
fail:rknn_destroy(m->ctx);free(m);return NULL;
}
int adas_npu_run_probe(void *handle,const void *rgb,float *a,float *b,float *c,int normalize){
    if(!handle||!rgb||normalize<0||normalize>1)return -1;model *m=handle;
    double tick=msnow();const unsigned char *src=rgb;
    for(int i=0;i<640*640*3;i++)m->input[i]=m->lookup[normalize][src[i]];
    rknn_input input={0};input.buf=m->input;input.size=640*640*3*2;
    input.type=RKNN_TENSOR_FLOAT16;input.fmt=RKNN_TENSOR_NHWC;input.pass_through=1;
    int rc=rknn_inputs_set(m->ctx,1,&input);m->timing[0]=msnow()-tick;if(rc)return rc;
    tick=msnow();rc=rknn_run(m->ctx,NULL);m->timing[1]=msnow()-tick;if(rc)return rc;
    rknn_output out[3]={{0}};void *buffers[3]={a,b,c};
    for(int i=0;i<3;i++){out[i].index=i;out[i].want_float=1;out[i].is_prealloc=1;out[i].buf=buffers[i];out[i].size=m->out[i].n_elems*sizeof(float);}
    tick=msnow();rc=rknn_outputs_get(m->ctx,3,out,NULL);m->timing[2]=msnow()-tick;
    if(!rc)rknn_outputs_release(m->ctx,3,out);return rc;
}
int adas_npu_run(void *handle,const void *rgb,float *a,float *b,float *c){
    return adas_npu_run_probe(handle,rgb,a,b,c,1);
}
void adas_npu_timing(void *handle,double *out){if(handle&&out)memcpy(out,((model*)handle)->timing,3*sizeof(double));}
void adas_npu_close(void *handle){if(handle){model *m=handle;free(m->input);rknn_destroy(m->ctx);free(m);}}
