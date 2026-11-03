/* Synchronous RKNN C API; no async/previous-frame outputs or system libraries replaced. */
#include "rknn_api.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
typedef struct { rknn_context ctx; rknn_tensor_attr out[3]; } model;
void *adas_npu_open(const char *path) {
    model *m=calloc(1,sizeof(*m));if(!m)return NULL;
    if(rknn_init(&m->ctx,(void*)path,0,0,NULL)!=0){free(m);return NULL;}
    rknn_input_output_num n={0};
    if(rknn_query(m->ctx,RKNN_QUERY_IN_OUT_NUM,&n,sizeof(n))||n.n_input!=1||n.n_output!=3)goto fail;
    for(int i=0;i<3;i++){
        m->out[i].index=i;
        if(rknn_query(m->ctx,RKNN_QUERY_OUTPUT_ATTR,&m->out[i],sizeof(m->out[i])))goto fail;
        fprintf(stderr,"RKNN output%d fmt=%d dims=%u,%u,%u,%u elems=%u\n",i,m->out[i].fmt,m->out[i].dims[0],m->out[i].dims[1],m->out[i].dims[2],m->out[i].dims[3],m->out[i].n_elems);
        int size=80>>i;
        if(m->out[i].n_dims!=4||m->out[i].dims[0]!=1||m->out[i].dims[1]!=33||m->out[i].dims[2]!=size||m->out[i].dims[3]!=size)goto fail;
    }
    return m;
fail:rknn_destroy(m->ctx);free(m);return NULL;
}
int adas_npu_run(void *handle,const void *rgb,float *a,float *b,float *c){
    if(!handle||!rgb)return -1;model *m=handle;
    rknn_input input={0};input.buf=(void*)rgb;input.size=640*640*3;
    input.type=RKNN_TENSOR_UINT8;input.fmt=RKNN_TENSOR_NHWC;input.pass_through=0;
    int rc=rknn_inputs_set(m->ctx,1,&input);if(rc)return rc;
    rc=rknn_run(m->ctx,NULL);if(rc)return rc;
    rknn_output out[3]={{0}};void *buffers[3]={a,b,c};
    for(int i=0;i<3;i++){out[i].index=i;out[i].want_float=1;out[i].is_prealloc=1;out[i].buf=buffers[i];out[i].size=m->out[i].n_elems*sizeof(float);}
    rc=rknn_outputs_get(m->ctx,3,out,NULL);
    if(!rc)rknn_outputs_release(m->ctx,3,out);
    return rc;
}
void adas_npu_close(void *handle){if(handle){model *m=handle;rknn_destroy(m->ctx);free(m);}}
