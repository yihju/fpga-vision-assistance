/* Same native BGR MPP pipeline, without PyGI's intermediate full-frame bytes.
 * Frame identity is counted at the decoder output and bound to actual PTS.
 */
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/video/video.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "rga_copy.h"
typedef struct {
    GstElement *pipeline;
    GstAppSink *sink;
    GstVideoInfo video;
    GMutex lock;
    guint64 pts[256],ids[256],count,last_pts;
    int identity_error;
    char error[512];
    int started,eos;
    CachedCopy cached;
    gint fps_num,fps_den;
} media;
static GstPadProbeReturn on_buffer(GstPad *pad,GstPadProbeInfo *info,gpointer user){
    media *m=user;GstBuffer *buffer=GST_PAD_PROBE_INFO_BUFFER(info);
    guint64 pts=GST_BUFFER_PTS(buffer);
    g_mutex_lock(&m->lock);
    if(!GST_CLOCK_TIME_IS_VALID(pts)||(m->count&&pts<=m->last_pts))m->identity_error=1;
    else {guint64 slot=m->count%256;m->pts[slot]=pts;m->ids[slot]=m->count;m->count++;m->last_pts=pts;}
    g_mutex_unlock(&m->lock);return GST_PAD_PROBE_OK;
}
void adas_media_close(void *handle){
    if(!handle)return;media *m=handle;
    if(m->pipeline){gst_element_set_state(m->pipeline,GST_STATE_NULL);gst_object_unref(m->pipeline);}
    if(m->sink)gst_object_unref(m->sink);
    cached_copy_close(&m->cached);g_mutex_clear(&m->lock);free(m);
}
void *adas_media_open(const char *path,int live,int *dimensions){
    gst_init(NULL,NULL);media *m=calloc(1,sizeof(*m));if(!m)return NULL;g_mutex_init(&m->lock);
    GError *error=NULL;
    const char *latest="filesrc name=source ! qtdemux ! parsebin ! mppvideodec format=BGR ignore-error=false ! video/x-raw,format=BGR ! appsink name=frames sync=true processing-deadline=0 emit-signals=false max-buffers=1 drop=true";
    const char *sequential="filesrc name=source ! qtdemux ! parsebin ! mppvideodec format=BGR ignore-error=false ! video/x-raw,format=BGR ! appsink name=frames sync=false emit-signals=false max-buffers=1 drop=false";
    m->pipeline=gst_parse_launch(live?latest:sequential,&error);
    if(error){g_printerr("media open: %s\n",error->message);g_error_free(error);goto fail;}
    if(!m->pipeline)goto fail;
    GstElement *source=gst_bin_get_by_name(GST_BIN(m->pipeline),"source");
    if(!source)goto fail;g_object_set(source,"location",path,NULL);gst_object_unref(source);
    m->sink=GST_APP_SINK(gst_bin_get_by_name(GST_BIN(m->pipeline),"frames"));if(!m->sink)goto fail;
    GstPad *pad=gst_element_get_static_pad(GST_ELEMENT(m->sink),"sink");
    gst_pad_add_probe(pad,GST_PAD_PROBE_TYPE_BUFFER,on_buffer,m,NULL);gst_object_unref(pad);
    if(gst_element_set_state(m->pipeline,GST_STATE_PAUSED)==GST_STATE_CHANGE_FAILURE)goto fail;
    if(gst_element_get_state(m->pipeline,NULL,NULL,10*GST_SECOND)==GST_STATE_CHANGE_FAILURE)goto fail;
    GstSample *sample=gst_app_sink_try_pull_preroll(m->sink,GST_SECOND);if(!sample)goto fail;
    GstCaps *caps=gst_sample_get_caps(sample);
    int valid=gst_video_info_from_caps(&m->video,caps)&&GST_VIDEO_INFO_FORMAT(&m->video)==GST_VIDEO_FORMAT_BGR;
    GstStructure *structure=gst_caps_get_structure(caps,0);
    if(!gst_structure_get_fraction(structure,"framerate",&m->fps_num,&m->fps_den))m->fps_num=0;
    gst_sample_unref(sample);if(!valid)goto fail;
    dimensions[0]=GST_VIDEO_INFO_WIDTH(&m->video);dimensions[1]=GST_VIDEO_INFO_HEIGHT(&m->video);
    dimensions[2]=m->fps_num;dimensions[3]=m->fps_den;return m;
fail:adas_media_close(m);return NULL;
}
int adas_media_read(void *handle,void *pixels,uint64_t capacity,uint64_t *metadata){
    if(!handle||!pixels||!metadata)return -1;media *m=handle;if(m->eos)return 0;
    if(!m->started){if(gst_element_set_state(m->pipeline,GST_STATE_PLAYING)==GST_STATE_CHANGE_FAILURE)return -2;m->started=1;}
    GstSample *sample=gst_app_sink_try_pull_sample(m->sink,5*GST_SECOND);
    if(!sample){
        GstBus *bus=gst_element_get_bus(m->pipeline);GstMessage *message=gst_bus_pop_filtered(bus,GST_MESSAGE_ERROR);gst_object_unref(bus);
        if(message){GError *err=NULL;gchar *debug=NULL;gst_message_parse_error(message,&err,&debug);
            g_strlcpy(m->error,err?err->message:"MPP error",sizeof(m->error));if(err)g_error_free(err);g_free(debug);gst_message_unref(message);return -3;}
        if(gst_app_sink_is_eos(m->sink)){m->eos=1;return 0;}return -4;
    }
    int result=-5;GstVideoInfo info;
    if(!gst_video_info_from_caps(&info,gst_sample_get_caps(sample))||GST_VIDEO_INFO_FORMAT(&info)!=GST_VIDEO_FORMAT_BGR||
       GST_VIDEO_INFO_WIDTH(&info)!=GST_VIDEO_INFO_WIDTH(&m->video)||GST_VIDEO_INFO_HEIGHT(&info)!=GST_VIDEO_INFO_HEIGHT(&m->video))goto done;
    int w=GST_VIDEO_INFO_WIDTH(&info),h=GST_VIDEO_INFO_HEIGHT(&info),stride=GST_VIDEO_INFO_PLANE_STRIDE(&info,0);
    if(w<1||h<1||stride<w*3||capacity<(uint64_t)w*h*3)goto done;
    GstBuffer *buffer=gst_sample_get_buffer(sample);guint64 pts=GST_BUFFER_PTS(buffer),id=0;int found=0;
    g_mutex_lock(&m->lock);
    if(!m->identity_error){for(int i=0;i<256&&i<(int)m->count;i++)if(m->pts[i]==pts){id=m->ids[i];found=1;break;}}
    g_mutex_unlock(&m->lock);if(!found){result=-6;goto done;}
    GstMapInfo map;if(!gst_buffer_map(buffer,&map,GST_MAP_READ))goto done;
    guint64 offset=GST_VIDEO_INFO_PLANE_OFFSET(&info,0);
    if(offset+(uint64_t)(h-1)*stride+w*3>map.size){gst_buffer_unmap(buffer,&map);goto done;}
    int copied=offset?-11:cached_copy(&m->cached,buffer,w,h,stride,pixels);
    if(copied){g_snprintf(m->error,sizeof(m->error),"RGA copy failed %d",copied);gst_buffer_unmap(buffer,&map);result=-7;goto done;}
    gst_buffer_unmap(buffer,&map);metadata[0]=id;metadata[1]=pts;result=1;
done:gst_sample_unref(sample);return result;
}
uint64_t adas_media_count(void *handle){media *m=handle;if(!m)return 0;g_mutex_lock(&m->lock);uint64_t result=m->count;g_mutex_unlock(&m->lock);return result;}
const char *adas_media_error(void *handle){return handle?((media*)handle)->error:"no reader";}
