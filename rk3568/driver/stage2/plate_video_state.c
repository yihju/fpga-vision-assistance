#include "plate_video_state.h"
#ifdef __KERNEL__
#include <linux/errno.h>
#include <linux/string.h>
#else
#include <errno.h>
#include <string.h>
#endif

void pvs_init(struct pvs_state *s)
{
    memset(s, 0, sizeof(*s)); s->device_slot=-1;
}
int pvs_open(struct pvs_state *s)
{
    if (s->detached) return -ENODEV;
    if (s->run==PVS_FAILED_QUARANTINED) return -EIO;
    if (s->opened || s->map_refs || s->device_slot>=0) return -EBUSY;
    s->opened=1; return 0;
}
void pvs_close(struct pvs_state *s)
{
    unsigned i;
    pvs_stop(s); s->opened=0;
    if (!s->map_refs)
        for (i=0;i<4;++i)
            if (s->slot[i].owner==PVS_IN_USE || s->slot[i].owner==PVS_READY)
                s->slot[i].owner=PVS_FREE;
}
int pvs_start(struct pvs_state *s, __u32 epoch)
{
    unsigned i;
    if (s->detached) return -ENODEV;
    if (s->run==PVS_FAILED_QUARANTINED) return -EIO;
    if (!s->opened) return -EBADF;
    if (s->run!=PVS_IDLE || s->device_slot>=0) return -EBUSY;
    if (!epoch || epoch==s->epoch) return -EINVAL;
    for (i=0; i<PLATE_VIDEO_SLOTS; ++i)
        if (s->slot[i].owner==PVS_IN_USE) return -EBUSY;
    for (i=0; i<PLATE_VIDEO_SLOTS; ++i) s->slot[i].owner=PVS_FREE;
    s->epoch=epoch; s->run=PVS_RUNNING; return 0;
}
int pvs_submit(struct pvs_state *s)
{
    int pick=-1; unsigned i; __u64 oldest=~(__u64)0;
    struct pvs_slot *slot;
    if (s->run!=PVS_RUNNING) return -ESHUTDOWN;
    if (s->device_slot>=0) return -EBUSY;
    if (s->next_seq==~(__u64)0) return -EOVERFLOW;
    for (i=0; i<PLATE_VIDEO_SLOTS; ++i)
        if (s->slot[i].owner==PVS_FREE) { pick=(int)i; break; }
    if (pick<0) {
        for (i=0; i<PLATE_VIDEO_SLOTS; ++i)
            if (s->slot[i].owner==PVS_READY && s->slot[i].lease.seq<oldest) {
                oldest=s->slot[i].lease.seq; pick=(int)i;
            }
        if (pick<0) return -ENOSPC;
        ++s->ready_drops;
    }
    if (++s->next_cookie==0) ++s->next_cookie;
    slot=&s->slot[pick]; memset(&slot->lease,0,sizeof(slot->lease));
    slot->lease.epoch=s->epoch; slot->lease.slot=(__u32)pick;
    slot->lease.cookie=s->next_cookie; slot->lease.seq=++s->next_seq;
    slot->owner=PVS_DEVICE; s->device_slot=pick; return pick;
}
int pvs_complete(struct pvs_state *s, const struct plate_video_record *r)
{
    struct pvs_slot *slot; const struct plate_video_lease *l; unsigned i;
    if (s->run==PVS_FAILED_QUARANTINED) return -EIO;
    if (s->device_slot<0) return -EAGAIN;
    slot=&s->slot[s->device_slot]; l=&slot->lease;
    if (!r->commit_cookie || r->commit_cookie!=l->cookie) return -EAGAIN;
    if (r->magic!=PLATE_VIDEO_MAGIC || r->major!=PLATE_VIDEO_MAJOR ||
        r->epoch!=l->epoch || r->slot!=l->slot || r->seq!=l->seq ||
        r->format!=PLATE_VIDEO_RGB565_LE || r->repeated_source_mask>15 ||
        r->status>PLATE_VIDEO_STATUS_ERROR || r->bytes>PLATE_VIDEO_FRAME_BYTES ||
        (r->bytes&15) || (r->status==PLATE_VIDEO_STATUS_OK &&
                         r->bytes!=PLATE_VIDEO_FRAME_BYTES)) return -EPROTO;
    for (i=0; i<sizeof(r->reserved); ++i) if (r->reserved[i]) return -EPROTO;
    slot->lease.status=r->status; slot->lease.bytes=r->bytes;
    memcpy(slot->lease.source_id,r->source_id,sizeof(r->source_id));
    slot->owner=r->status==PLATE_VIDEO_STATUS_OK ? PVS_READY : PVS_FREE;
    s->device_slot=-1;
    if (s->run==PVS_STOPPING || r->status!=PLATE_VIDEO_STATUS_OK) s->run=PVS_IDLE;
    return 0;
}
int pvs_dequeue(struct pvs_state *s, struct plate_video_lease *lease)
{
    int pick=-1; unsigned i; __u64 oldest=~(__u64)0;
    if (s->run==PVS_FAILED_QUARANTINED) return -EIO;
    for (i=0; i<PLATE_VIDEO_SLOTS; ++i)
        if (s->slot[i].owner==PVS_READY && s->slot[i].lease.seq<oldest) {
            pick=(int)i; oldest=s->slot[i].lease.seq;
        }
    if (pick<0) return -EAGAIN;
    s->slot[pick].owner=PVS_IN_USE; *lease=s->slot[pick].lease; return 0;
}
int pvs_queue(struct pvs_state *s, const struct plate_video_lease *lease)
{
    struct pvs_slot *slot;
    if (s->run==PVS_FAILED_QUARANTINED) return -EIO;
    if (lease->slot>=PLATE_VIDEO_SLOTS) return -ESTALE;
    slot=&s->slot[lease->slot];
    if (slot->owner!=PVS_IN_USE || memcmp(&slot->lease,lease,sizeof(*lease)))
        return -ESTALE;
    slot->owner=PVS_FREE; return 0;
}
void pvs_stop(struct pvs_state *s)
{
    if (s->run!=PVS_FAILED_QUARANTINED)
        s->run=s->device_slot<0 ? PVS_IDLE : PVS_STOPPING;
}
void pvs_stop_timeout(struct pvs_state *s)
{
    if (s->device_slot>=0) s->run=PVS_FAILED_QUARANTINED;
}
void pvs_map_get(struct pvs_state *s) { ++s->map_refs; }
void pvs_map_put(struct pvs_state *s)
{
    unsigned i;
    if (s->map_refs) --s->map_refs;
    if (!s->map_refs && !s->opened)
        for (i=0;i<4;++i)
            if (s->slot[i].owner==PVS_IN_USE || s->slot[i].owner==PVS_READY)
                s->slot[i].owner=PVS_FREE;
}
int pvs_can_free(const struct pvs_state *s)
{
    return s->run!=PVS_FAILED_QUARANTINED && s->device_slot<0 && !s->map_refs;
}
int pvs_validate_caps(const struct plate_video_caps *c)
{
    if (c->magic!=PLATE_VIDEO_MAGIC || c->major!=PLATE_VIDEO_MAJOR ||
        (c->features&PLATE_VIDEO_REQUIRED_FEATURES)!=PLATE_VIDEO_REQUIRED_FEATURES ||
        c->width!=1280 || c->height!=720 || c->stride!=2560 || c->format!=1 ||
        c->frame_bytes!=1843200 || c->slots!=4 || c->slot_stride!=1847296 ||
        c->metadata_offset!=1843200 || c->metadata_bytes!=128 ||
        c->dma_addr_bits!=32 || !c->epoch || !c->clock_hz) return -EPROTO;
    return 0;
}
int pvs_command(const struct plate_video_lease *l, __u64 dma, __u32 w[12])
{
    if ((dma&4095) || !l->cookie || !l->epoch || l->slot>=4) return -EINVAL;
    if (dma>0xffe3d000ULL) return -ERANGE;
    memset(w,0,12*sizeof(*w)); w[0]=(__u32)dma;
    w[2]=l->cookie; w[3]=(__u32)l->seq; w[4]=(__u32)(l->seq>>32);
    w[5]=l->epoch; w[6]=l->slot; w[7]=1843200; w[11]=1; return 0;
}
void pvs_detach(struct pvs_state *s)
{
    s->detached=1; pvs_stop(s);
}
int pvs_map_index(__u64 offset, __u64 length, unsigned writable, unsigned executable)
{
    if (writable || executable || length!=1847296 || offset%1847296 ||
        offset/1847296>=4) return -EINVAL;
    return (int)(offset/1847296);
}
