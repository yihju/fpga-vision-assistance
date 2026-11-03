#ifndef PLATE_VIDEO_STATE_H
#define PLATE_VIDEO_STATE_H
#include "plate_video_uapi.h"

/* Serialized by the adapter mutex. No kernel primitives or DMA addresses. */
enum pvs_owner { PVS_FREE, PVS_DEVICE, PVS_READY, PVS_IN_USE };
enum pvs_run { PVS_IDLE, PVS_RUNNING, PVS_STOPPING, PVS_FAILED_QUARANTINED };
struct pvs_slot {
    enum pvs_owner owner;
    struct plate_video_lease lease;
};
struct pvs_state {
    struct pvs_slot slot[PLATE_VIDEO_SLOTS];
    enum pvs_run run;
    int device_slot;
    unsigned opened, map_refs, detached;
    __u32 epoch, next_cookie;
    __u64 next_seq, ready_drops;
};
void pvs_init(struct pvs_state *s);
int pvs_open(struct pvs_state *s);
void pvs_close(struct pvs_state *s);
int pvs_start(struct pvs_state *s, __u32 epoch);
int pvs_submit(struct pvs_state *s);
/* Record has been READ_ONCE(cookie), dma_rmb(), copied and LE decoded by adapter. */
int pvs_complete(struct pvs_state *s, const struct plate_video_record *r);
int pvs_dequeue(struct pvs_state *s, struct plate_video_lease *lease);
int pvs_queue(struct pvs_state *s, const struct plate_video_lease *lease);
void pvs_stop(struct pvs_state *s);
void pvs_stop_timeout(struct pvs_state *s);
void pvs_map_get(struct pvs_state *s);
void pvs_map_put(struct pvs_state *s);
int pvs_can_free(const struct pvs_state *s);
int pvs_validate_caps(const struct plate_video_caps *caps);
int pvs_command(const struct plate_video_lease *lease, __u64 dma, __u32 words[12]);
void pvs_detach(struct pvs_state *s);
int pvs_map_index(__u64 offset, __u64 length, unsigned writable, unsigned executable);
#endif
