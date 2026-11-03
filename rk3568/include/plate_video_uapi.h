/* Stage2 ABI v1. Wire records/caps are little endian; ioctl structs native endian.
 * No userspace DMA address, BAR mapping or writable DMA memory is exposed. */
#ifndef PLATE_VIDEO_UAPI_H
#define PLATE_VIDEO_UAPI_H
#include <linux/types.h>
#include <linux/ioctl.h>
#define PLATE_VIDEO_MAGIC 0x44564c50U
#define PLATE_VIDEO_MAJOR 1U
#define PLATE_VIDEO_MINOR 0U
#define PLATE_VIDEO_FEATURE_BATCH 1U
#define PLATE_VIDEO_FEATURE_RECORD 2U
#define PLATE_VIDEO_FEATURE_MSI 4U
#define PLATE_VIDEO_FEATURE_SOURCE_ID 8U
#define PLATE_VIDEO_REQUIRED_FEATURES 15U
#define PLATE_VIDEO_RGB565_LE 1U
#define PLATE_VIDEO_WIDTH 1280U
#define PLATE_VIDEO_HEIGHT 720U
#define PLATE_VIDEO_STRIDE 2560U
#define PLATE_VIDEO_FRAME_BYTES 1843200U
#define PLATE_VIDEO_SLOTS 4U
#define PLATE_VIDEO_SLOT_STRIDE 1847296U
#define PLATE_VIDEO_RECORD_BYTES 128U
#define PLATE_VIDEO_STATUS_OK 0U
#define PLATE_VIDEO_STATUS_CANCELLED 1U
#define PLATE_VIDEO_STATUS_ERROR 2U
#define PLATE_VIDEO_BAR_CAPS 0x1a00U
#define PLATE_VIDEO_BAR_STATE 0x1b00U
#define PLATE_VIDEO_BAR_COMMAND 0x200U
#define PLATE_VIDEO_DOORBELL_SUBMIT 1U
#define PLATE_VIDEO_DOORBELL_STOP 2U

struct plate_video_caps {
    __u32 magic, major, minor, features;
    __u32 width, height, stride, format;
    __u32 frame_bytes, slots, slot_stride, metadata_offset;
    __u32 metadata_bytes, dma_addr_bits, epoch, clock_hz;
};
struct plate_video_record {
    __u32 magic, major, epoch, slot;
    __u64 seq;
    __u32 bytes, status;
    __u32 source_id[4];
    __u64 source_tick[4];
    __u32 repeated_source_mask, format;
    __u64 acquire_tick;
    __u8 reserved[28];
    __u32 commit_cookie;
};
struct plate_video_start { __u32 major, flags, max_submission_fps, timeout_ms; };
struct plate_video_lease {
    __u32 epoch, slot, cookie, status;
    __u64 seq;
    __u32 bytes, flags;
    __u32 source_id[4];
    __u8 reserved[16];
};
#define PLATE_VIDEO_GET_CAPS _IOR('V', 0, struct plate_video_caps)
#define PLATE_VIDEO_START _IOW('V', 1, struct plate_video_start)
#define PLATE_VIDEO_DQBUF _IOR('V', 2, struct plate_video_lease)
#define PLATE_VIDEO_QBUF _IOW('V', 3, struct plate_video_lease)
#define PLATE_VIDEO_STOP _IO('V', 4)
#endif
