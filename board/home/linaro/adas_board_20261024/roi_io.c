/* Aligned 32-bit PCIe I/O for the independent RPS1 road ROI feature block.
 * Never writes PCI config, camera DMA, LEDs or Flash. One owner via flock.
 */
#define _POSIX_C_SOURCE 200809L
#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdatomic.h>
#include <sys/file.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

struct roi_handle { volatile uint32_t *read, *write; int owner; };
struct roi_result {
    uint32_t sequence, count, gray_sum, red, green, yellow, binary_ones, gray_min, gray_max;
};
static uint64_t now_ns(void) {
    struct timespec ts;
    if (clock_gettime(CLOCK_MONOTONIC, &ts)) return 0;
    return (uint64_t)ts.tv_sec*1000000000ULL+(uint64_t)ts.tv_nsec;
}
static uint32_t load(struct roi_handle *h, unsigned byte) {
    uint32_t value=h->read[byte/4];
    atomic_thread_fence(memory_order_seq_cst);
    return value;
}
static void store(struct roi_handle *h, unsigned byte, uint32_t value) {
    atomic_thread_fence(memory_order_seq_cst);
    h->write[byte/4]=value;
    atomic_thread_fence(memory_order_seq_cst);
}
void adas_roi_close(void *handle) {
    struct roi_handle *h=handle;
    if (!h) return;
    if (h->write!=MAP_FAILED) {
        store(h,0x250,0x524f0002U);
        munmap((void *)h->write,4096);
    }
    if (h->read!=MAP_FAILED) munmap((void *)h->read,8192);
    if (h->owner>=0) close(h->owner);
    free(h);
}
void *adas_roi_open(const char *device) {
    struct roi_handle *h=calloc(1,sizeof(*h));
    if (!h) return NULL;
    h->read=MAP_FAILED;h->write=MAP_FAILED;h->owner=-1;
    h->owner=open("/run/lock/adas-roi.lock",O_RDWR|O_CREAT|O_CLOEXEC,0600);
    if (h->owner<0 || flock(h->owner,LOCK_EX|LOCK_NB)) goto fail;
    char path[1024];
    if (snprintf(path,sizeof(path),"%s/resource0",device)>=(int)sizeof(path)) { errno=ENAMETOOLONG;goto fail; }
    int fd=open(path,O_RDONLY|O_SYNC|O_CLOEXEC);
    if (fd<0) goto fail;
    h->read=mmap(NULL,8192,PROT_READ,MAP_SHARED,fd,0);close(fd);
    if (h->read==MAP_FAILED) goto fail;
    if (load(h,0x1d00)!=0x52505331U) { errno=ENODEV;goto fail; }
    if (snprintf(path,sizeof(path),"%s/resource1",device)>=(int)sizeof(path)) { errno=ENAMETOOLONG;goto fail; }
    fd=open(path,O_RDWR|O_SYNC|O_CLOEXEC);
    if (fd<0) goto fail;
    h->write=mmap(NULL,4096,PROT_READ|PROT_WRITE,MAP_SHARED,fd,0);close(fd);
    if (h->write==MAP_FAILED) goto fail;
    return h;
fail: {
    int saved=errno;adas_roi_close(h);errno=saved;return NULL;
}}
int adas_roi_key(void *handle) {
    struct roi_handle *h=handle;
    if (!h || load(h,0x1d00)!=0x52505331U) { errno=ENODEV;return -1; }
    return (int)((load(h,0x1d04)>>8)&1U);
}
int adas_roi_compute(void *handle, uint32_t sequence, const uint32_t *rgb,
                     uint32_t count, uint32_t timeout_ms, struct roi_result *result) {
    struct roi_handle *h=handle;
    if (!h || !rgb || !result || count==0 || count>4096 || timeout_ms==0 || timeout_ms>1000) {
        errno=EINVAL;return -1;
    }
    if (load(h,0x1d00)!=0x52505331U) { errno=ENODEV;return -1; }
    uint64_t started=now_ns(), deadline=started+(uint64_t)timeout_ms*1000000ULL;
    if (!started) { errno=EIO;return -1; }
    store(h,0x250,0x524f0002U);
    store(h,0x260,sequence);store(h,0x264,count);
    (void)load(h,0x1d20); /* drain ordered posted writes before START */
    store(h,0x250,0x524f0001U);
    for (uint32_t i=0;i<count;++i) {
        if (rgb[i]&0xff000000U) { errno=EINVAL;goto abort_job; }
        if (!(i&63U) && now_ns()>=deadline) { errno=ETIMEDOUT;goto abort_job; }
        store(h,0x270,rgb[i]);
    }
    uint32_t status;
    do {
        status=load(h,0x1d04);
        if (status&4U) { errno=EPROTO;goto abort_job; }
        if (now_ns()>=deadline) { errno=ETIMEDOUT;goto abort_job; }
    } while (!(status&2U));
    result->sequence=load(h,0x1d08);result->count=load(h,0x1d0c);
    result->gray_sum=load(h,0x1d10);result->red=load(h,0x1d14);
    result->green=load(h,0x1d18);result->yellow=load(h,0x1d1c);
    result->binary_ones=load(h,0x1d24);
    uint32_t extrema=load(h,0x1d28);
    result->gray_min=extrema&255U;result->gray_max=(extrema>>8)&255U;
    if (result->sequence!=sequence || result->count!=count || load(h,0x1d20)!=count ||
        (load(h,0x1d04)&7U)!=2U || now_ns()>=deadline) { errno=EPROTO;goto abort_job; }
    return 0;
abort_job: {
    int saved=errno;store(h,0x250,0x524f0002U);errno=saved;return -1;
}}
