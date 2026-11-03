#define _POSIX_C_SOURCE 200809L
/* Stage2 owned-frame reader. The driver validates DMA completion; this reader
 * independently validates the leased record before copying to frame_exchange. */
#include "frame_exchange.h"
#include "../driver/stage2/plate_video_state.h"
#include <errno.h>
#include <ctype.h>
#include <fcntl.h>
#include <getopt.h>
#include <poll.h>
#include <pthread.h>
#include <signal.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

static volatile sig_atomic_t stream_signal;
struct stream {
    struct frame_exchange *exchange;
    const char *device, *dump, *records;
    unsigned frames, max_fps;
    int stats, stdout_pixels, producer_error, consumer_error;
    atomic_int cancel;
    uint64_t started, acquired, displayed, unique[4], repeated[4];
    uint32_t previous[4];
    uint32_t stream_epoch;
    uint64_t last_transport_seq;
    uint64_t periods[512], last_received, poll_timeouts;
    uint64_t ready_before, ready_drops;
    int ready_drop_known;
};
static void stream_on_signal(int sig) { (void)sig; stream_signal=1; }
static int stream_cancelled(struct stream *s) { return stream_signal || atomic_load(&s->cancel); }
static uint64_t stream_ns(void)
{
    struct timespec t; if(clock_gettime(CLOCK_MONOTONIC,&t)) abort();
    return (uint64_t)t.tv_sec*1000000000ULL+(uint64_t)t.tv_nsec;
}
static uint32_t stream_u32(const unsigned char *p)
{
    return (uint32_t)p[0] | (uint32_t)p[1]<<8 | (uint32_t)p[2]<<16 | (uint32_t)p[3]<<24;
}
static uint64_t stream_u64(const unsigned char *p)
{
    return stream_u32(p) | (uint64_t)stream_u32(p+4)<<32;
}
static int stream_parse_ready_drops(FILE *f,uint64_t *value)
{
    char buf[64], *end;
    if(!fgets(buf,sizeof(buf),f) || !isdigit((unsigned char)buf[0]))return -1;
    errno=0; unsigned long long number=strtoull(buf,&end,10);
    if(errno)return -1;
    while(isspace((unsigned char)*end))++end;
    if(*end)return -1;
    *value=(uint64_t)number; return 0;
}
static int stream_read_ready_drops(uint64_t *value)
{
    FILE *f=fopen("/sys/class/misc/plate_video_dma/ready_drops","r");
    if(!f)return -1;
    int ret=stream_parse_ready_drops(f,value); fclose(f); return ret;
}
static int stream_record_valid(const unsigned char *r, const struct plate_video_lease *l)
{
    if(l->slot>=4 || !l->cookie || !l->epoch || !l->seq || l->status || l->bytes!=1843200 || l->flags)
        return 0;
    for(unsigned i=0;i<sizeof(l->reserved);++i) if(l->reserved[i]) return 0;
    if(stream_u32(r)!=0x44564c50 || stream_u32(r+4)!=1 || stream_u32(r+8)!=l->epoch ||
       stream_u32(r+12)!=l->slot || stream_u64(r+16)!=l->seq || stream_u32(r+24)!=1843200 ||
       stream_u32(r+28) || stream_u32(r+80)>15 || stream_u32(r+84)!=1 ||
       stream_u32(r+124)!=l->cookie) return 0;
    for(unsigned i=0;i<4;++i) if(stream_u32(r+32+4*i)!=l->source_id[i]) return 0;
    for(unsigned i=96;i<124;++i) if(r[i]) return 0;
    return 1;
}
static int stream_source_counts(struct stream *s,const struct plate_video_lease *l)
{
    if(s->acquired && (l->epoch!=s->stream_epoch || l->seq<=s->last_transport_seq))return -1;
    for(unsigned i=0;i<4;++i)
        if(s->acquired && (uint32_t)(l->source_id[i]-s->previous[i])>=0x80000000U) return -1;
    for(unsigned i=0;i<4;++i) {
        if(!s->acquired || l->source_id[i]!=s->previous[i]) ++s->unique[i];
        else ++s->repeated[i];
        s->previous[i]=l->source_id[i];
    }
    s->stream_epoch=l->epoch; s->last_transport_seq=l->seq;
    return 0;
}
static void *stream_capture(void *arg)
{
    struct stream *s=arg; struct plate_video_caps caps;
    unsigned char *maps[4]={0}; unsigned mapped=0; int started=0, safe=1;
    FILE *records=NULL;
    int fd=open(s->device,O_RDWR|O_CLOEXEC);
    if(fd<0) { perror("open Stage2 device"); s->producer_error=1; goto finish; }
    if(ioctl(fd,PLATE_VIDEO_GET_CAPS,&caps)<0 || pvs_validate_caps(&caps)) {
        fprintf(stderr,"incompatible Stage2 capability; no DMA START\n"); s->producer_error=1; goto cleanup;
    }
    for(mapped=0;mapped<4;++mapped) {
        void *p=mmap(NULL,1847296,PROT_READ,MAP_SHARED,fd,(off_t)mapped*1847296);
        if(p==MAP_FAILED) { perror("read-only slot mmap"); s->producer_error=1; goto cleanup; }
        maps[mapped]=p;
    }
    if(s->records && !(records=fopen(s->records,"wb"))) { perror("completion records"); s->producer_error=1; goto cleanup; }
    if(s->stats) s->ready_drop_known=stream_read_ready_drops(&s->ready_before)==0;
    struct plate_video_start start={1,0,s->max_fps,500};
    if(ioctl(fd,PLATE_VIDEO_START,&start)<0) { perror("Stage2 START"); s->producer_error=1; goto cleanup; }
    started=1;
    while(!stream_cancelled(s) && (!s->frames || s->acquired<s->frames)) {
        struct pollfd pollfd={fd,POLLIN,0};
        int ready=poll(&pollfd,1,100);
        if(ready<0) { if(errno==EINTR)continue; perror("poll capture"); s->producer_error=1; break; }
        if(!ready) { ++s->poll_timeouts; continue; }
        if(pollfd.revents&(POLLERR|POLLHUP|POLLNVAL)) { fprintf(stderr,"Stage2 stopped/removed/error\n"); s->producer_error=1; break; }
        if(!(pollfd.revents&POLLIN)) continue;
        struct plate_video_lease lease;
        if(ioctl(fd,PLATE_VIDEO_DQBUF,&lease)<0) {
            if(errno==EAGAIN || errno==EINTR)continue;
            perror("DQBUF"); s->producer_error=1; break;
        }
        unsigned char record[128]; struct frame_lease dest; int own_copy=0, valid=0;
        if(lease.slot<4) {
            atomic_thread_fence(memory_order_acquire);
            memcpy(record,maps[lease.slot]+1843200,128);
            valid=stream_record_valid(record,&lease);
        }
        if(!valid || stream_source_counts(s,&lease)) {
            fprintf(stderr,"invalid/short/stale leased completion; pixels not published\n"); s->producer_error=1;
        } else if(frame_exchange_begin_write(s->exchange,&dest)==1) {
            own_copy=1; memcpy(dest.data,maps[lease.slot],1843200);
            uint64_t now=stream_ns();
            if(s->last_received) s->periods[(s->acquired-1)%512]=now-s->last_received;
            s->last_received=now; ++s->acquired;
            if(records && fwrite(record,1,128,records)!=128) { perror("write records"); s->producer_error=1; }
        }
        /* Release the DMA slot only after a full independent image copy. */
        if(ioctl(fd,PLATE_VIDEO_QBUF,&lease)<0) { perror("QBUF"); s->producer_error=1; }
        if(own_copy) {
            if(s->producer_error) frame_exchange_abort_write(s->exchange,&dest);
            else if(frame_exchange_publish(s->exchange,&dest,lease.seq)!=1) s->producer_error=1;
        }
        if(s->producer_error) break;
    }
cleanup:
    if(started && ioctl(fd,PLATE_VIDEO_STOP,NULL)<0) {
        fprintf(stderr,"STOP unconfirmed: driver quarantines DMA RAM; clean reboot required; no explicit munmap/close\n");
        s->producer_error=1; safe=0;
    }
    if(s->ready_drop_known) {
        uint64_t after;
        if(stream_read_ready_drops(&after) || after<s->ready_before) s->ready_drop_known=0;
        else s->ready_drops=after-s->ready_before;
    }
    if(records && fclose(records)) s->producer_error=1;
    if(safe) {
        while(mapped) { --mapped; if(munmap(maps[mapped],1847296)) s->producer_error=1; }
        if(close(fd)) s->producer_error=1;
    }
finish:
    frame_exchange_close(s->exchange); return NULL;
}
static int stream_write(struct stream *s,const unsigned char *data,size_t left)
{
    while(left) {
        if(stream_cancelled(s)) return 1;
        ssize_t n=write(STDOUT_FILENO,data,left);
        if(n<0) {
            if(errno==EINTR)continue;
            if(errno==EAGAIN || errno==EWOULDBLOCK) {
                struct pollfd out={STDOUT_FILENO,POLLOUT,0};
                if(poll(&out,1,50)<0 && errno!=EINTR)return -1;
                continue;
            }
            return -1;
        }
        if(!n){errno=EIO;return -1;} data+=n; left-=(size_t)n;
    }
    return 0;
}
static int stream_dump(const char *path,const unsigned char *frame)
{
    FILE *f=fopen(path,"wb"); if(!f)return -1;
    int fail=fprintf(f,"P6\n1280 720\n255\n")<0;
    unsigned char row[1280*3];
    for(unsigned y=0;y<720 && !fail;++y) {
        for(unsigned x=0;x<1280;++x) {
            size_t p=((size_t)y*1280+x)*2;
            unsigned v=frame[p] | (unsigned)frame[p+1]<<8;
            unsigned r=(v>>11)&31,g=(v>>5)&63,b=v&31;
            row[x*3]=(unsigned char)((r<<3)|(r>>2));
            row[x*3+1]=(unsigned char)((g<<2)|(g>>4));
            row[x*3+2]=(unsigned char)((b<<3)|(b>>2));
        }
        fail=fwrite(row,3,1280,f)!=1280;
    }
    if(fclose(f))fail=1;
    return fail?-1:0;
}
static void *stream_display(void *arg)
{
    struct stream *s=arg; struct frame_lease image;
    while(frame_exchange_acquire_latest(s->exchange,&image)==1) {
        int rc=s->dump?stream_dump(s->dump,image.data):stream_write(s,image.data,1843200);
        frame_exchange_release(s->exchange,&image);
        if(rc<0) { perror("display output"); s->consumer_error=1; atomic_store(&s->cancel,1); frame_exchange_close(s->exchange); break; }
        if(rc>0)break;
        ++s->displayed;
    }
    return NULL;
}
static int stream_period_compare(const void *a,const void *b)
{
    uint64_t x=*(const uint64_t *)a,y=*(const uint64_t *)b; return (x>y)-(x<y);
}
static void stream_report(struct stream *s)
{
    char ready_drops[32];
    if(s->ready_drop_known)snprintf(ready_drops,sizeof(ready_drops),"%llu",(unsigned long long)s->ready_drops);
    else strcpy(ready_drops,"unavailable");
    unsigned n=s->acquired>1?(s->acquired-1<512?(unsigned)s->acquired-1:512):0;
    qsort(s->periods,n,sizeof(s->periods[0]),stream_period_compare);
    double elapsed=(stream_ns()-s->started)/1e9;
    fprintf(stderr,"Stage2: acquired=%llu displayed=%llu display_drops=%llu transport_fps=%.2f output_fps=%.2f "
            "p50_period_ms=%.2f p95_period_ms=%.2f poll_timeouts=%llu errors=%d "
            "ready_drops=%s source_unique_counts=[%llu,%llu,%llu,%llu] source_repeated_counts=[%llu,%llu,%llu,%llu] "
            "unique_source_fps=[%.2f,%.2f,%.2f,%.2f]\n",
            (unsigned long long)s->acquired,(unsigned long long)s->displayed,
            (unsigned long long)frame_exchange_dropped(s->exchange),s->acquired/elapsed,s->displayed/elapsed,
            n?s->periods[(n*50+99)/100-1]/1e6:0,n?s->periods[(n*95+99)/100-1]/1e6:0,
            (unsigned long long)s->poll_timeouts,s->producer_error+s->consumer_error,ready_drops,
            (unsigned long long)s->unique[0],(unsigned long long)s->unique[1],(unsigned long long)s->unique[2],(unsigned long long)s->unique[3],
            (unsigned long long)s->repeated[0],(unsigned long long)s->repeated[1],(unsigned long long)s->repeated[2],(unsigned long long)s->repeated[3],
            s->unique[0]/elapsed,s->unique[1]/elapsed,s->unique[2]/elapsed,s->unique[3]/elapsed);
}
int main(int argc,char **argv)
{
    struct stream s={.device="/dev/plate_video_dma",.max_fps=25};
    const struct option options[]={ {"device",required_argument,0,'d'}, {"frames",required_argument,0,'n'},
        {"max-fps",required_argument,0,'f'}, {"stdout-rgb565",no_argument,0,'o'},
        {"dump-frame",required_argument,0,'p'}, {"stats",no_argument,0,'s'}, {"records",required_argument,0,'r'}, {0,0,0,0} };
    int opt; stream_signal=0; optind=1;
    while((opt=getopt_long(argc,argv,"",options,NULL))!=-1) {
        char *end=NULL; unsigned long value;
        switch(opt) {
        case 'd':s.device=optarg;break;
        case 'o':s.stdout_pixels=1;break;
        case 'p':s.dump=optarg;break;
        case 'r':s.records=optarg;break;
        case 's':s.stats=1;break;
        case 'n':case 'f':
            errno=0; value=strtoul(optarg,&end,10);
            if(errno || *end || *optarg=='-' || value>0xffffffffUL || (opt=='f' && (!value || value>60)))return 2;
            if(opt=='n')s.frames=(unsigned)value;else s.max_fps=(unsigned)value;break;
        default:return 2;
        }
    }
    if(optind!=argc || (!!s.dump + !!s.stdout_pixels)!=1)return 2;
    if(s.dump)s.frames=1;
    signal(SIGPIPE,SIG_IGN);
    struct sigaction action={0}; action.sa_handler=stream_on_signal; sigemptyset(&action.sa_mask);
    sigaction(SIGINT,&action,NULL); sigaction(SIGTERM,&action,NULL); sigaction(SIGHUP,&action,NULL);
    int original=-1;
    if(s.stdout_pixels) { original=fcntl(STDOUT_FILENO,F_GETFL); if(original<0 || fcntl(STDOUT_FILENO,F_SETFL,original|O_NONBLOCK)<0)return 1; }
    s.exchange=frame_exchange_create(1843200); if(!s.exchange)return 1;
    atomic_init(&s.cancel,0); s.started=stream_ns(); pthread_t capture,display;
    if(pthread_create(&capture,NULL,stream_capture,&s)) { frame_exchange_destroy(s.exchange); return 1; }
    if(pthread_create(&display,NULL,stream_display,&s)) {
        atomic_store(&s.cancel,1); frame_exchange_close(s.exchange); pthread_join(capture,NULL);
        frame_exchange_destroy(s.exchange); return 1;
    }
    pthread_join(capture,NULL); pthread_join(display,NULL);
    if(s.stats)stream_report(&s);
    frame_exchange_destroy(s.exchange);
    if(original>=0)fcntl(STDOUT_FILENO,F_SETFL,original);
    return s.producer_error || s.consumer_error ? 1:0;
}
