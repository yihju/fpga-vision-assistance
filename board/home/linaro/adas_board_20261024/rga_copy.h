/* Copy identical BGR bytes from uncached MPP DMA to CPU-cached DMA memory. */
#include <gst/allocators/gstdmabuf.h>
#include <rga/im2d.h>
#include <linux/dma-heap.h>
#include <linux/dma-buf.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
typedef struct {int fd;void *pointer;size_t bytes;int stride;} CachedCopy;
static void cached_copy_close(CachedCopy *cache){
    if(cache->bytes){munmap(cache->pointer,cache->bytes);close(cache->fd);cache->bytes=0;}
}
static int cached_copy(CachedCopy *cache,GstBuffer *buffer,int w,int h,int source_stride,void *destination){
    if(gst_buffer_n_memory(buffer)!=1||source_stride%3)return -1;
    GstMemory *memory=gst_buffer_peek_memory(buffer,0);
    if(!gst_is_dmabuf_memory(memory)||memory->offset)return -2;
    int source_fd=gst_dmabuf_memory_get_fd(memory);if(source_fd<0)return -3;
    int stride=(w+15)&~15;
    if(!cache->bytes){
        int heap=open("/dev/dma_heap/system",O_RDWR|O_CLOEXEC);if(heap<0)return -4;
        struct dma_heap_allocation_data allocation={0};allocation.len=(size_t)stride*h*3;
        allocation.fd_flags=O_RDWR|O_CLOEXEC;
        int rc=ioctl(heap,DMA_HEAP_IOCTL_ALLOC,&allocation);close(heap);if(rc<0)return -5;
        cache->fd=allocation.fd;
        cache->pointer=mmap(NULL,allocation.len,PROT_READ|PROT_WRITE,MAP_SHARED,cache->fd,0);
        if(cache->pointer==MAP_FAILED){close(cache->fd);return -6;}
        cache->bytes=allocation.len;cache->stride=stride;
    }
    if(cache->stride!=stride||cache->bytes<(size_t)stride*h*3)return -7;
    rga_buffer_t source=wrapbuffer_fd_t(source_fd,w,h,source_stride/3,h,RK_FORMAT_BGR_888);
    rga_buffer_t target=wrapbuffer_fd_t(cache->fd,w,h,stride,h,RK_FORMAT_BGR_888);
    IM_STATUS copied=imcopy_t(source,target,1);
    if(copied!=IM_STATUS_SUCCESS){g_printerr("RGA copy: %s\n",imStrError(copied));return -8;}
    struct dma_buf_sync sync={.flags=DMA_BUF_SYNC_READ|DMA_BUF_SYNC_START};
    if(ioctl(cache->fd,DMA_BUF_IOCTL_SYNC,&sync)<0)return -9;
    for(int row=0;row<h;row++)memcpy((char*)destination+(size_t)row*w*3,(char*)cache->pointer+(size_t)row*stride*3,w*3);
    sync.flags=DMA_BUF_SYNC_READ|DMA_BUF_SYNC_END;
    if(ioctl(cache->fd,DMA_BUF_IOCTL_SYNC,&sync)<0)return -10;
    return 0;
}
