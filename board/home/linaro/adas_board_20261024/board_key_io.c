/* Read-only board keys: logical KEY0=M16, KEY1=ARM GPIO0_B0. */
#define _POSIX_C_SOURCE 200809L
#include <errno.h>
#include <fcntl.h>
#include <linux/gpio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

struct keys { volatile uint32_t *bar; int input; };
void adas_keys_close(void *handle) {
    struct keys *h=handle;
    if (!h) return;
    if (h->input>=0) close(h->input);
    if (h->bar!=MAP_FAILED) munmap((void *)h->bar,8192);
    free(h);
}
void *adas_keys_open(void) {
    struct keys *h=calloc(1,sizeof(*h));
    if (!h) return NULL;
    h->bar=MAP_FAILED;h->input=-1;
    int fd=open("/sys/bus/pci/devices/0002:21:00.0/resource0",O_RDONLY|O_SYNC|O_CLOEXEC);
    if (fd<0) goto fail;
    h->bar=mmap(NULL,8192,PROT_READ,MAP_SHARED,fd,0);close(fd);
    if (h->bar==MAP_FAILED) goto fail;
    if (h->bar[0x1d00/4]!=0x52505331U) { errno=ENODEV;goto fail; }
    fd=open("/dev/gpiochip0",O_RDONLY|O_CLOEXEC);
    if (fd<0) goto fail;
    struct gpiochip_info info={0};
    if (ioctl(fd,GPIO_GET_CHIPINFO_IOCTL,&info)<0 || strcmp(info.label,"gpio0")) {
        close(fd);errno=ENODEV;goto fail;
    }
    struct gpiohandle_request request={0};
    request.lines=1;request.lineoffsets[0]=8;request.flags=GPIOHANDLE_REQUEST_INPUT;
    strcpy(request.consumer_label,"adas-ordinary-user-key");
    if (ioctl(fd,GPIO_GET_LINEHANDLE_IOCTL,&request)<0) { close(fd);goto fail; }
    close(fd);h->input=request.fd;return h;
fail: {
    int saved=errno;adas_keys_close(h);errno=saved;return NULL;
}}
int adas_keys_read(void *handle) {
    struct keys *h=handle;
    if (!h || h->bar[0x1d00/4]!=0x52505331U) { errno=ENODEV;return -1; }
    struct gpiohandle_data data={0};
    if (ioctl(h->input,GPIOHANDLE_GET_LINE_VALUES_IOCTL,&data)<0) return -1;
    unsigned fpga_pressed=(h->bar[0x1d04/4]>>8)&1U;
    return (int)((1U-fpga_pressed)|((unsigned)data.values[0]<<1));
}
