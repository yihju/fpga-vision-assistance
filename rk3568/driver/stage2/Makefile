ifneq ($(KERNELRELEASE),)
obj-m += plate_video_dma.o
plate_video_dma-y := plate_video_pci.o plate_video_state.o
ccflags-y += -I$(src)/../../include
else
KDIR ?= /usr/src/linux-headers-6.1-rockchip
.PHONY: all clean
all:
	$(MAKE) -C $(KDIR) M=$(CURDIR) modules
clean:
	$(MAKE) -C $(KDIR) M=$(CURDIR) clean
endif
