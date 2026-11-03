// SPDX-License-Identifier: GPL-2.0
/* Stage2 only. No legacy IOCTLs, user DMA addresses, writable BAR/DMA mmap,
 * force-unload recovery, or IRQ-as-completion shortcut. */
#include <linux/dma-mapping.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/kref.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pci.h>
#include <linux/poll.h>
#include <linux/random.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include "plate_video_state.h"

#define DRIVER_NAME "plate_video_dma"
#define STOP_MS 2000U
struct plate_dma_slot { void *cpu; dma_addr_t dma; };
struct plate_device {
    struct pci_dev *pdev;
    struct kref ref;
    struct mutex lock;
    struct pvs_state state;
    struct plate_video_caps caps;
    struct plate_dma_slot slot[4];
    void __iomem *bar0, *bar1;
    struct miscdevice misc;
    struct delayed_work work;
    wait_queue_head_t wait;
    unsigned long submitted_at, stop_at, next_at, period;
    unsigned timeout_ms;
    int irq;
    bool online, pinned, have_irq;
};
/* A quarantined endpoint must not be rebound/retried before clean reboot. */
static atomic_t quarantined = ATOMIC_INIT(0);
static ssize_t ready_drops_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct plate_device *d=dev_get_drvdata(dev); u64 drops;
    (void)attr;
    mutex_lock(&d->lock); drops=d->state.ready_drops; mutex_unlock(&d->lock);
    return sysfs_emit(buf,"%llu\n",(unsigned long long)drops);
}
static DEVICE_ATTR_RO(ready_drops);

static void plate_free(struct kref *ref)
{
    struct plate_device *d = container_of(ref, struct plate_device, ref);
    unsigned i;
    /* A failed stop owns an extra reference forever; never reaches here. */
    if (WARN_ON(!pvs_can_free(&d->state))) return;
    for (i=0; i<4; ++i)
        if (d->slot[i].cpu)
            dma_free_coherent(&d->pdev->dev, PLATE_VIDEO_SLOT_STRIDE,
                              d->slot[i].cpu, d->slot[i].dma);
    pci_dev_put(d->pdev); kfree(d);
}
static void plate_quarantine_locked(struct plate_device *d, const char *why)
{
    if (d->pinned) return;
    d->state.run=PVS_FAILED_QUARANTINED;
    d->pinned=true; atomic_set(&quarantined,1);
    kref_get(&d->ref); __module_get(THIS_MODULE);
    dev_crit(&d->pdev->dev, "FAILED_QUARANTINED: %s; coherent RAM/module retained; clean reboot required\n", why);
    wake_up_all(&d->wait);
}
static void plate_read_caps(struct plate_device *d, struct plate_video_caps *c)
{
    unsigned i; u32 *w=(u32 *)c;
    for (i=0; i<16; ++i) w[i]=readl(d->bar0+PLATE_VIDEO_BAR_CAPS+4*i);
}
static void plate_decode_record(struct plate_video_record *r)
{
    unsigned i;
    r->magic=le32_to_cpu((__force __le32)r->magic);
    r->major=le32_to_cpu((__force __le32)r->major);
    r->epoch=le32_to_cpu((__force __le32)r->epoch);
    r->slot=le32_to_cpu((__force __le32)r->slot);
    r->seq=le64_to_cpu((__force __le64)r->seq);
    r->bytes=le32_to_cpu((__force __le32)r->bytes);
    r->status=le32_to_cpu((__force __le32)r->status);
    for (i=0;i<4;++i) {
        r->source_id[i]=le32_to_cpu((__force __le32)r->source_id[i]);
        r->source_tick[i]=le64_to_cpu((__force __le64)r->source_tick[i]);
    }
    r->repeated_source_mask=le32_to_cpu((__force __le32)r->repeated_source_mask);
    r->format=le32_to_cpu((__force __le32)r->format);
    r->acquire_tick=le64_to_cpu((__force __le64)r->acquire_tick);
    r->commit_cookie=le32_to_cpu((__force __le32)r->commit_cookie);
}
static int plate_complete_locked(struct plate_device *d)
{
    struct plate_video_record record, *dma_record;
    u32 cookie; int ret, slot=d->state.device_slot;
    if (slot<0) return -EAGAIN;
    dma_record=d->slot[slot].cpu+PLATE_VIDEO_FRAME_BYTES;
    cookie=le32_to_cpu((__force __le32)READ_ONCE(dma_record->commit_cookie));
    if (!cookie || cookie!=d->state.slot[slot].lease.cookie) return -EAGAIN;
    /* Coherent does not remove the need for ordering after the commit word. */
    dma_rmb(); memcpy(&record,dma_record,sizeof(record)); plate_decode_record(&record);
    ret=pvs_complete(&d->state,&record);
    if (ret==-EPROTO) plate_quarantine_locked(d,"committed record fails identity/size/format validation");
    if (!ret) wake_up_all(&d->wait);
    return ret;
}
static void plate_request_stop_locked(struct plate_device *d)
{
    pvs_stop(&d->state);
    if (d->state.device_slot>=0 && d->online) {
        if (!d->stop_at) d->stop_at=jiffies+msecs_to_jiffies(STOP_MS);
        writel(PLATE_VIDEO_DOORBELL_STOP,d->bar1+PLATE_VIDEO_BAR_COMMAND+0x2c);
        readl(d->bar0+PLATE_VIDEO_BAR_STATE); /* flush posted MMIO, not DMA quiesce */
    }
}
static int plate_submit_locked(struct plate_device *d)
{
    u32 words[12]; int slot, ret; unsigned i;
    /* Never submit while the previous terminal MSI/command remains in flight. */
    if (readl(d->bar0+PLATE_VIDEO_BAR_STATE) ||
        readl(d->bar0+PLATE_VIDEO_BAR_STATE+4)) return -EBUSY;
    if (!readl(d->bar0+PLATE_VIDEO_BAR_STATE+8)) return -EIO;
    slot=pvs_submit(&d->state); if (slot<0) return slot;
    ret=pvs_command(&d->state.slot[slot].lease,d->slot[slot].dma,words);
    if (ret) { /* No descriptor was written: no possible device ownership. */
        d->state.slot[slot].owner=PVS_FREE; d->state.device_slot=-1; return ret;
    }
    memset(d->slot[slot].cpu+PLATE_VIDEO_FRAME_BYTES,0,PLATE_VIDEO_RECORD_BYTES);
    dma_wmb();
    for (i=0;i<11;++i) writel(words[i],d->bar1+PLATE_VIDEO_BAR_COMMAND+4*i);
    wmb(); writel(words[11],d->bar1+PLATE_VIDEO_BAR_COMMAND+44);
    readl(d->bar0+PLATE_VIDEO_BAR_STATE);
    d->submitted_at=jiffies; d->stop_at=0; d->next_at=jiffies+d->period;
    return 0;
}
static void plate_work(struct work_struct *work)
{
    struct plate_device *d=container_of(to_delayed_work(work),struct plate_device,work);
    int ret; bool again=false;
    mutex_lock(&d->lock);
    if (!d->online || d->state.run==PVS_FAILED_QUARANTINED) goto out;
    if (d->state.device_slot>=0) {
        ret=plate_complete_locked(d);
        if (ret==-EAGAIN && d->state.run==PVS_RUNNING &&
            time_after_eq(jiffies,d->submitted_at+msecs_to_jiffies(d->timeout_ms)))
            plate_request_stop_locked(d);
        if (d->state.device_slot>=0 && d->state.run==PVS_STOPPING && d->stop_at &&
            time_after_eq(jiffies,d->stop_at))
            plate_quarantine_locked(d,"STOP has no ordered terminal completion after 2000ms");
    }
    if (d->state.run==PVS_RUNNING && d->state.device_slot<0 &&
        time_after_eq(jiffies,d->next_at)) {
        ret=plate_submit_locked(d);
        if (ret<0 && ret!=-EBUSY && ret!=-ENOSPC) {
            pvs_stop(&d->state); wake_up_all(&d->wait);
            dev_err(&d->pdev->dev,"submission rejected: %d\n",ret);
        }
    }
    again=d->state.run==PVS_RUNNING || d->state.run==PVS_STOPPING;
out:
    /* Schedule under the lock so remove cannot race a post-cancel requeue. */
    if (again) mod_delayed_work(system_wq,&d->work,msecs_to_jiffies(1));
    mutex_unlock(&d->lock);
}
static irqreturn_t plate_irq(int irq, void *arg)
{
    struct plate_device *d=arg;
    (void)irq;
    /* IRQ is a hint; the worker validates cookie+dma_rmb+full record. */
    if (READ_ONCE(d->online)) mod_delayed_work(system_wq,&d->work,0);
    return IRQ_HANDLED;
}
static int plate_stop(struct plate_device *d)
{
    long waited; int ret=0;
    mutex_lock(&d->lock);
    if (d->state.run==PVS_FAILED_QUARANTINED) { ret=-EIO; goto unlock; }
    if (d->state.device_slot>=0 && !d->online) {
        plate_quarantine_locked(d,"device removed during active DMA"); ret=-EIO; goto unlock;
    }
    plate_request_stop_locked(d);
    if (d->state.device_slot<0) goto unlock;
    mod_delayed_work(system_wq,&d->work,0); mutex_unlock(&d->lock);
    waited=wait_event_timeout(d->wait,READ_ONCE(d->state.device_slot)<0 ||
        READ_ONCE(d->state.run)==PVS_FAILED_QUARANTINED,msecs_to_jiffies(STOP_MS));
    mutex_lock(&d->lock);
    if (!waited && d->state.device_slot>=0)
        plate_quarantine_locked(d,"synchronous STOP timeout");
    if (d->state.run==PVS_FAILED_QUARANTINED) ret=-EIO;
unlock:
    mutex_unlock(&d->lock); return ret;
}
static int plate_open(struct inode *inode, struct file *file)
{
    struct miscdevice *misc=file->private_data;
    struct plate_device *d=container_of(misc,struct plate_device,misc); int ret;
    (void)inode; mutex_lock(&d->lock);
    ret=d->online ? pvs_open(&d->state) : -ENODEV;
    if (!ret) { kref_get(&d->ref); file->private_data=d; }
    mutex_unlock(&d->lock); return ret;
}
static int plate_release(struct inode *inode, struct file *file)
{
    struct plate_device *d=file->private_data;
    (void)inode; plate_stop(d);
    mutex_lock(&d->lock); pvs_close(&d->state); mutex_unlock(&d->lock);
    kref_put(&d->ref,plate_free); return 0;
}
static long plate_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    struct plate_device *d=file->private_data; void __user *u=(void __user *)arg;
    struct plate_video_start start; struct plate_video_lease lease;
    struct plate_video_caps caps; u32 epoch; int ret=0;
    if (cmd==PLATE_VIDEO_STOP) return plate_stop(d);
    if (cmd==PLATE_VIDEO_START) {
        if (copy_from_user(&start,u,sizeof(start))) return -EFAULT;
        if (start.major!=1 || start.flags || !start.max_submission_fps ||
            start.max_submission_fps>60 || start.timeout_ms<50 || start.timeout_ms>2000)
            return -EINVAL;
    } else if (cmd==PLATE_VIDEO_QBUF && copy_from_user(&lease,u,sizeof(lease))) return -EFAULT;
    mutex_lock(&d->lock);
    if (!d->online || d->state.detached) { ret=-ENODEV; goto out; }
    if (d->state.run==PVS_FAILED_QUARANTINED) { ret=-EIO; goto out; }
    switch(cmd) {
    case PLATE_VIDEO_GET_CAPS:
        plate_read_caps(d,&caps); ret=pvs_validate_caps(&caps);
        if (!ret && copy_to_user(u,&caps,sizeof(caps))) ret=-EFAULT;
        break;
    case PLATE_VIDEO_START:
        plate_read_caps(d,&caps); ret=pvs_validate_caps(&caps); if (ret) break;
        if (!d->have_irq || !d->pdev->msi_enabled) { ret=-ENXIO; break; }
        do { epoch=get_random_u32(); } while (!epoch || epoch==d->state.epoch);
        ret=pvs_start(&d->state,epoch); if (ret) break;
        pci_set_master(d->pdev);
        d->period=max_t(unsigned long,1,DIV_ROUND_UP(HZ,start.max_submission_fps));
        d->timeout_ms=start.timeout_ms; d->next_at=jiffies; d->stop_at=0;
        mod_delayed_work(system_wq,&d->work,0); break;
    case PLATE_VIDEO_DQBUF:
        ret=pvs_dequeue(&d->state,&lease);
        if (!ret && copy_to_user(u,&lease,sizeof(lease))) {
            /* User never got this lease: return it to READY, not DEVICE. */
            d->state.slot[lease.slot].owner=PVS_READY; ret=-EFAULT;
        }
        break;
    case PLATE_VIDEO_QBUF:
        ret=pvs_queue(&d->state,&lease);
        if (!ret && d->state.run==PVS_RUNNING) mod_delayed_work(system_wq,&d->work,0);
        break;
    default: ret=-ENOTTY;
    }
out:
    mutex_unlock(&d->lock); return ret;
}
static __poll_t plate_poll(struct file *file, poll_table *wait)
{
    struct plate_device *d=file->private_data; __poll_t mask=0; unsigned i;
    poll_wait(file,&d->wait,wait); mutex_lock(&d->lock);
    if (!d->online || d->state.detached || d->state.run==PVS_FAILED_QUARANTINED) mask=EPOLLERR|EPOLLHUP;
    else {
        for (i=0;i<4;++i) if (d->state.slot[i].owner==PVS_READY) mask|=EPOLLIN|EPOLLRDNORM;
        if (d->state.run==PVS_IDLE) mask|=EPOLLHUP;
    }
    mutex_unlock(&d->lock); return mask;
}
static void plate_vma_open(struct vm_area_struct *vma)
{
    struct plate_device *d=vma->vm_private_data;
    __module_get(THIS_MODULE); kref_get(&d->ref);
    mutex_lock(&d->lock); pvs_map_get(&d->state); mutex_unlock(&d->lock);
}
static void plate_vma_close(struct vm_area_struct *vma)
{
    struct plate_device *d=vma->vm_private_data;
    mutex_lock(&d->lock); pvs_map_put(&d->state); mutex_unlock(&d->lock);
    kref_put(&d->ref,plate_free); module_put(THIS_MODULE);
}
static const struct vm_operations_struct plate_vm_ops={.open=plate_vma_open,.close=plate_vma_close};
static int plate_mmap(struct file *file, struct vm_area_struct *vma)
{
    struct plate_device *d=file->private_data;
    int ret, slot;
    if (PAGE_SIZE!=4096 || vma->vm_pgoff>(~0ULL>>PAGE_SHIFT)) return -EINVAL;
    slot=pvs_map_index((u64)vma->vm_pgoff<<PAGE_SHIFT,vma->vm_end-vma->vm_start,
                      !!(vma->vm_flags&VM_WRITE),!!(vma->vm_flags&VM_EXEC));
    if (slot<0) return slot;
    mutex_lock(&d->lock);
    if (!d->online || d->state.detached || d->state.run==PVS_FAILED_QUARANTINED) { ret=-ENODEV; goto out; }
    vma->vm_flags&=~(VM_MAYWRITE|VM_MAYEXEC);
    vma->vm_flags|=VM_DONTEXPAND|VM_DONTDUMP;
    vma->vm_pgoff=0; /* This call maps one independently allocated slot. */
    ret=dma_mmap_coherent(&d->pdev->dev,vma,d->slot[slot].cpu,d->slot[slot].dma,PLATE_VIDEO_SLOT_STRIDE);
    if (!ret) {
        vma->vm_private_data=d; vma->vm_ops=&plate_vm_ops;
        __module_get(THIS_MODULE); kref_get(&d->ref); pvs_map_get(&d->state);
    }
out:
    mutex_unlock(&d->lock); return ret;
}
static const struct file_operations plate_fops={
    .owner=THIS_MODULE,.open=plate_open,.release=plate_release,
    .unlocked_ioctl=plate_ioctl,.compat_ioctl=plate_ioctl,.poll=plate_poll,
    .mmap=plate_mmap,.llseek=no_llseek,
};
static int plate_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
    struct plate_device *d; unsigned i; int ret;
    (void)id;
    if (atomic_read(&quarantined)) return -EIO;
    /* Refuse the factory 64KiB example before enabling bus mastering. */
    if (pci_resource_len(pdev,0)!=8192 || pci_resource_len(pdev,1)!=4096 ||
        pci_resource_len(pdev,2)!=8192 || !pci_find_capability(pdev,PCI_CAP_ID_MSI)) return -ENODEV;
    ret=pci_enable_device_mem(pdev); if (ret) return ret;
    pci_clear_master(pdev);
    ret=pci_request_regions(pdev,DRIVER_NAME); if (ret) goto disable;
    d=kzalloc(sizeof(*d),GFP_KERNEL); if (!d) { ret=-ENOMEM; goto regions; }
    d->pdev=pci_dev_get(pdev); kref_init(&d->ref); mutex_init(&d->lock);
    pvs_init(&d->state); init_waitqueue_head(&d->wait); INIT_DELAYED_WORK(&d->work,plate_work);
    d->bar0=pci_iomap(pdev,0,0); d->bar1=pci_iomap(pdev,1,0);
    if (!d->bar0 || !d->bar1) { ret=-ENOMEM; goto unmap; }
    plate_read_caps(d,&d->caps); ret=pvs_validate_caps(&d->caps); if (ret) goto unmap;
    if (readl(d->bar0+PLATE_VIDEO_BAR_STATE) || readl(d->bar0+PLATE_VIDEO_BAR_STATE+4)) {
        ret=-EBUSY; goto unmap; /* only bind a clean idle endpoint */
    }
    ret=dma_set_mask_and_coherent(&pdev->dev,DMA_BIT_MASK(32)); if (ret) goto unmap;
    for (i=0;i<4;++i) {
        d->slot[i].cpu=dma_alloc_coherent(&pdev->dev,PLATE_VIDEO_SLOT_STRIDE,&d->slot[i].dma,GFP_KERNEL);
        if (!d->slot[i].cpu) { ret=-ENOMEM; goto unmap; }
        if (d->slot[i].dma>0xffe3d000ULL || (d->slot[i].dma&4095)) { ret=-ERANGE; goto unmap; }
        memset(d->slot[i].cpu,0,PLATE_VIDEO_SLOT_STRIDE);
    }
    ret=pci_alloc_irq_vectors(pdev,1,1,PCI_IRQ_MSI); if (ret<0) goto unmap;
    d->irq=pci_irq_vector(pdev,0);
    ret=request_irq(d->irq,plate_irq,0,DRIVER_NAME,d); if (ret) goto vectors;
    d->have_irq=true; d->online=true;
    d->misc.minor=MISC_DYNAMIC_MINOR; d->misc.name=DRIVER_NAME;
    d->misc.fops=&plate_fops; d->misc.parent=&pdev->dev;
    ret=misc_register(&d->misc); if (ret) goto irq;
    dev_set_drvdata(d->misc.this_device,d);
    ret=device_create_file(d->misc.this_device,&dev_attr_ready_drops);
    if (ret) { misc_deregister(&d->misc); goto irq; }
    pci_set_drvdata(pdev,d);
    dev_info(&pdev->dev,"Stage2 v1 four coherent slots, 32-bit DMA, MSI IRQ %d; stopped until START\n",d->irq);
    return 0;
irq:
    d->online=false; free_irq(d->irq,d); cancel_delayed_work_sync(&d->work);
vectors:
    pci_free_irq_vectors(pdev);
unmap:
    if (d->bar0) pci_iounmap(pdev,d->bar0);
    if (d->bar1) pci_iounmap(pdev,d->bar1);
    kref_put(&d->ref,plate_free);
regions:
    pci_release_regions(pdev);
disable:
    pci_disable_device(pdev); return ret;
}
static void plate_remove(struct pci_dev *pdev)
{
    struct plate_device *d=pci_get_drvdata(pdev);
    if (!d) return;
    /* Reject START from an existing fd before dropping the misc node or waiting. */
    mutex_lock(&d->lock); pvs_detach(&d->state); mutex_unlock(&d->lock);
    device_remove_file(d->misc.this_device,&dev_attr_ready_drops);
    misc_deregister(&d->misc); plate_stop(d);
    mutex_lock(&d->lock); d->online=false; mutex_unlock(&d->lock);
    wake_up_all(&d->wait);
    free_irq(d->irq,d); cancel_delayed_work_sync(&d->work);
    pci_free_irq_vectors(pdev);
    /* Containment only: this does NOT justify freeing unconfirmed DMA RAM. */
    pci_clear_master(pdev);
    pci_iounmap(pdev,d->bar0); pci_iounmap(pdev,d->bar1);
    pci_release_regions(pdev); pci_disable_device(pdev); pci_set_drvdata(pdev,NULL);
    kref_put(&d->ref,plate_free); /* VMAs/file/quarantine keep RAM and device alive */
}
static const struct pci_device_id plate_ids[]={ {PCI_DEVICE(0x0755,0x0755)}, {0} };
MODULE_DEVICE_TABLE(pci,plate_ids);
static struct pci_driver plate_driver={.name=DRIVER_NAME,.id_table=plate_ids,.probe=plate_probe,.remove=plate_remove};
module_pci_driver(plate_driver);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Stage2 four-camera ordered-frame DMA with MSI and read-only leased slots");
MODULE_VERSION("1.0");
