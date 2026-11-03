#ifndef FRAME_EXCHANGE_H
#define FRAME_EXCHANGE_H
#include <stddef.h>
#include <stdint.h>

struct frame_exchange;
/* generation is an ownership token, seq is the host transport sequence. */
struct frame_lease {
    unsigned slot;
    uint64_t seq, generation;
    unsigned char *data;
};

struct frame_exchange *frame_exchange_create(size_t frame_bytes);
/* 1 = lease obtained; 0 = closed; -1 = invalid argument/lease. */
int frame_exchange_begin_write(struct frame_exchange *, struct frame_lease *);
int frame_exchange_publish(struct frame_exchange *, const struct frame_lease *, uint64_t seq);
void frame_exchange_abort_write(struct frame_exchange *, const struct frame_lease *);
/* Drains the last READY frame on close, then returns 0. */
int frame_exchange_acquire_latest(struct frame_exchange *, struct frame_lease *);
void frame_exchange_release(struct frame_exchange *, const struct frame_lease *);
void frame_exchange_close(struct frame_exchange *);
/* Caller must join all users before destroy. close does not free pixels. */
void frame_exchange_destroy(struct frame_exchange *);
uint64_t frame_exchange_dropped(struct frame_exchange *);
#endif
