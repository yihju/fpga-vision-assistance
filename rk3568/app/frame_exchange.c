#include "frame_exchange.h"
#include <errno.h>
#include <pthread.h>
#include <stdlib.h>

enum slot_state { FREE, FILLING, READY, IN_USE };
struct slot {
    enum slot_state state;
    struct frame_lease lease;
};
struct frame_exchange {
    pthread_mutex_t mutex;
    pthread_cond_t changed;
    struct slot slots[2];
    uint64_t generation, dropped;
    int closed;
};

static uint64_t next_generation(struct frame_exchange *x)
{
    if (++x->generation == 0) ++x->generation;
    return x->generation;
}

static struct slot *match(struct frame_exchange *x, const struct frame_lease *lease,
                          enum slot_state state)
{
    if (!lease || lease->slot >= 2) return NULL;
    struct slot *s = &x->slots[lease->slot];
    return s->state == state && s->lease.generation == lease->generation &&
           s->lease.data == lease->data && s->lease.seq == lease->seq ? s : NULL;
}

struct frame_exchange *frame_exchange_create(size_t bytes)
{
    if (!bytes) { errno = EINVAL; return NULL; }
    struct frame_exchange *x = calloc(1, sizeof(*x));
    if (!x) return NULL;
    int error = pthread_mutex_init(&x->mutex, NULL);
    if (error) { free(x); errno = error; return NULL; }
    error = pthread_cond_init(&x->changed, NULL);
    if (error) {
        pthread_mutex_destroy(&x->mutex); free(x); errno = error; return NULL;
    }
    for (unsigned i = 0; i < 2; ++i) {
        x->slots[i].lease.slot = i;
        x->slots[i].lease.data = malloc(bytes);
        if (!x->slots[i].lease.data) {
            frame_exchange_destroy(x);
            return NULL;
        }
    }
    return x;
}

int frame_exchange_begin_write(struct frame_exchange *x, struct frame_lease *out)
{
    if (!x || !out) { errno = EINVAL; return -1; }
    pthread_mutex_lock(&x->mutex);
    while (!x->closed) {
        struct slot *s = NULL;
        for (unsigned i = 0; i < 2; ++i)
            if (x->slots[i].state == FREE) { s = &x->slots[i]; break; }
        if (!s) {
            for (unsigned i = 0; i < 2; ++i)
                if (x->slots[i].state == READY) {
                    s = &x->slots[i]; ++x->dropped; break;
                }
        }
        if (s) {
            s->state = FILLING;
            s->lease.seq = 0;
            s->lease.generation = next_generation(x);
            *out = s->lease;
            pthread_mutex_unlock(&x->mutex);
            return 1;
        }
        pthread_cond_wait(&x->changed, &x->mutex);
    }
    pthread_mutex_unlock(&x->mutex);
    return 0;
}

int frame_exchange_publish(struct frame_exchange *x, const struct frame_lease *lease,
                            uint64_t seq)
{
    if (!x) { errno = EINVAL; return -1; }
    pthread_mutex_lock(&x->mutex);
    struct slot *s = match(x, lease, FILLING);
    int result;
    if (!s) { errno = EINVAL; result = -1; }
    else if (x->closed) { s->state = FREE; result = 0; }
    else {
        for (unsigned i = 0; i < 2; ++i) {
            if (x->slots[i].state == READY) {
                x->slots[i].state = FREE;
                ++x->dropped;
            }
        }
        s->lease.seq = seq;
        s->state = READY;
        result = 1;
    }
    pthread_cond_broadcast(&x->changed);
    pthread_mutex_unlock(&x->mutex);
    return result;
}

void frame_exchange_abort_write(struct frame_exchange *x, const struct frame_lease *lease)
{
    if (!x) return;
    pthread_mutex_lock(&x->mutex);
    struct slot *s = match(x, lease, FILLING);
    if (s) { s->state = FREE; pthread_cond_broadcast(&x->changed); }
    pthread_mutex_unlock(&x->mutex);
}

int frame_exchange_acquire_latest(struct frame_exchange *x, struct frame_lease *out)
{
    if (!x || !out) { errno = EINVAL; return -1; }
    pthread_mutex_lock(&x->mutex);
    for (;;) {
        for (unsigned i = 0; i < 2; ++i) {
            struct slot *s = &x->slots[i];
            if (s->state == READY) {
                s->state = IN_USE;
                /* A writer lease must never release a consumer acquisition. */
                s->lease.generation = next_generation(x);
                *out = s->lease;
                pthread_mutex_unlock(&x->mutex);
                return 1;
            }
        }
        if (x->closed) { pthread_mutex_unlock(&x->mutex); return 0; }
        pthread_cond_wait(&x->changed, &x->mutex);
    }
}

void frame_exchange_release(struct frame_exchange *x, const struct frame_lease *lease)
{
    if (!x) return;
    pthread_mutex_lock(&x->mutex);
    struct slot *s = match(x, lease, IN_USE);
    if (s) { s->state = FREE; pthread_cond_broadcast(&x->changed); }
    pthread_mutex_unlock(&x->mutex);
}

void frame_exchange_close(struct frame_exchange *x)
{
    if (!x) return;
    pthread_mutex_lock(&x->mutex);
    x->closed = 1;
    pthread_cond_broadcast(&x->changed);
    pthread_mutex_unlock(&x->mutex);
}

uint64_t frame_exchange_dropped(struct frame_exchange *x)
{
    if (!x) return 0;
    pthread_mutex_lock(&x->mutex);
    uint64_t result = x->dropped;
    pthread_mutex_unlock(&x->mutex);
    return result;
}

void frame_exchange_destroy(struct frame_exchange *x)
{
    if (!x) return;
    for (unsigned i = 0; i < 2; ++i) free(x->slots[i].lease.data);
    pthread_cond_destroy(&x->changed);
    pthread_mutex_destroy(&x->mutex);
    free(x);
}
