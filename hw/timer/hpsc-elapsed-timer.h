#ifndef HPSC_ELAPSED_TIMER_H
#define HPSC_ELAPSED_TIMER_H

#include <stdint.h>

#define TYPE_HPSC_ELAPSED_TIMER "hpsc,hpsc-elapsed-timer"

struct HPSCElapsedTimer;
struct HPSCElapsedTimerEvent;

typedef void (hpsc_elapsed_timer_event_cb)(void *arg);

uint64_t hpsc_elapsed_timer_get_count(struct HPSCElapsedTimer *s);

struct HPSCElapsedTimerEvent *
hpsc_elapsed_timer_event_create(struct HPSCElapsedTimer *s,
                                hpsc_elapsed_timer_event_cb *cb, void *cb_arg);
void hpsc_elapsed_timer_event_destroy(struct HPSCElapsedTimerEvent *e);
void hpsc_elapsed_timer_event_schedule(struct HPSCElapsedTimerEvent *e, uint64_t time);

#endif // HPSC_ELAPSED_TIMER_H
