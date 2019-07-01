#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "hw/register.h"
#include "qemu/bitops.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "qemu/main-loop.h"
#include "hw/fdt_generic_util.h"
#include "hw/arm/arm-system-counter.h"

#ifndef HPSC_ELAPSED_TIMER_ERR_DEBUG
#define HPSC_ELAPSED_TIMER_ERR_DEBUG 0
#endif

#define DB_PRINT_L(lvl, fmt, args...) do {\
    if (HPSC_ELAPSED_TIMER_ERR_DEBUG >= lvl) {\
        qemu_log(TYPE_HPSC_ELAPSED_TIMER ": %s:" fmt, __func__, ## args);\
    } \
} while (0);

#define DB_PRINT(fmt, args...) DB_PRINT_L(1, fmt, ## args)

#define TYPE_HPSC_ELAPSED_TIMER         "hpsc-elapsed-timer"
#define TYPE_HPSC_ELAPSED_TIMER_EVENT   "hpsc-elapsed-timer-event"

#define HPSC_ELAPSED_TIMER(obj) \
     OBJECT_CHECK(HPSCElapsedTimer, (obj), TYPE_HPSC_ELAPSED_TIMER)

#define HPSC_ELAPSED_TIMER_EVENT(obj) \
     OBJECT_CHECK(HPSCElapsedTimerEvent, (obj), TYPE_HPSC_ELAPSED_TIMER_EVENT)

/* This timer's counter register is in fixed units, regardless of
 * the clock frequency, which means that on each clock cycle the
 * timer may increment by a delta of more than 1. This macro
 * specifies the fixed units.
 *
 * NOTE: This macro is not a configurable parameter, it is a fixed
 * characteristic of the backend used to implement this System Counter model.
 * To change the tick rate of this System Counter (and thus ARM Generic
 * Timers) change the clk-freq-hz object property, not this macro. */
#define NOMINAL_FREQ_HZ 1000000000 /* 1 GHz for fixed units of ns */

// TODO: is there support for 64-bit registers? to avoid HI,LO

REG32(REG_CAPTURED_LO,      0x00)
REG32(REG_CAPTURED_HI,      0x04)
REG32(REG_LOAD_LO,          0x08)
REG32(REG_LOAD_HI,          0x0c)
REG32(REG_EVENT_LO,         0x10)
REG32(REG_EVENT_HI,         0x14)
REG32(REG_SYNC_LO,          0x18)
REG32(REG_SYNC_HI,          0x1c)
REG32(REG_PAUSE,            0x20)
REG32(REG_SYNC_INTERVAL,    0x24)
REG32(REG_STATUS,           0x28)
    FIELD(REG_STATUS, EVENT,       0, 1)
    FIELD(REG_STATUS, SYNC,        1, 1)
    FIELD(REG_STATUS, PAUSE,       2, 1)
    FIELD(REG_STATUS, PULSE_OUT,   3, 1)
    FIELD(REG_STATUS, SYNC_PERIOD, 4, 1)
REG32(REG_SYNC_PERIOD,      0x2c) // 32-bit field of STATUS reg in spec
REG32(REG_CONFIG_LO,        0x30)
    FIELD(REG_CONFIG_LO, SYNC_SOURCE, 0,  3) // Sync Select in spec
    FIELD(REG_CONFIG_LO, TICKDIV,     3, 16)
REG32(REG_CONFIG_HI,        0x34)
REG32(REG_PULSE_THRES,      0x38) // 32-bit filed of Pulse Config reg in spec
REG32(REG_PULSE_CONFIG,     0x3c)
    FIELD(REG_PULSE_CONFIG, PULSE_WIDTH, 0, 16)

REG32(REG_CMD_ARM,          0x40)
REG32(REG_CMD_FIRE,         0x44)

#define R_MAX (R_REG_CMD_FIRE + 1)

/* Align memory-mapped IO region to page size */
#define IOMEM_PAGE_SIZE 0x400
#define IOMEM_REGION_SIZE   (((R_MAX * 4) / IOMEM_PAGE_SIZE + 1) * IOMEM_PAGE_SIZE)

#define PTIMER_MODE_ONE_SHOT 1
#define PTIMER_MODE_CONT     0

typedef enum {
    CMD_INVALID = 0,
    CMD_CAPTURE,
    CMD_LOAD,
    CMD_EVENT,
    CMD_SYNC,
} cmd_t;

// TODO: Codes from spec, once spec has them
#define CMD_CAPTURE_ARM    0xCD01
#define CMD_LOAD_ARM       0xCD02
#define CMD_EVENT_ARM      0xCD03
#define CMD_SYNC_ARM       0xCD04

#define CMD_CAPTURE_FIRE   0x01CD
#define CMD_LOAD_FIRE      0x02CD
#define CMD_EVENT_FIRE     0x03CD
#define CMD_SYNC_FIRE      0x04CD

typedef struct HPSCElapsedTimerEvent {
    ARMSystemCounterEvent parent_obj;

    QLIST_ENTRY(HPSCElapsedTimerEvent) list_entry;

    QEMUTimer qtimer;
    bool scheduled;
    ARMSystemCounterEventCb *cb;
    void *arg;
} HPSCElapsedTimerEvent;


typedef struct HPSCElapsedTimer {
    SysBusDevice parent_obj;
    MemoryRegion iomem;

    uint32_t clk_freq_hz;
    uint32_t freq_hz; /* after divider applied */
    uint32_t max_tickdiv;
    uint64_t max_count;
    unsigned max_delta;
    unsigned delta;
    int64_t offset; /* implements setting the counter */

    QLIST_HEAD(se_list_head, HPSCElapsedTimerEvent) slave_events;
    HPSCElapsedTimerEvent event; /* note: also in slave_events list */
    
    qemu_irq irq;

    uint32_t regs[R_MAX];
    RegisterInfo regs_info[R_MAX];
} HPSCElapsedTimer;


static uint64_t get_count(HPSCElapsedTimer *s)
{
    uint64_t count_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    uint64_t count = (count_ns - count_ns % s->delta) + s->offset;
    DB_PRINT("%s: count -> %lx offset -> %lx (%ld)\n",
            object_get_canonical_path(OBJECT(s)), count, s->offset, s->offset);
    return count;
}

static void set_count(HPSCElapsedTimer *s, uint64_t count)
{
    /* Note, here we need the truncated count but without the offset. */
    uint64_t cur_count_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    s->offset = count - (cur_count_ns - cur_count_ns % s->delta);
    DB_PRINT("%s: count <- %lx offset <- %lx (%ld)\n",
            object_get_canonical_path(OBJECT(s)), count, s->offset, s->offset);
}

static void update_freq(HPSCElapsedTimer *s)
{
    uint32_t tickdiv = extract32(s->regs[R_REG_CONFIG_LO],
            R_REG_CONFIG_LO_TICKDIV_SHIFT, R_REG_CONFIG_LO_TICKDIV_LENGTH) + 1;
    unsigned cur_delta = s->delta;
    uint64_t cur_remaining, remaining;
    bool scheduled;

    s->freq_hz = s->clk_freq_hz / tickdiv;
    s->delta = NOMINAL_FREQ_HZ / s->freq_hz;

    DB_PRINT("%s: update freq <- %u (tickdiv <- %u tick delta (%u) <- %u\n",
            object_get_canonical_path(OBJECT(s)),
            s->freq_hz, tickdiv, cur_delta, s->delta);

    /* If frequency didn't change, then events don't need scaling. */
    if (s->delta == cur_delta)
        return;

    HPSCElapsedTimerEvent *he;
    QLIST_FOREACH(he, &s->slave_events, list_entry) {
        scheduled = he->scheduled;
        if (scheduled) {
            /* The timer.h interface does not support changing the scale
             * of a timer, so we reinit and re-scale the remaining time. */
            cur_remaining = timer_expire_time(&he->qtimer) * cur_delta;
            remaining = cur_remaining / s->delta;
            DB_PRINT("%s: update freq: event: mod: remaining %lu -> %lu\n",
                     object_get_canonical_path(OBJECT(s)),
                     cur_remaining, remaining);
            timer_del(&he->qtimer);
        }

        timer_init(&he->qtimer, QEMU_CLOCK_VIRTUAL, s->delta, he->cb, he->arg);

        if (scheduled) {
            timer_mod(&he->qtimer, remaining);
        }
    }
}

static void synchronize(HPSCElapsedTimer *s)
{
    DB_PRINT("%s: synchronize\n",
             object_get_canonical_path(OBJECT(s)));

    uint64_t count = get_count(s);
    s->regs[R_REG_SYNC_HI] = count >> 32;
    s->regs[R_REG_SYNC_LO] = (uint32_t)count;

    uint32_t sync_interval = s->regs[R_REG_SYNC_INTERVAL];
    assert(sync_interval > 0); // TODO: hw spec: default?
    count = (count / sync_interval + 1) * sync_interval;
    set_count(s, count);
}

static void event_init(HPSCElapsedTimerEvent *he, HPSCElapsedTimer *s,
                       ARMSystemCounterEventCb *cb, void *arg)
{
    he->scheduled = false;
    he->cb = cb;
    he->arg = arg;
    /* Timer is scaled by delta, because our maximum resolution is bounded by
     * the clk freq (i.e. the delta). We don't have to scale it, though, could
     * also keep it at same freq as QEMU_CLOCK_VIRTUAL. */
    timer_init(&he->qtimer, QEMU_CLOCK_VIRTUAL, s->delta, cb, arg);
    DB_PRINT("%s: event create: delta %u exp %lu\n",
             object_get_canonical_path(OBJECT(s)), s->delta,
             timer_expire_time(&he->qtimer));
    QLIST_INSERT_HEAD(&s->slave_events, he, list_entry);
}

static void event_deinit(HPSCElapsedTimerEvent *he)
{
    QLIST_REMOVE(he, list_entry);
    timer_deinit(&he->qtimer);
}

static void event_schedule(HPSCElapsedTimerEvent *he, HPSCElapsedTimer *s, uint64_t time)
{
    he->scheduled = true;
    /* Scale the time given (in units exported by this timer interface, i.e.
     * ns) by delta, because this internal event backend timer is scaled. */
    uint64_t event_time = (time - s->offset) / s->delta;
    DB_PRINT("%s: event sched: time %lx offset %ld "
             "(time - offset) %lx event_time %lx\n",
             object_get_canonical_path(OBJECT(s)), time, s->offset,
             time - s->offset, event_time);
    timer_mod(&he->qtimer, event_time);
}

static void event_cancel(HPSCElapsedTimerEvent *he)
{
    timer_del(&he->qtimer);
    he->scheduled = false;
}

static void execute_cmd(HPSCElapsedTimer *s, cmd_t cmd)
{
    uint64_t count, event_time;
    switch (cmd) {
        case CMD_CAPTURE:
                count = get_count(s);
                DB_PRINT("%s: cmd: capture: count -> %lx\n",
                         object_get_canonical_path(OBJECT(s)), count);
                s->regs[R_REG_CAPTURED_LO] = (uint32_t)(count & 0xffffffff);
                s->regs[R_REG_CAPTURED_HI] = (uint32_t)(count >> 32);
                break;
        case CMD_LOAD:
                count = ((uint64_t)s->regs[R_REG_LOAD_HI] << 32) | s->regs[R_REG_LOAD_LO];
                DB_PRINT("%s: cmd: load: count <- %lx\n",
                         object_get_canonical_path(OBJECT(s)), count);
                set_count(s, count);
                break;
        case CMD_EVENT:
                event_time = ((uint64_t)s->regs[R_REG_EVENT_HI] << 32) |
                             s->regs[R_REG_EVENT_LO];
                DB_PRINT("%s: cmd: sched event @ %lx ticks\n",
                         object_get_canonical_path(OBJECT(s)), event_time);
                event_schedule(&s->event, s, event_time);
                break;
        case CMD_SYNC:
                synchronize(s);
                break;
        default:
                assert(false && "unhandled cmd");
    }
}

static void post_write_cmd_fire(RegisterInfo *reg, uint64_t val64)
{
    HPSCElapsedTimer *s = HPSC_ELAPSED_TIMER(reg->opaque);
    cmd_t cmd = CMD_INVALID;
    uint32_t arm_code = s->regs[R_REG_CMD_ARM];
    DB_PRINT("%s: arm code: %x\n",
             object_get_canonical_path(OBJECT(s)), arm_code);
    switch (val64) {
        case CMD_CAPTURE_FIRE: if (arm_code == CMD_CAPTURE_ARM) cmd = CMD_CAPTURE; break;
        case CMD_LOAD_FIRE:    if (arm_code == CMD_LOAD_ARM)    cmd = CMD_LOAD;    break;
        case CMD_EVENT_FIRE:   if (arm_code == CMD_EVENT_ARM)   cmd = CMD_EVENT;   break;
        case CMD_SYNC_FIRE:    if (arm_code == CMD_SYNC_ARM)    cmd = CMD_SYNC;    break;
        default:
            qemu_log_mask(LOG_GUEST_ERROR,
                    "%s: unrecognized cmd fire code: %" PRIx64 "\n",
                     object_get_canonical_path(OBJECT(s)), val64);
    }

    if (cmd != CMD_INVALID) {
            DB_PRINT("%s: cmd fired: %u\n",
                     object_get_canonical_path(OBJECT(s)), cmd);
            execute_cmd(s, cmd);
    } else {
        qemu_log_mask(LOG_GUEST_ERROR,
                "%s: mismatched fire code %" PRIx64 " for arm code %u\n",
                object_get_canonical_path(OBJECT(s)), val64, arm_code);
    }
}

static void post_write_config_lo(RegisterInfo *reg, uint64_t val64)
{
    HPSCElapsedTimer *s = HPSC_ELAPSED_TIMER(reg->opaque);
    update_freq(s); // TODO: is this update indeed a NOP if value unchanged?
}

static void post_write_status(RegisterInfo *reg, uint64_t val64)
{
    HPSCElapsedTimer *s = HPSC_ELAPSED_TIMER(reg->opaque);
    uint32_t status = s->regs[R_REG_STATUS];

    if (status & R_REG_STATUS_EVENT_MASK) {
        qemu_set_irq(s->irq, 0); // TODO: spec: how is interrupt cleared?
        /* TODO: hw spec: should the event be left enabled, i.e. should it
         * trigger after main timer counter rolls over? */
    }

    if (status & R_REG_STATUS_SYNC_MASK)
        s->regs[R_REG_SYNC_HI] = s->regs[R_REG_SYNC_LO] = 0;

    if (status & R_REG_STATUS_PAUSE_MASK)
        s->regs[R_REG_PAUSE] = 0;

    if (status & R_REG_STATUS_PULSE_OUT_MASK) {
        // TODO: spec: is this supposed to enable/disable the pulse output?
        // Spec says this is W1C, though.
    }

    if (status & R_REG_STATUS_SYNC_PERIOD_MASK) {
        // TODO: spec: what is this supposed to do?
    }
}

static RegisterAccessInfo hpsc_elapsed_regs_info[] = {
    { .name = "REG_CAPTURED_LO",
        .addr = A_REG_CAPTURED_LO,
        .reset = 0x0,
        .ro = ~0,
    },{ .name = "REG_CAPTURED_HI",
        .addr = A_REG_CAPTURED_HI,
        .reset = 0x0,
        .ro = ~0,
    },{ .name = "REG_LOAD_LO",
        .addr = A_REG_LOAD_LO,
        .reset = 0x0,
    },{ .name = "REG_LOAD_HI",
        .addr = A_REG_LOAD_HI,
        .reset = 0x0,
    },{ .name = "REG_EVENT_LO",
        .addr = A_REG_EVENT_LO,
        .reset = 0x0,
    },{ .name = "REG_EVENT_HI",
        .addr = A_REG_EVENT_HI,
        .reset = 0x0,
    },{ .name = "REG_SYNC_LO",
        .addr = A_REG_SYNC_LO,
        .reset = 0x0,
        .ro = ~0,
    },{ .name = "REG_SYNC_HI",
        .addr = A_REG_SYNC_HI,
        .reset = 0x0,
        .ro = ~0,
    },{ .name = "REG_PAUSE",
        .addr = A_REG_PAUSE,
        .reset = 0x0,
        .ro = ~0,
    },{ .name = "REG_SYNC_INTERVAL",
        .addr = A_REG_SYNC_INTERVAL,
        .reset = 0x0,
    },{ .name = "REG_CONFIG_LO",
        .addr = A_REG_CONFIG_LO,
        .reset = 0x0,
        .rsvd = ~(R_REG_CONFIG_LO_SYNC_SOURCE_MASK | R_REG_CONFIG_LO_TICKDIV_MASK),
        .post_write = post_write_config_lo,
    },{ .name = "REG_CONFIG_HI",
        .addr = A_REG_CONFIG_HI,
        .rsvd = ~0,
    },{ .name = "REG_STATUS",
        .addr = A_REG_STATUS,
        .reset = 0x0,
        .rsvd = ~(R_REG_STATUS_EVENT_MASK |
                  R_REG_STATUS_SYNC_MASK |
                  R_REG_STATUS_PAUSE_MASK |
                  R_REG_STATUS_PULSE_OUT_MASK |
                  R_REG_STATUS_SYNC_PERIOD_MASK),
        .post_write = post_write_status,
    },{ .name = "REG_SYNC_PERIOD",
        .addr = A_REG_SYNC_PERIOD,
        .reset = 0x0,
        .ro = ~0,
    },{ .name = "REG_PULSE_THRES",
        .addr = A_REG_PULSE_THRES,
        .reset = 0x0,
    },{ .name = "REG_PULSE_CONFIG",
        .addr = A_REG_PULSE_CONFIG,
        .reset = 0x0,
        .rsvd = ~(R_REG_PULSE_CONFIG_PULSE_WIDTH_MASK),
    },{ .name = "REG_CMD_ARM",
        .addr = A_REG_CMD_ARM,
        .reset = 0x0,
    },{ .name = "REG_CMD_FIRE",
        .addr = A_REG_CMD_FIRE,
        .reset = 0x0,
        .post_write = post_write_cmd_fire,
   }
};

static uint64_t hpsc_elapsed_read(void *opaque, hwaddr addr, unsigned size)
{
    HPSCElapsedTimer *s = HPSC_ELAPSED_TIMER(opaque);
    RegisterInfo *r = &s->regs_info[addr / 4];

    if (!r->data) {
        qemu_log_mask(LOG_GUEST_ERROR,
                "%s: Decode error: read from %" HWADDR_PRIx "\n",
                object_get_canonical_path(OBJECT(s)), addr);
        return 0;
    }
    return register_read(r, ~0, NULL, false);
}

static void hpsc_elapsed_write(void *opaque, hwaddr addr, uint64_t value,
                      unsigned size)
{
    HPSCElapsedTimer *s = HPSC_ELAPSED_TIMER(opaque);
    RegisterInfo *r = &s->regs_info[addr / 4];

    if (!r->data) {
        qemu_log_mask(LOG_GUEST_ERROR,
                "%s: Decode error: write to %" HWADDR_PRIx "=%" PRIx64 "\n",
                object_get_canonical_path(OBJECT(s)), addr, value);
        return;
    }
    register_write(r, value, ~0, NULL, false);
}

static void handle_event(void *opaque)
{
    HPSCElapsedTimer *s = HPSC_ELAPSED_TIMER(opaque);
    DB_PRINT("%s: event triggered\n", object_get_canonical_path(OBJECT(s)));

    s->regs[R_REG_STATUS] |= R_REG_STATUS_EVENT_MASK;
    qemu_set_irq(s->irq, 1);
}

static uint64_t hpsc_elapsed_timer_max_count(ARMSystemCounter *asc)
{
    HPSCElapsedTimer *s = HPSC_ELAPSED_TIMER(asc);
    return s->max_count;
}

static unsigned hpsc_elapsed_timer_max_delta(ARMSystemCounter *asc)
{
    HPSCElapsedTimer *s = HPSC_ELAPSED_TIMER(asc);
    return s->max_delta;
}

static uint64_t hpsc_elapsed_timer_count(ARMSystemCounter *asc)
{
    HPSCElapsedTimer *s = HPSC_ELAPSED_TIMER(asc);
    return get_count(s);
}

static ARMSystemCounterEvent *hpsc_elapsed_timer_event_create(
                                        ARMSystemCounter *asc,
                                        ARMSystemCounterEventCb *cb, void *arg)
{
    HPSCElapsedTimer *s = HPSC_ELAPSED_TIMER(asc);
    HPSCElapsedTimerEvent *he =
        HPSC_ELAPSED_TIMER_EVENT(object_new(TYPE_HPSC_ELAPSED_TIMER_EVENT));
    ARMSystemCounterEvent *e = ARM_SYSTEM_COUNTER_EVENT(he);
    e->sc = asc;
    event_init(he, s, cb, arg);
    return e;
}

static void hpsc_elapsed_timer_event_destroy(ARMSystemCounterEvent *e)
{
    HPSCElapsedTimerEvent *he = HPSC_ELAPSED_TIMER_EVENT(e);
    e->sc = NULL;
    event_deinit(he);
    object_unref(OBJECT(he));
}

static void hpsc_elapsed_timer_event_schedule(ARMSystemCounterEvent *e,
                                              uint64_t time)
{
    HPSCElapsedTimerEvent *he = HPSC_ELAPSED_TIMER_EVENT(e);
    HPSCElapsedTimer *s = HPSC_ELAPSED_TIMER(e->sc);
    DB_PRINT("%s: slave event sched @ %lx\n",
             object_get_canonical_path(OBJECT(s)), time);
    event_schedule(he, s, time);
}

static void hpsc_elapsed_timer_event_cancel(ARMSystemCounterEvent *asc_e)
{
    HPSCElapsedTimerEvent *he = HPSC_ELAPSED_TIMER_EVENT(asc_e);
    DB_PRINT("%s: slave event cancel\n",
             object_get_canonical_path(OBJECT(asc_e->sc)));
    event_cancel(he);
}

static void hpsc_elapsed_reset(DeviceState *dev)
{
    HPSCElapsedTimer *s = HPSC_ELAPSED_TIMER(dev);
    unsigned int i;

    for (i = 0; i < ARRAY_SIZE(s->regs_info); ++i)
        register_reset(&s->regs_info[i]);

    s->offset = 0;

    /* The event register was reset, so stop the interval event timer.
     *
     * TODO: hw spec: is the event always enabled? That is, in the abscence of
     * CMD_EVENT, should the event should trigger for the first time when the
     * timer rolls over? If yes, then we should schedule the event here, but
     * the we can't schedule it for 0 because then it would trigger immediately,
     * so we could schedule it for max_count, which would be off by one cycle.
     */
    event_cancel(&s->event);

    /* To external slave events need the re-scaling due to frequency change,
     * but they should be kept running, because the comparators are logically
     * owned by the comsumer of our API, which may be in a different reset
     * domain. */
    update_freq(s);

    qemu_set_irq(s->irq, 0);
}

static void hpsc_elapsed_realize(DeviceState *dev, Error **errp)
{
    HPSCElapsedTimer *s = HPSC_ELAPSED_TIMER(dev);
    unsigned int i;

    // There range of the Qemu timer.h backend is a signed 64-bit integer.
    s->max_delta = NOMINAL_FREQ_HZ / s->clk_freq_hz;
    s->delta = s->max_delta;
    s->max_count = INT64_MAX / s->max_delta;

    qemu_log(TYPE_HPSC_ELAPSED_TIMER
             "%s: init: max_tickdiv %u max_count 0x%lx max_delta %u\n",
             object_get_canonical_path(OBJECT(s)),
             s->max_tickdiv, s->max_count, s->max_delta);

    event_init(&s->event, s, handle_event, s);

    QLIST_INIT(&s->slave_events);

    for (i = 0; i < ARRAY_SIZE(hpsc_elapsed_regs_info); ++i) {
        RegisterInfo *r =
                    &s->regs_info[hpsc_elapsed_regs_info[i].addr / 4];

        *r = (RegisterInfo) {
            .data = (uint8_t *)&s->regs[
                    hpsc_elapsed_regs_info[i].addr/4],
            .data_size = sizeof(uint32_t),
            .access = &hpsc_elapsed_regs_info[i],
            .opaque = s,
        };
        register_init(r);
        qdev_pass_all_gpios(DEVICE(r), dev);
    }
}

static const MemoryRegionOps hpsc_elapsed_ops = {
    .read = hpsc_elapsed_read,
    .write = hpsc_elapsed_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void hpsc_elapsed_timer_init(Object *obj)
{
    HPSCElapsedTimer *s = HPSC_ELAPSED_TIMER(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &hpsc_elapsed_ops, s,
                          TYPE_HPSC_ELAPSED_TIMER, IOMEM_REGION_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);

    sysbus_init_irq(SYS_BUS_DEVICE(s), &s->irq);
}

static const VMStateDescription vmstate_hpsc_elapsed = {
    .name = TYPE_HPSC_ELAPSED_TIMER,
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, HPSCElapsedTimer, R_MAX),
        VMSTATE_END_OF_LIST(),
    }
};

static Property hpsc_elapsed_props[] = {
    DEFINE_PROP_UINT32("clk-freq-hz",  HPSCElapsedTimer,    clk_freq_hz,      125000000),
    DEFINE_PROP_UINT32("max-divider", HPSCElapsedTimer,     max_tickdiv, 32), // f_min=3906250 Hz in spec
    DEFINE_PROP_END_OF_LIST()
};

static void hpsc_elapsed_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ARMSystemCounterClass *sc = ARM_SYSTEM_COUNTER_CLASS(klass);

    dc->reset = hpsc_elapsed_reset;
    dc->realize = hpsc_elapsed_realize;
    dc->vmsd = &vmstate_hpsc_elapsed;
    dc->props = hpsc_elapsed_props;

    sc->max_count = hpsc_elapsed_timer_max_count;
    sc->max_delta = hpsc_elapsed_timer_max_delta;
    sc->count = hpsc_elapsed_timer_count;
    sc->event_create = hpsc_elapsed_timer_event_create;
    sc->event_destroy = hpsc_elapsed_timer_event_destroy;
    sc->event_schedule = hpsc_elapsed_timer_event_schedule;
    sc->event_cancel = hpsc_elapsed_timer_event_cancel;
}

static const TypeInfo hpsc_elapsed_info[] = {
    {
        .name          = TYPE_HPSC_ELAPSED_TIMER,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(HPSCElapsedTimer),
        .class_init    = hpsc_elapsed_class_init,
        .instance_init = hpsc_elapsed_timer_init,
        .interfaces = (InterfaceInfo []) {
            {TYPE_ARM_SYSTEM_COUNTER},
        },
    },
    {
        .name          = TYPE_HPSC_ELAPSED_TIMER_EVENT,
        .parent        = TYPE_ARM_SYSTEM_COUNTER_EVENT,
        .instance_size = sizeof(HPSCElapsedTimerEvent),
    },
};

DEFINE_TYPES(hpsc_elapsed_info)
