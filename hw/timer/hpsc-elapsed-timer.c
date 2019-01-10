#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "hw/register-dep.h"
#include "qemu/bitops.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "qemu/main-loop.h"
#include "hw/ptimer.h"
#include "hw/fdt_generic_util.h"

#include "hpsc-elapsed-timer.h"

#ifndef HPSC_ELAPSED_TIMER_ERR_DEBUG
#define HPSC_ELAPSED_TIMER_ERR_DEBUG 0
#endif

#define DB_PRINT_L(lvl, fmt, args...) do {\
    if (HPSC_ELAPSED_TIMER_ERR_DEBUG >= lvl) {\
        qemu_log(TYPE_HPSC_ELAPSED_TIMER ": %s:" fmt, __func__, ## args);\
    } \
} while (0);

#define DB_PRINT(fmt, args...) DB_PRINT_L(1, fmt, ## args)

#define HPSC_ELAPSED_TIMER(obj) \
     OBJECT_CHECK(HPSCElapsedTimer, (obj), TYPE_HPSC_ELAPSED_TIMER)

// TODO: is there support for 64-bit registers? to avoid HI,LO

DEP_REG32(REG_CAPTURED_LO,      0x00)
DEP_REG32(REG_CAPTURED_HI,      0x04)
DEP_REG32(REG_LOAD_LO,          0x08)
DEP_REG32(REG_LOAD_HI,          0x0c)
DEP_REG32(REG_EVENT_LO,         0x10)
DEP_REG32(REG_EVENT_HI,         0x14)
DEP_REG32(REG_SYNC_LO,          0x18)
DEP_REG32(REG_SYNC_HI,          0x1c)
DEP_REG32(REG_PAUSE,            0x20)
DEP_REG32(REG_SYNC_INTERVAL,    0x24)
DEP_REG32(REG_STATUS,           0x28)
    DEP_FIELD(REG_STATUS, EVENT,       1, 0)
    DEP_FIELD(REG_STATUS, SYNC,        1, 1)
    DEP_FIELD(REG_STATUS, PAUSE,       1, 2)
    DEP_FIELD(REG_STATUS, PULSE_OUT,   1, 3)
    DEP_FIELD(REG_STATUS, SYNC_PERIOD, 1, 4)
DEP_REG32(REG_SYNC_PERIOD,      0x2c) // 32-bit field of STATUS reg in spec
DEP_REG32(REG_CONFIG_LO,        0x30)
    DEP_FIELD(REG_CONFIG_LO, SYNC_SOURCE,  3, 0) // Sync Select in spec
    DEP_FIELD(REG_CONFIG_LO, TICKDIV,     16, 3)
DEP_REG32(REG_CONFIG_HI,        0x34)
DEP_REG32(REG_PULSE_THRES,      0x38) // 32-bit filed of Pulse Config reg in spec
DEP_REG32(REG_PULSE_CONFIG,     0x3c)
    DEP_FIELD(REG_PULSE_CONFIG, PULSE_WIDTH, 16, 0)

DEP_REG32(REG_CMD_ARM,          0x40)
DEP_REG32(REG_CMD_FIRE,         0x44)

#define R_MAX (R_REG_CMD_FIRE + 1)

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

struct HPSCElapsedTimerEvent;

typedef struct HPSCElapsedTimer {
    SysBusDevice parent_obj;
    MemoryRegion iomem;

    // const props from DT
    uint32_t nominal_freq_hz; // determines count reg units regardless of clk
    uint32_t clk_freq_hz;
    uint32_t max_tickdiv;
    uint64_t max_count; // a derived prop

    uint32_t freq_hz; // after divider applied

    // Main timer
    ptimer_state *ptimer;
    QEMUBH *bh;

    // Timer used to schedule the event interrupt
    ptimer_state *ptimer_event;
    QEMUBH *bh_event;

    QLIST_HEAD(se_list_head, HPSCElapsedTimerEvent) slave_events;

    uint64_t limit;
    unsigned tick_delta;
    
    qemu_irq irq;

    uint32_t regs[R_MAX];
    DepRegisterInfo regs_info[R_MAX];
} HPSCElapsedTimer;

// Slave timers clocked by the Elapsed Timer can schedule events
typedef struct HPSCElapsedTimerEvent {
    QLIST_ENTRY(HPSCElapsedTimerEvent) list_entry;

    HPSCElapsedTimer *etimer;
    ptimer_state *ptimer;
    QEMUBH *bh;
} HPSCElapsedTimerEvent;


// Convert from units of time (given by the "nominal" frequency) to ptimer units
static uint64_t time_to_ticks(HPSCElapsedTimer *s, uint64_t time)
{
    return time / s->tick_delta;
}
static uint64_t get_count(HPSCElapsedTimer *s)
{
    uint64_t count = ptimer_get_count(s->ptimer);
    DB_PRINT("%s: count -> %lx (face count %lx)\n",
            object_get_canonical_path(OBJECT(s)),
            count, (s->limit - count) * s->tick_delta);
    return (s->limit - count) * s->tick_delta;
}

static void set_count(HPSCElapsedTimer *s, uint64_t count)
{
    DB_PRINT("%s: count <- %lx (face count %lx)\n",
            object_get_canonical_path(OBJECT(s)),
            s->limit - count / s->tick_delta, count);
    ptimer_set_count(s->ptimer, s->limit - count / s->tick_delta);
}

static void timer_update_freq(HPSCElapsedTimer *s)
{
    uint32_t tickdiv = extract32(s->regs[R_REG_CONFIG_LO],
            R_REG_CONFIG_LO_TICKDIV_SHIFT, R_REG_CONFIG_LO_TICKDIV_LENGTH) + 1;
    s->freq_hz = s->clk_freq_hz / tickdiv;
    s->tick_delta = s->nominal_freq_hz / s->freq_hz;

    // since user-facing count is internal count * delta, we limit the max
    // internal count so that that product does not overflow
    s->limit = s->max_count / s->tick_delta;

    DB_PRINT("%s: update freq <- %u (tickdiv %u tick delta %u max count %lx)\n",
            object_get_canonical_path(OBJECT(s)), s->freq_hz, tickdiv, s->tick_delta, s->limit);

    ptimer_set_freq(s->ptimer, s->freq_hz);
    ptimer_set_freq(s->ptimer_event, s->freq_hz);

    HPSCElapsedTimerEvent *e;
    QLIST_FOREACH(e, &s->slave_events, list_entry) {
        ptimer_set_freq(e->ptimer, s->freq_hz);
    }

    ptimer_set_limit(s->ptimer, s->limit, /* reload? */ 1);

    // Since event is an absolute time, once it's hit, the next hit will be
    // after a rollover and counting to to the same event value.
    ptimer_set_limit(s->ptimer_event, s->limit, /* reload? */ 1);
}

static void timer_sync(HPSCElapsedTimer *s)
{
    DB_PRINT("%s: sync\n",
             object_get_canonical_path(OBJECT(s)));

    uint64_t count = get_count(s);
    s->regs[R_REG_SYNC_HI] = count >> 32;
    s->regs[R_REG_SYNC_LO] = (uint32_t)count;

    uint32_t sync_interval = s->regs[R_REG_SYNC_INTERVAL];
    count = (count / sync_interval + 1) * sync_interval;
    set_count(s, count);
}

static void execute_cmd(HPSCElapsedTimer *s, cmd_t cmd)
{
    uint64_t count, event, remaining, remaining_ticks;
    switch (cmd) {
        case CMD_CAPTURE:
                count = get_count(s);
                s->regs[R_REG_CAPTURED_LO] = (uint32_t)(count & 0xffffffff);
                s->regs[R_REG_CAPTURED_HI] = (uint32_t)(count >> 32);
                DB_PRINT("%s: cmd: capture: count -> %lx\n",
                         object_get_canonical_path(OBJECT(s)), count);
                break;
        case CMD_LOAD:
                count = ((uint64_t)s->regs[R_REG_LOAD_HI] << 32) | s->regs[R_REG_LOAD_LO];
                set_count(s, count);
                DB_PRINT("%s: cmd: load: count <- %lx\n",
                         object_get_canonical_path(OBJECT(s)), count);
                break;
        case CMD_EVENT:
                ptimer_stop(s->ptimer_event); // in case changing before the event

                count = get_count(s);
                event = ((uint64_t)s->regs[R_REG_EVENT_HI] << 32) | s->regs[R_REG_EVENT_LO];
                if (event > count) { // future
                    remaining = event - count;
                } else { // in the past
                    remaining = count + event;
                }
                remaining_ticks = time_to_ticks(s, remaining);
                ptimer_set_limit(s->ptimer_event, remaining_ticks, /* reload? */ 1);
                ptimer_run(s->ptimer_event, PTIMER_MODE_CONT);

                DB_PRINT("%s: cmd: event timer count <- %lx ticks (%lx time)\n",
                         object_get_canonical_path(OBJECT(s)), remaining_ticks, remaining);
                break;
        case CMD_SYNC:
                timer_sync(s);
                break;
        default:
                assert(false && "unhandled cmd");
    }
}

static void post_write_cmd_fire(DepRegisterInfo *reg, uint64_t val64)
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

static void post_write_config_lo(DepRegisterInfo *reg, uint64_t val64)
{
    HPSCElapsedTimer *s = HPSC_ELAPSED_TIMER(reg->opaque);
    timer_update_freq(s); // TODO: is this update indeed a NOP if value unchanged?
}

static void post_write_status(DepRegisterInfo *reg, uint64_t val64)
{
    HPSCElapsedTimer *s = HPSC_ELAPSED_TIMER(reg->opaque);
    uint32_t status = s->regs[R_REG_STATUS];

    if (status & R_REG_STATUS_EVENT_MASK) {
        qemu_set_irq(s->irq, 0); // TODO: spec: how is interrupt cleared?
        ptimer_stop(s->ptimer_event);
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

static DepRegisterAccessInfo hpsc_elapsed_regs_info[] = {
    { .name = "REG_CAPTURED_LO",
        .decode.addr = A_REG_CAPTURED_LO,
        .reset = 0x0,
        .ro = ~0,
    },{ .name = "REG_CAPTURED_HI",
        .decode.addr = A_REG_CAPTURED_HI,
        .reset = 0x0,
        .ro = ~0,
    },{ .name = "REG_LOAD_LO",
        .decode.addr = A_REG_LOAD_LO,
        .reset = 0x0,
    },{ .name = "REG_LOAD_HI",
        .decode.addr = A_REG_LOAD_HI,
        .reset = 0x0,
    },{ .name = "REG_EVENT_LO",
        .decode.addr = A_REG_EVENT_LO,
        .reset = 0x0,
    },{ .name = "REG_EVENT_HI",
        .decode.addr = A_REG_EVENT_HI,
        .reset = 0x0,
    },{ .name = "REG_SYNC_LO",
        .decode.addr = A_REG_SYNC_LO,
        .reset = 0x0,
        .ro = ~0,
    },{ .name = "REG_SYNC_HI",
        .decode.addr = A_REG_SYNC_HI,
        .reset = 0x0,
        .ro = ~0,
    },{ .name = "REG_PAUSE",
        .decode.addr = A_REG_PAUSE,
        .reset = 0x0,
        .ro = ~0,
    },{ .name = "REG_SYNC_INTERVAL",
        .decode.addr = A_REG_SYNC_INTERVAL,
        .reset = 0x0,
    },{ .name = "REG_CONFIG_LO",
        .decode.addr = A_REG_CONFIG_LO,
        .reset = 0x0,
        .rsvd = ~(R_REG_CONFIG_LO_SYNC_SOURCE_MASK | R_REG_CONFIG_LO_TICKDIV_MASK),
        .post_write = post_write_config_lo,
    },{ .name = "REG_CONFIG_HI",
        .decode.addr = A_REG_CONFIG_HI,
        .rsvd = ~0,
    },{ .name = "REG_STATUS",
        .decode.addr = A_REG_STATUS,
        .reset = 0x0,
        .rsvd = ~(R_REG_STATUS_EVENT_MASK |
                  R_REG_STATUS_SYNC_MASK |
                  R_REG_STATUS_PAUSE_MASK |
                  R_REG_STATUS_PULSE_OUT_MASK |
                  R_REG_STATUS_SYNC_PERIOD_MASK),
        .post_write = post_write_status,
    },{ .name = "REG_SYNC_PERIOD",
        .decode.addr = A_REG_SYNC_PERIOD,
        .reset = 0x0,
        .ro = ~0,
    },{ .name = "REG_PULSE_THRES",
        .decode.addr = A_REG_PULSE_THRES,
        .reset = 0x0,
    },{ .name = "REG_PULSE_CONFIG",
        .decode.addr = A_REG_PULSE_CONFIG,
        .reset = 0x0,
        .rsvd = ~(R_REG_PULSE_CONFIG_PULSE_WIDTH_MASK),
    },{ .name = "REG_CMD_ARM",
        .decode.addr = A_REG_CMD_ARM,
        .reset = 0x0,
    },{ .name = "REG_CMD_FIRE",
        .decode.addr = A_REG_CMD_FIRE,
        .reset = 0x0,
        .post_write = post_write_cmd_fire,
   }
};

static void hpsc_elapsed_reset(DeviceState *dev)
{
    HPSCElapsedTimer *s = HPSC_ELAPSED_TIMER(dev);
    unsigned int i;

    // TODO: Workaround for Qemu calling reset more than once (second time via busj
    // reset). The true fix is to fix Qemu to reset only once.
    ptimer_stop(s->ptimer);
    ptimer_stop(s->ptimer_event);

    for (i = 0; i < ARRAY_SIZE(s->regs_info); ++i)
        dep_register_reset(&s->regs_info[i]);

    timer_update_freq(s);

    DB_PRINT("%s: reset: count <- %lx\n",
            object_get_canonical_path(OBJECT(s)), s->limit);
    ptimer_set_count(s->ptimer, s->limit);
    ptimer_run(s->ptimer, PTIMER_MODE_CONT);
    qemu_set_irq(s->irq, 0);
}

static uint64_t hpsc_elapsed_read(void *opaque, hwaddr addr, unsigned size)
{
    HPSCElapsedTimer *s = HPSC_ELAPSED_TIMER(opaque);
    DepRegisterInfo *r = &s->regs_info[addr / 4];

    if (!r->data) {
        qemu_log_mask(LOG_GUEST_ERROR,
                "%s: Decode error: read from %" HWADDR_PRIx "\n",
                object_get_canonical_path(OBJECT(s)), addr);
        return 0;
    }
    return dep_register_read(r);
}

static void hpsc_elapsed_write(void *opaque, hwaddr addr, uint64_t value,
                      unsigned size)
{
    HPSCElapsedTimer *s = HPSC_ELAPSED_TIMER(opaque);
    DepRegisterInfo *r = &s->regs_info[addr / 4];

    if (!r->data) {
        qemu_log_mask(LOG_GUEST_ERROR,
                "%s: Decode error: write to %" HWADDR_PRIx "=%" PRIx64 "\n",
                object_get_canonical_path(OBJECT(s)), addr, value);
        return;
    }
    dep_register_write(r, value, ~0);
}

static void hpsc_elapsed_access(MemoryTransaction *tr)
{
    MemTxAttrs attr = tr->attr;
    void *opaque = tr->opaque;
    hwaddr addr = tr->addr;
    unsigned size = tr->size;
    uint64_t value = tr->data.u64;;
    bool is_write = tr->rw;

    if (!attr.secure) {
        qemu_log_mask(LOG_GUEST_ERROR, "unsecure access to timer denied\n");
        return;
    }

    if (is_write) {
        hpsc_elapsed_write(opaque, addr, value, size);
    } else {
        tr->data.u64 = hpsc_elapsed_read(opaque, addr, size);
    }
}

static void main_timer_rollover(void *opaque)
{
    // Nothing to do, but ptimer requires the bottom half (BH) callback
}

static void event_timer_rollover(void *opaque)
{
    HPSCElapsedTimer *s = opaque;
    DB_PRINT("%s: event rollover\n", object_get_canonical_path(OBJECT(s)));

    s->regs[R_REG_STATUS] |= R_REG_STATUS_EVENT_MASK;
    qemu_set_irq(s->irq, 1);
}

uint64_t hpsc_elapsed_timer_get_count(HPSCElapsedTimer *s)
{
    return get_count(s);
}

struct HPSCElapsedTimerEvent *
hpsc_elapsed_timer_event_create(struct HPSCElapsedTimer *s,
                                hpsc_elapsed_timer_event_cb *cb, void *cb_arg)
{
    HPSCElapsedTimerEvent *e = g_malloc0(sizeof(HPSCElapsedTimerEvent));
    e->etimer = s;
    e->bh = qemu_bh_new(cb, cb_arg);
    e->ptimer = ptimer_init(e->bh, PTIMER_POLICY_DEFAULT);
    QLIST_INSERT_HEAD(&s->slave_events, e, list_entry);
    return e;
}
void hpsc_elapsed_timer_event_destroy(struct HPSCElapsedTimerEvent *e)
{
    assert(e);
    QLIST_REMOVE(e, list_entry);
    ptimer_free(e->ptimer); // deletes bh
    e->etimer = NULL;
    e->ptimer = NULL;
    e->bh = NULL;
    g_free(e);
}

void hpsc_elapsed_timer_event_schedule(HPSCElapsedTimerEvent *e, uint64_t time)
{
    assert(e && e->ptimer);
    uint64_t delta = time - get_count(e->etimer);
    uint64_t delta_ticks = time_to_ticks(e->etimer, delta);
    DB_PRINT("%s: slave event sched @ %lx time (delta %lx time = %lx ticks)\n",
             object_get_canonical_path(OBJECT(e->etimer)),
             time, delta, delta_ticks);
    ptimer_set_count(e->ptimer, delta_ticks);
    ptimer_run(e->ptimer, PTIMER_MODE_ONE_SHOT);
}

static const MemoryRegionOps hpsc_elapsed_ops = {
    .access = hpsc_elapsed_access,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void hpsc_elapsed_realize(DeviceState *dev, Error **errp)
{
    HPSCElapsedTimer *s = HPSC_ELAPSED_TIMER(dev);
    const char *prefix = object_get_canonical_path(OBJECT(dev));
    unsigned int i;

    // Note: there is a limitation from the Qemu timer.h/ptimer.h backend:
    // period[ns] * count must not overflow a int64_t. Period is at most
    // 1/(clk_freq_hz/max_tickdiv) * 10^9 ns (= 256, i.e. 8 bits), so count must be 8 bits
    // smaller (and another -1 bit due to int64_t instead of uint64_t).
    s->max_count = (1ULL << (64 - log2_of_pow2(s->max_tickdiv) - 1)) - 1;
    DB_PRINT("%s: init: max_tickdiv %x max_count %lx\n",
             object_get_canonical_path(OBJECT(s)), s->max_tickdiv, s->max_count);

    for (i = 0; i < ARRAY_SIZE(hpsc_elapsed_regs_info); ++i) {
        DepRegisterInfo *r =
                    &s->regs_info[hpsc_elapsed_regs_info[i].decode.addr / 4];

        *r = (DepRegisterInfo) {
            .data = (uint8_t *)&s->regs[
                    hpsc_elapsed_regs_info[i].decode.addr/4],
            .data_size = sizeof(uint32_t),
            .access = &hpsc_elapsed_regs_info[i],
            .debug = HPSC_ELAPSED_TIMER_ERR_DEBUG,
            .prefix = prefix,
            .opaque = s,
        };
        dep_register_init(r);
        qdev_pass_all_gpios(DEVICE(r), dev);
    }

    s->bh = qemu_bh_new(main_timer_rollover, s);
    s->ptimer = ptimer_init(s->bh, PTIMER_POLICY_DEFAULT);

    s->bh_event = qemu_bh_new(event_timer_rollover, s);
    s->ptimer_event = ptimer_init(s->bh_event, PTIMER_POLICY_DEFAULT);

    QLIST_INIT(&s->slave_events);
}

static void hpsc_elapsed_init(Object *obj)
{
    HPSCElapsedTimer *s = HPSC_ELAPSED_TIMER(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &hpsc_elapsed_ops, s,
                          TYPE_HPSC_ELAPSED_TIMER, R_MAX * 4);
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
    DEFINE_PROP_UINT32("nominal-freq-hz", HPSCElapsedTimer, nominal_freq_hz, 1000000000), // s.t. count units = ns
    DEFINE_PROP_UINT32("clk-freq-hz",  HPSCElapsedTimer,    clk_freq_hz,      125000000),
    DEFINE_PROP_UINT32("max-divider", HPSCElapsedTimer,     max_tickdiv, 32), // f_min=3906250 Hz in spec
    DEFINE_PROP_END_OF_LIST()
};

static void hpsc_elapsed_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = hpsc_elapsed_reset;
    dc->realize = hpsc_elapsed_realize;
    dc->vmsd = &vmstate_hpsc_elapsed;
    dc->props = hpsc_elapsed_props;
}

static const TypeInfo hpsc_elapsed_info = {
    .name          = TYPE_HPSC_ELAPSED_TIMER,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(HPSCElapsedTimer),
    .class_init    = hpsc_elapsed_class_init,
    .instance_init = hpsc_elapsed_init,
};

static void hpsc_elapsed_register_types(void)
{
    type_register_static(&hpsc_elapsed_info);
}

type_init(hpsc_elapsed_register_types)
