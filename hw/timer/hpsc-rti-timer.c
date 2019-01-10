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
#include "hw/timer/hpsc-elapsed-timer.h"

#ifndef HPSC_RTI_TIMER_ERR_DEBUG
#define HPSC_RTI_TIMER_ERR_DEBUG 0
#endif

#define TYPE_HPSC_RTI_TIMER "hpsc,hpsc-rti-timer"

#define DB_PRINT_L(lvl, fmt, args...) do {\
    if (HPSC_RTI_TIMER_ERR_DEBUG >= lvl) {\
        qemu_log(TYPE_HPSC_RTI_TIMER ": %s:" fmt, __func__, ## args);\
    } \
} while (0);

#define DB_PRINT(fmt, args...) DB_PRINT_L(1, fmt, ## args)

#define HPSC_RTI_TIMER(obj) \
     OBJECT_CHECK(HPSCRTITimer, (obj), TYPE_HPSC_RTI_TIMER)

// TODO: is there support for 64-bit registers? to avoid HI,LO

DEP_REG32(REG_INTERVAL_LO,      0x00)
DEP_REG32(REG_INTERVAL_HI,      0x04)
DEP_REG32(REG_COUNT_LO,         0x08)
DEP_REG32(REG_COUNT_HI,         0x0c)

DEP_REG32(REG_CMD_ARM,          0x10)
DEP_REG32(REG_CMD_FIRE,         0x14)

#define R_MAX (R_REG_CMD_FIRE + 1)

#define PTIMER_MODE_ONE_SHOT 1
#define PTIMER_MODE_CONT     0

typedef enum {
    CMD_INVALID = 0,
    CMD_CAPTURE,
    CMD_LOAD,
} cmd_t;

// TODO: Codes from spec, once spec has them
#define CMD_CAPTURE_ARM    0xCD01
#define CMD_LOAD_ARM       0xCD02

#define CMD_CAPTURE_FIRE   0x01CD
#define CMD_LOAD_FIRE      0x02CD

typedef struct HPSCRTITimer {
    SysBusDevice parent_obj;
    MemoryRegion iomem;

    struct HPSCElapsedTimer *etimer;
    struct HPSCElapsedTimerEvent *etimer_event;

    uint64_t start_count;
    uint64_t interval;

    qemu_irq irq;

    uint32_t regs[R_MAX];
    DepRegisterInfo regs_info[R_MAX];
} HPSCRTITimer;


static uint64_t get_count(HPSCRTITimer *s)
{
    uint64_t count = hpsc_elapsed_timer_get_count(s->etimer);
    DB_PRINT("%s: count -> %lx\n",
            object_get_canonical_path(OBJECT(s)), count);
    return count;
}

static void schedule_event(HPSCRTITimer *s)
{
    uint64_t event;

    s->start_count = get_count(s);
    event = s->start_count + s->interval;
    DB_PRINT("%s: sched event @ %lx\n",
            object_get_canonical_path(OBJECT(s)), event);
    hpsc_elapsed_timer_event_schedule(s->etimer_event, event);
}

static void etimer_event_cb(void *opaque)
{
    HPSCRTITimer *s = opaque;
    DB_PRINT("%s: etimer event cb\n", object_get_canonical_path(OBJECT(s)));

    // Generate an edge, the interrupt controller should latch it
    qemu_set_irq(s->irq, 1);
    qemu_set_irq(s->irq, 0);

    schedule_event(s);
}

static void execute_cmd(HPSCRTITimer *s, cmd_t cmd)
{
    uint64_t count;
    switch (cmd) {
        case CMD_CAPTURE:
                count = get_count(s) - s->start_count;
                s->regs[R_REG_COUNT_LO] = (uint32_t)(count & 0xffffffff);
                s->regs[R_REG_COUNT_HI] = (uint32_t)(count >> 32);
                DB_PRINT("%s: cmd: capture: count -> %lx\n",
                         object_get_canonical_path(OBJECT(s)), count);
                break;
        case CMD_LOAD:
                s->interval = ((uint64_t)s->regs[R_REG_INTERVAL_HI] << 32) | s->regs[R_REG_INTERVAL_LO];
                s->start_count = get_count(s);
                DB_PRINT("%s: cmd: load: interval <- %lx\n",
                         object_get_canonical_path(OBJECT(s)), s->interval);
                // Note: we don't de-assert the IRQ here. So, if the LOAD
                // command executes while IRQ is asserted (i.e. within one
                // cycle of the timer clock), then the interrupt controller
                // will not get an edge, so software won't get an interrupt.
                // This seems to agree with the behavior of the HW from the
                // spec diagram.
                schedule_event(s);
                break;
        default:
                assert(false && "unhandled cmd");
    }
}

static void post_write_cmd_fire(DepRegisterInfo *reg, uint64_t val64)
{
    HPSCRTITimer *s = HPSC_RTI_TIMER(reg->opaque);
    cmd_t cmd = CMD_INVALID;
    uint32_t arm_code = s->regs[R_REG_CMD_ARM];
    DB_PRINT("%s: arm code: %x\n",
             object_get_canonical_path(OBJECT(s)), arm_code);
    switch (val64) {
        case CMD_CAPTURE_FIRE: if (arm_code == CMD_CAPTURE_ARM) cmd = CMD_CAPTURE; break;
        case CMD_LOAD_FIRE:    if (arm_code == CMD_LOAD_ARM)    cmd = CMD_LOAD;    break;
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

static DepRegisterAccessInfo hpsc_rti_regs_info[] = {
    { .name = "REG_COUNT_LO",
        .decode.addr = A_REG_COUNT_LO,
        .reset = 0x0,
        .ro = ~0,
    },{ .name = "REG_COUNT_HI",
        .decode.addr = A_REG_COUNT_HI,
        .reset = 0x0,
        .ro = ~0,
    },{ .name = "REG_INTERVAL_LO",
        .decode.addr = A_REG_INTERVAL_LO,
         //.reset = ~0x0, // set to parent timer's max count on reset
    },{ .name = "REG_INTERVAL_HI",
        .decode.addr = A_REG_INTERVAL_HI,
         //.reset = ~0x0, // set to parent timer's max count on reset
    },{ .name = "REG_CMD_ARM",
        .decode.addr = A_REG_CMD_ARM,
        .reset = 0x0,
    },{ .name = "REG_CMD_FIRE",
        .decode.addr = A_REG_CMD_FIRE,
        .reset = 0x0,
        .post_write = post_write_cmd_fire,
   }
};

static void hpsc_rti_reset(DeviceState *dev)
{
    HPSCRTITimer *s = HPSC_RTI_TIMER(dev);
    unsigned int i;

    for (i = 0; i < ARRAY_SIZE(s->regs_info); ++i)
        dep_register_reset(&s->regs_info[i]);

    qemu_set_irq(s->irq, 0);
}

static uint64_t hpsc_rti_read(void *opaque, hwaddr addr, unsigned size)
{
    HPSCRTITimer *s = HPSC_RTI_TIMER(opaque);
    DepRegisterInfo *r = &s->regs_info[addr / 4];

    if (!r->data) {
        qemu_log_mask(LOG_GUEST_ERROR,
                "%s: Decode error: read from %" HWADDR_PRIx "\n",
                object_get_canonical_path(OBJECT(s)), addr);
        return 0;
    }
    return dep_register_read(r);
}

static void hpsc_rti_write(void *opaque, hwaddr addr, uint64_t value,
                      unsigned size)
{
    HPSCRTITimer *s = HPSC_RTI_TIMER(opaque);
    DepRegisterInfo *r = &s->regs_info[addr / 4];

    if (!r->data) {
        qemu_log_mask(LOG_GUEST_ERROR,
                "%s: Decode error: write to %" HWADDR_PRIx "=%" PRIx64 "\n",
                object_get_canonical_path(OBJECT(s)), addr, value);
        return;
    }
    dep_register_write(r, value, ~0);
}

static void hpsc_rti_access(MemoryTransaction *tr)
{
#if 0
    MemTxAttrs attr = tr->attr;
#endif
    void *opaque = tr->opaque;
    hwaddr addr = tr->addr;
    unsigned size = tr->size;
    uint64_t value = tr->data.u64;;
    bool is_write = tr->rw;

#if 0 // TODO: spec: restricted to secure access?
    if (!attr.secure) {
        qemu_log_mask(LOG_GUEST_ERROR, "unsecure access to timer denied\n");
        return;
    }
#endif

    if (is_write) {
        hpsc_rti_write(opaque, addr, value, size);
    } else {
        tr->data.u64 = hpsc_rti_read(opaque, addr, size);
    }
}

static const MemoryRegionOps hpsc_rti_ops = {
    .access = hpsc_rti_access,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void hpsc_rti_realize(DeviceState *dev, Error **errp)
{
    HPSCRTITimer *s = HPSC_RTI_TIMER(dev);
    const char *prefix = object_get_canonical_path(OBJECT(dev));
    unsigned int i;
    uint64_t max_count;
    DepRegisterAccessInfo *rai;

    // Ideally, max count would be constant ~0ULL, but due to limitations in
    // the backedn, we can't have all 64-bits (see Elapsed Timer model).
    max_count = hpsc_elapsed_timer_get_max_count(s->etimer);
    rai = &hpsc_rti_regs_info[R_REG_INTERVAL_HI];
    rai->reset = max_count >> 32;
    rai->rsvd = ~rai->reset;
    rai = &hpsc_rti_regs_info[R_REG_INTERVAL_LO];
    rai->reset = max_count & 0xffffffff;
    rai->rsvd = ~rai->reset;

    for (i = 0; i < ARRAY_SIZE(hpsc_rti_regs_info); ++i) {
        DepRegisterInfo *r =
                    &s->regs_info[hpsc_rti_regs_info[i].decode.addr / 4];

        *r = (DepRegisterInfo) {
            .data = (uint8_t *)&s->regs[
                    hpsc_rti_regs_info[i].decode.addr/4],
            .data_size = sizeof(uint32_t),
            .access = &hpsc_rti_regs_info[i],
            .debug = HPSC_RTI_TIMER_ERR_DEBUG,
            .prefix = prefix,
            .opaque = s,
        };
        dep_register_init(r);
        qdev_pass_all_gpios(DEVICE(r), dev);
    }

    s->etimer_event = hpsc_elapsed_timer_event_create(s->etimer,
                                etimer_event_cb, s);
}

static void hpsc_rti_init(Object *obj)
{
    HPSCRTITimer *s = HPSC_RTI_TIMER(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &hpsc_rti_ops, s,
                          TYPE_HPSC_RTI_TIMER, R_MAX * 4);
    sysbus_init_mmio(sbd, &s->iomem);

    sysbus_init_irq(SYS_BUS_DEVICE(s), &s->irq);

    // TODO: define an interface instead of depending on Elapsed Timer directly
    object_property_add_link(obj, "master-timer", TYPE_HPSC_ELAPSED_TIMER,
                             (Object **)&s->etimer,
                             qdev_prop_allow_set_link_before_realize,
                             OBJ_PROP_LINK_UNREF_ON_RELEASE,
                             &error_abort);
}

static const VMStateDescription vmstate_hpsc_rti = {
    .name = TYPE_HPSC_RTI_TIMER,
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, HPSCRTITimer, R_MAX),
        VMSTATE_END_OF_LIST(),
    }
};

static Property hpsc_rti_props[] = {
    DEFINE_PROP_END_OF_LIST()
};

static void hpsc_rti_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = hpsc_rti_reset;
    dc->realize = hpsc_rti_realize;
    dc->vmsd = &vmstate_hpsc_rti;
    dc->props = hpsc_rti_props;
}

static const TypeInfo hpsc_rti_info = {
    .name          = TYPE_HPSC_RTI_TIMER,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(HPSCRTITimer),
    .class_init    = hpsc_rti_class_init,
    .instance_init = hpsc_rti_init,
};

static void hpsc_rti_register_types(void)
{
    type_register_static(&hpsc_rti_info);
}

type_init(hpsc_rti_register_types)
