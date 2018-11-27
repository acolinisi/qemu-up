#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/log.h"

#include "hw/misc/hpsc_mbox.h"

static void update_irq(HPSCMboxState *s, unsigned instance)
{
    HPSCMboxInstance *si = &s->mbox[instance];
    unsigned int_idx;

    // For now, we resolve the mapping here, every time. Alternatively, we
    // could resolve the mapping upon write to Interrupt Enable register, store
    // it in the instance state, and simply set the state of the saved irq here.

    for (int_idx = 0; int_idx < HPSC_MBOX_INTS; ++int_idx) {
        unsigned event_mask = (si->int_enable >> (2 * int_idx)) & HPSC_MBOX_EVENTS_MASK;
        if (event_mask) { // are any events mapped to this interrupt?

            // Are any *of the mapped events* raised?
            bool intval = !!(si->event_status & event_mask);

            qemu_log_mask(LOG_GUEST_ERROR, "%s: instance %u event mask %x: set irq %u to %u\n",
                          __func__, instance, event_mask, int_idx, intval);
            qemu_set_irq(s->arm_irq[int_idx], intval);
        }
    }
}

static bool check_owner(HPSCMboxInstance *si, MemTxAttrs attrs, const char *ctx)
{
#if 0 // HW does not enforce this
    if (si->owner != attrs.master_id) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to register by non-owner '%s'\n", __func__, ctx);
        return false;
    }
#endif
    return true;
}

static bool check_owner_or_dest(HPSCMboxInstance *si, MemTxAttrs attrs, const char *ctx)
{
#if 0 // HW does not enforce this
    if (si->owner != attrs.master_id && si->dest != attrs.master_id) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to register by non-owner/non-dest '%s'\n", __func__, ctx);
        return false;
    }
#endif
    return true;
}

static void hpsc_mbox_reset_instance(HPSCMboxState *s, unsigned instance)
{
    unsigned i;
    HPSCMboxInstance *si = &s->mbox[instance];

    si->owner = 0;
    si->src = 0;
    si->dest = 0;
    si->unsecure = false;
    si->event_status = 0;
    si->int_enable = 0;

    for (i = 0; i < HPSC_MBOX_DATA_REGS; ++i)
        si->data[i] = 0;
}

static void hpsc_mbox_reset(DeviceState *dev)
{
    HPSCMboxState *s = HPSC_MBOX(dev);
    unsigned i, int_idx;
    for (i = 0; i < HPSC_MBOX_INSTANCES; ++i)
        hpsc_mbox_reset_instance(s, i);
    for (int_idx = 0; int_idx < HPSC_MBOX_INTS; ++int_idx)
        qemu_set_irq(s->arm_irq[int_idx], 0);
}

static MemTxResult hpsc_mbox_read(void *opaque, hwaddr offset, uint64_t *r, unsigned size, MemTxAttrs attrs)
{
    uint64_t ie;
    uint32_t int_idx;
    unsigned instance = offset / HPSC_MBOX_INSTANCE_REGION;
    offset %= HPSC_MBOX_INSTANCE_REGION;

    HPSCMboxState *s = HPSC_MBOX(opaque);
    HPSCMboxInstance *si = &s->mbox[instance];

    /* Allow reads to everyone, including non-owner and non-destination */

    switch (offset) {
    case REG_CONFIG:
        *r = ((si->owner << REG_CONFIG__OWNER__SHIFT) & REG_CONFIG__OWNER__MASK) |
             ((si->src   << REG_CONFIG__SRC__SHIFT)   & REG_CONFIG__SRC__MASK) |
             ((si->dest  << REG_CONFIG__DEST__SHIFT)  & REG_CONFIG__DEST__MASK) |
             (si->unsecure ? REG_CONFIG__UNSECURE : 0);
        break;
    case REG_INT_ENABLE:
        *r = si->int_enable;
        break;
    case REG_EVENT_STATUS:
        *r = si->event_status;
        break;
    case REG_EVENT_CAUSE:
        // If the events are mapped to any interrupt, then cause bits are set
        ie = 0;
        for (int_idx = 0; int_idx < HPSC_MBOX_INTS; ++int_idx)
            ie |= (si->int_enable >> (2 * int_idx)) & HPSC_MBOX_EVENTS_MASK;
        *r = si->event_status & ie;
        qemu_log_mask(LOG_GUEST_ERROR, "%s: %p: cause: event_status %x cause %lx\n", __func__, si, si->event_status, *r);
        break;
    default:
        if (offset >= REG_DATA) {
            unsigned reg_idx = (offset - REG_DATA) / 4; // each reg is 32 bits = 4 bytes
            *r = si->data[reg_idx];
            qemu_log_mask(LOG_GUEST_ERROR, "%s: read data[%lx|%u] -> %x\n", __func__, offset, reg_idx, si->data[reg_idx]);
        } else {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: write to unrecognized register\n", __func__);
        }
    }
    return MEMTX_OK;
}

static MemTxResult hpsc_mbox_write(void *opaque, hwaddr offset,
                               uint64_t value, unsigned size, MemTxAttrs attrs)
{
    uint32_t owner, src, dest;
    bool unsecure;
    unsigned reg_idx;

    unsigned instance = offset / HPSC_MBOX_INSTANCE_REGION;
    offset %= HPSC_MBOX_INSTANCE_REGION;

    HPSCMboxState *s = HPSC_MBOX(opaque);
    HPSCMboxInstance *si = &s->mbox[instance];

    switch (offset) {
    case REG_CONFIG:
        owner = (value & REG_CONFIG__OWNER__MASK) >> REG_CONFIG__OWNER__SHIFT;
        src   = (value & REG_CONFIG__SRC__MASK)   >> REG_CONFIG__SRC__SHIFT;
        dest  = (value & REG_CONFIG__DEST__MASK)  >> REG_CONFIG__DEST__SHIFT;
        unsecure = value & REG_CONFIG__UNSECURE; // TODO: enforce
        if (owner) { // claiming
            if (!si->owner /* && value == attrs.master_id */) { /* if unclaimed and the writer actually claiming for himself */
                si->owner = owner;
                si->src = src;
                si->dest = dest;
                si->unsecure = unsecure;
            } else if (si->owner) {
                qemu_log_mask(LOG_GUEST_ERROR, "%s: cannot claim for %x: already claimed by %x\n",
                              __func__, owner, si->owner);
            } else {
                qemu_log_mask(LOG_GUEST_ERROR, "%s: cannot claim on behalf of another bus master %x (self %lx)\n",
                              __func__, owner, attrs.master_id);
            }
        } else { // releasing
            if (owner || !check_owner(si, attrs, "OWNER"))
                return MEMTX_ERROR;
            hpsc_mbox_reset_instance(s, instance);
        }
        break;
    case REG_INT_ENABLE: // bit 2*K+X maps event X to interrupt K
        if (!check_owner_or_dest(si, attrs, "EVENT_ENABLE"))
            return MEMTX_ERROR;
        si->int_enable = value;
        qemu_log_mask(LOG_GUEST_ERROR, "%s: int_enable %x\n", __func__, si->int_enable);
        update_irq(s, instance);
        break;
    case REG_EVENT_CLEAR:
        if (!check_owner_or_dest(si, attrs, "EVENT_CLEAR"))
            return MEMTX_ERROR;
        si->event_status &= ~value;
        qemu_log_mask(LOG_GUEST_ERROR, "%s: event_status %x\n", __func__, si->event_status);
        update_irq(s, instance);
        break;
    case REG_EVENT_SET:
        if (!check_owner_or_dest(si, attrs, "EVENT_SET"))
            return MEMTX_ERROR;
        si->event_status |= value;
        qemu_log_mask(LOG_GUEST_ERROR, "%s: event_status %x\n", __func__, si->event_status);
        update_irq(s, instance);
        break;
    default:
        if (offset >= REG_DATA) {
            if (!check_owner_or_dest(si, attrs, "DATA"))
                return MEMTX_ERROR;
            reg_idx = (offset - REG_DATA) / 4; // each register is 32 bits = 4 bytes
            assert(reg_idx < HPSC_MBOX_DATA_REGS); // otherwise we got here through wrong calc, not bad user code
            si->data[reg_idx] = (uint32_t)value;
            qemu_log_mask(LOG_GUEST_ERROR, "%s: wrote data[%lx|%u] <- %x\n", __func__, offset, reg_idx, (uint32_t)value);
        } else {
            switch (offset) {
                case REG_EVENT_CAUSE:
                case REG_EVENT_STATUS:
                    // TODO: throw exception?
                    qemu_log_mask(LOG_GUEST_ERROR, "%s: write to RO register\n", __func__);
                    break;
                default:
                     qemu_log_mask(LOG_GUEST_ERROR, "%s: write to unrecognized register\n", __func__);
            }
        }
    }

    return MEMTX_OK;
}

static const MemoryRegionOps hpsc_mbox_ops = {
    .read_with_attrs = hpsc_mbox_read,
    .write_with_attrs = hpsc_mbox_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

/* vmstate of a single mailbox */
static const VMStateDescription vmstate_hpsc_mbox_box = {
    .name = TYPE_HPSC_MBOX "_box",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(owner, HPSCMboxInstance),
        VMSTATE_UINT32(dest, HPSCMboxInstance),
        VMSTATE_UINT32(src, HPSCMboxInstance),
        VMSTATE_UINT32(dest, HPSCMboxInstance),
        VMSTATE_UINT32(int_enable, HPSCMboxInstance),
        VMSTATE_UINT32(event_status, HPSCMboxInstance),
        VMSTATE_UINT32_ARRAY(data, HPSCMboxInstance, HPSC_MBOX_DATA_REGS),
        VMSTATE_END_OF_LIST()
    }
};

/* vmstate of the entire device */
static const VMStateDescription vmstate_hpsc_mbox = {
    .name = TYPE_HPSC_MBOX,
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .fields      = (VMStateField[]) {
        VMSTATE_STRUCT_ARRAY(mbox, HPSCMboxState, HPSC_MBOX_INSTANCES, 1,
                             vmstate_hpsc_mbox_box, HPSCMboxInstance),
        VMSTATE_END_OF_LIST()
    }
};

static void hpsc_mbox_init(Object *obj)
{
    HPSCMboxState *s = HPSC_MBOX(obj);
    unsigned int_idx;

    memory_region_init_io(&s->iomem, obj, &hpsc_mbox_ops, s,
                          TYPE_HPSC_MBOX, HPSC_MBOX_INSTANCES * HPSC_MBOX_INSTANCE_REGION);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);

    for (int_idx = 0; int_idx < HPSC_MBOX_INTS; ++int_idx)
        sysbus_init_irq(SYS_BUS_DEVICE(s), &s->arm_irq[int_idx]);
}

static void hpsc_mbox_realize(DeviceState *dev, Error **errp)
{
    HPSCMboxState *s = HPSC_MBOX(dev);
    unsigned i;

    for (i = 0; i < HPSC_MBOX_INSTANCES; ++i)
        hpsc_mbox_reset_instance(s, i);
}

static void hpsc_mbox_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = hpsc_mbox_realize;
    dc->reset = hpsc_mbox_reset;
    dc->vmsd = &vmstate_hpsc_mbox;
}

static TypeInfo hpsc_mbox_info = {
    .name          = TYPE_HPSC_MBOX,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(HPSCMboxState),
    .class_init    = hpsc_mbox_class_init,
    .instance_init = hpsc_mbox_init,
};

static void hpsc_mbox_register_types(void)
{
    type_register_static(&hpsc_mbox_info);
}

type_init(hpsc_mbox_register_types)
