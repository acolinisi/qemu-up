#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/log.h"

#include "hw/misc/hpsc_mbox.h"

static void set_int(HPSCMboxState *s, uint32_t instance, uint32_t old_status) {
    unsigned i;
    HPSCMboxInstance *si = &s->mbox[instance];

    for (i = 0; i < HPSC_MBOX_INTS; ++i) {
        if (si->int_enabled & (1 << i)) {
            bool intval = !!(si->int_status & (1 << i));

            qemu_log_mask(LOG_GUEST_ERROR, "%s: instance %u: set irq %u to %u\n", __func__, instance, i, intval);
            qemu_set_irq(si->arm_irq[i], intval);
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
    si->dest = 0;
    si->int_enabled = 0;
   
    uint32_t old_status = si->int_status;
    si->int_status  = 0;
    set_int(s, instance, old_status);

    for (i = 0; i < HPSC_MBOX_DATA_REGS; ++i)
        si->data[i] = 0;
}

static void hpsc_mbox_reset(DeviceState *dev)
{
    HPSCMboxState *s = HPSC_MBOX(dev);
    unsigned i, j;
    for (i = 0; i < HPSC_MBOX_INSTANCES; ++i) {
        hpsc_mbox_reset_instance(s, i);
        for (j = 0; j < HPSC_MBOX_INTS; ++j)
            qemu_set_irq(s->mbox[i].arm_irq[j], 0);
    }
}

static MemTxResult hpsc_mbox_read(void *opaque, hwaddr offset, uint64_t *r, unsigned size, MemTxAttrs attrs)
{
    unsigned instance = offset / HPSC_MBOX_INSTANCE_REGION;
    offset %= HPSC_MBOX_INSTANCE_REGION;

    HPSCMboxState *s = HPSC_MBOX(opaque);
    HPSCMboxInstance *si = &s->mbox[instance];

    /* Allow reads to everyone, including non-owner and non-destination */

    switch (offset) {
    case REG_OWNER:
        *r = si->owner;
        break;
    case REG_DESTINATION:
        *r = si->dest;
        break;
    case REG_INT_ENABLE:
        *r = si->int_enabled;
        break;
    case REG_INT_CAUSE:
        *r = si->int_status & si->int_enabled;
        break;
    case REG_INT_STATUS:
        *r = si->int_status;
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
    uint32_t old_status;
    unsigned reg_idx;

    unsigned instance = offset / HPSC_MBOX_INSTANCE_REGION;
    offset %= HPSC_MBOX_INSTANCE_REGION;

    HPSCMboxState *s = HPSC_MBOX(opaque);
    HPSCMboxInstance *si = &s->mbox[instance];

    switch (offset) {
    case REG_OWNER:
        if (value) { // claiming
            if (!si->owner /* && value == attrs.master_id */) { /* if unclaimed and the writer actually claiming for himself */
                si->owner = value;
            } else if (si->owner) {
                qemu_log_mask(LOG_GUEST_ERROR, "%s: cannot claim for %lx: already claimed by %x\n", __func__, value, si->owner);
            } else {
                qemu_log_mask(LOG_GUEST_ERROR, "%s: cannot claim on behalf of another bus master %lx (self %lx)\n",
                              __func__, value, attrs.master_id);
            }
        } else { // releasing
            if (value || !check_owner(si, attrs, "OWNER"))
                return MEMTX_ERROR;
            si->owner = 0;
            hpsc_mbox_reset_instance(s, instance);
        }
        break;
    case REG_DESTINATION:
        if (!check_owner(si, attrs, "DESTINATION"))
            return MEMTX_ERROR;
        si->dest = value;
        break;
    case REG_INT_ENABLE:
        if (!check_owner_or_dest(si, attrs, "INT_ENABLE"))
            return MEMTX_ERROR;
        si->int_enabled = value & 0b11;
        break;
    case REG_INT_STATUS_CLEAR:
        if (!check_owner_or_dest(si, attrs, "INT_STATUS_CLEAR"))
            return MEMTX_ERROR;
        old_status = si->int_status;
        si->int_status &= ~value;
        set_int(s, instance, old_status);
        break;
    case REG_INT_STATUS_SET:
        if (!check_owner_or_dest(si, attrs, "INT_STATUS_SET"))
            return MEMTX_ERROR;

        old_status = si->int_status;
        si->int_status |= value;
        set_int(s, instance, old_status);
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
            case REG_INT_CAUSE:
            case REG_INT_STATUS:
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
        VMSTATE_UINT32(int_enabled, HPSCMboxInstance),
        VMSTATE_UINT32(int_status, HPSCMboxInstance),
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
    unsigned i, j;

    memory_region_init_io(&s->iomem, obj, &hpsc_mbox_ops, s,
                          TYPE_HPSC_MBOX, HPSC_MBOX_INSTANCES * HPSC_MBOX_INSTANCE_REGION);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);

    for (i = 0; i < HPSC_MBOX_INSTANCES; ++i)
        for (j = 0; j < HPSC_MBOX_INTS; ++j)
            sysbus_init_irq(SYS_BUS_DEVICE(s), &s->mbox[i].arm_irq[j]);
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
