#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/log.h"

#include "hw/misc/hpsc_mbox.h"

static void set_int(HPSCMboxInstance *s, uint32_t old_status) {
    unsigned i;

    for (i = 0; i < HPSC_MBOX_INTS; ++i) {
        if ((old_status & (1 << i)) != (s->int_status & (1 << i))) {
            if ((!(s->int_status & (1 << i))) || (s->int_enabled & (1 << i))) { // TODO: is this logic accurate?
                qemu_log_mask(LOG_GUEST_ERROR, "%s: set irq %u to %u\n", __func__, i, s->int_enabled & (1 << i));
                qemu_set_irq(s->arm_irq[i], s->int_status & (1 << i));
            }
        }
    }
}

static bool check_owner(HPSCMboxInstance *si, MemTxAttrs attrs)
{
    if (si->owner != attrs.master_id) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to register by non-owner\n", __func__);
        return false;
    }
    return true;
}

static bool check_dest(HPSCMboxInstance *si, MemTxAttrs attrs)
{
    if (si->dest != attrs.master_id) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to register by non-destination\n", __func__);
        return false;
    }
    return true;
}

static void hpsc_mbox_reset_instance(HPSCMboxInstance *s)
{
    unsigned i;

    s->owner = 0;
    s->dest = 0;
    s->int_enabled = 0;
   
    uint32_t old_status = s->int_status;
    s->int_status  = 0;
    set_int(s, old_status);

    for (i = 0; i < HPSC_MBOX_DATA_REGS; ++i)
        s->data[i] = 0;
}

static void hpsc_mbox_reset(DeviceState *dev)
{
    HPSCMboxState *s = HPSC_MBOX(dev);
    unsigned i;
    for (i = 0; i < HPSC_MBOX_INSTANCES; ++i)
        hpsc_mbox_reset_instance(&s->mbox[i]);
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
            unsigned reg_idx = offset - REG_DATA;
            *r = si->data[reg_idx];
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
        if (!si->owner && value == attrs.master_id) { /* if unclaimed and the writer actually claiming for himself */
            si->owner = value;
            if (si->owner == 0)
                hpsc_mbox_reset_instance(si);
        }
        break;
    case REG_DESTINATION:
        if (!check_owner(si, attrs))
            return MEMTX_ERROR;
        si->dest = value;
        break;
    case REG_INT_ENABLE:
        if (!check_owner(si, attrs) && !check_dest(si, attrs))
            return MEMTX_ERROR;
        si->int_enabled = value & 0b11;
        break;
    case REG_INT_STATUS_CLEAR:
        if (!check_owner(si, attrs) && !check_dest(si, attrs))
            return MEMTX_ERROR;
        old_status = si->int_status;
        si->int_status &= ~value;
        set_int(si, old_status);
        break;
    case REG_INT_STATUS_SET:
        if (!check_owner(si, attrs) && !check_dest(si, attrs))
            return MEMTX_ERROR;

        old_status = si->int_status;
        si->int_status |= value;
        set_int(si, old_status);
        break;
    default:
        if (offset >= REG_DATA) {
            if (!check_owner(si, attrs)) /* is destination allowed to write to data regs? assuming no. */
                return MEMTX_ERROR;
            reg_idx = offset - REG_DATA;
            si->data[reg_idx] = value;
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

    for (i = 0; i < HPSC_MBOX_INTS; ++i) {
        HPSCMboxInstance *si = &s->mbox[i];

        for (j = 0; j < HPSC_MBOX_INTS; ++j)
            sysbus_init_irq(SYS_BUS_DEVICE(s), &si->arm_irq[j]);
    }
}

static void hpsc_mbox_realize(DeviceState *dev, Error **errp)
{
    HPSCMboxState *s = HPSC_MBOX(dev);
    unsigned i;

    for (i = 0; i < HPSC_MBOX_INTS; ++i)
        hpsc_mbox_reset_instance(&s->mbox[i]);
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
