/*
 * Raspberry Pi emulation (c) 2012 Gregory Estrade
 * This code is licensed under the GNU GPLv2 and later.
 *
 * This file models the system mailboxes, which are used for
 * communication with low-bandwidth GPU peripherals. Refs:
 *   https://github.com/raspberrypi/firmware/wiki/Mailboxes
 *   https://github.com/raspberrypi/firmware/wiki/Accessing-mailboxes
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/misc/hpsc_mbox.h"
#include "qemu/log.h"


#define REG_OWNER             0x00
#define REG_INT_ENABLE        0x04
#define REG_INT_CAUSE         0x08
#define REG_INT_STATUS        0x0C
#define REG_INT_STATUS_CLEAR  0x08 /* TODO: is this overlap by design */
#define REG_INT_STATUS_SET    0x0C
#define REG_DESTINATION       0x1C
#define REG_DATA              0x20

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

static bool check_owner(HPSCMboxInstance *s /* TODO memattr_t attr */)
{
#if 0
    if (s->owner != attr->master_id /* TODO */) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to register by non-owner\n", __func__);
        return false;
    }
#endif
    return true;
}

static bool check_dest(HPSCMboxInstance *s /* TODO memattr_t attr */)
{
#if 0
    if (s->dest != attr->master_id /* TODO */) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to register by non-destination\n", __func__);
        return false;
    }
#endif
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

static uint64_t hpsc_mbox_read(void *opaque, hwaddr offset, unsigned size)
{
    HPSCMboxInstance *s = opaque;

    offset &= 0xff;

    /* Allow reads to everyone, including non-owner and non-destination */

    switch (offset) {
    case REG_OWNER:
        return s->owner;
    case REG_DESTINATION:
        return s->dest;
    case REG_INT_ENABLE:
        return s->int_enabled;
    case REG_INT_CAUSE:
        return s->int_status & s->int_enabled;
    case REG_INT_STATUS:
        return s->int_status;
    default:
        if (offset >= REG_DATA) {
            unsigned reg_idx = offset - REG_DATA;
            return s->data[reg_idx];
        } else {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: write to unrecognized register\n", __func__);
        }
    }
    return 0;
}

static void hpsc_mbox_write(void *opaque, hwaddr offset,
                               uint64_t value, unsigned size)
{
    HPSCMboxInstance *s = opaque;
    uint32_t old_status;
    unsigned reg_idx;

    offset &= 0xff;

    switch (offset) {
    case REG_OWNER:

#if 0
        if (!s->owner && value == attr->master_id /* TODO */) { /* if unclaimed and the writer actually claiming for himself */
            s->owner = value;
            if (s->owner = 0)
                hpsc_mbox_reset_instance(s);
        }
#endif
        break;
    case REG_DESTINATION:
        if (!check_owner(s))
            return;
        s->dest = value;
        break;
    case REG_INT_ENABLE:
        if (!check_owner(s) && !check_dest(s))
            return;
        s->int_enabled = value & 0b11;
        break;
    case REG_INT_STATUS_CLEAR:
        if (!check_owner(s) && !check_dest(s))
            return;
        old_status = s->int_status;
        s->int_status &= ~value;
        set_int(s, old_status);
        break;
    case REG_INT_STATUS_SET:
        if (!check_owner(s) && !check_dest(s))
            return;

        old_status = s->int_status;
        s->int_status |= value;
        set_int(s, old_status);
        break;
    default:
        if (offset >= REG_DATA) {
            if (!check_owner(s)) /* is destination allowed to write to data regs? assuming no. */
                return;
            reg_idx = offset - REG_DATA;
            s->data[reg_idx] = value;
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
}

static const MemoryRegionOps hpsc_mbox_ops = {
    .read = hpsc_mbox_read,
    .write = hpsc_mbox_write,
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

    for (i = 0; i < HPSC_MBOX_INTS; ++i) {
        HPSCMboxInstance *si = &s->mbox[i];

        memory_region_init_io(&si->iomem, obj, &hpsc_mbox_ops, si,
                              TYPE_HPSC_MBOX, REG_DATA + HPSC_MBOX_DATA_REGS * 4);
        sysbus_init_mmio(SYS_BUS_DEVICE(s), &si->iomem);

        for (j = 0; j < HPSC_MBOX_INTS; ++j)
            sysbus_init_irq(SYS_BUS_DEVICE(s), &si->arm_irq[j]);
    }
}

static void hpsc_mbox_realize(DeviceState *dev, Error **errp)
{
    HPSCMboxState *s = HPSC_MBOX(dev);
    unsigned i;

#if 0
    /* Internal memory region for request/response communication with
     * mailbox-addressable peripherals (not exported)
     */
    memory_region_init(&s->mbox_mr_real, OBJECT(dev), "hpsc-mbox",
                       MBOX_CHAN_COUNT << MBOX_AS_CHAN_SHIFT);

/*
    object_property_add_const_link(OBJECT(&s->mboxes), "mbox-mr",
                                   OBJECT(&s->mbox_mr), &error_abort);

    obj = object_property_get_link(OBJECT(dev), "mbox-mr", &err);
*/
    obj = OBJECT(&s->mbox_mr_real);
    if (obj == NULL) {
        error_setg(errp, "%s: required mbox-mr link not found: %s",
                   __func__, error_get_pretty(err));
        return;
    }

    s->mbox_mr = MEMORY_REGION(obj);
    address_space_init(&s->mbox_as, s->mbox_mr, NULL);
#endif
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
