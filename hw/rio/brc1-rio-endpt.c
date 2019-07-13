#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "hw/register.h"
#include "qemu/bitops.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qapi/qmp/qerror.h"
#include "hw/fdt_generic_util.h"
#include "hw/rio/brc1-rio-endpt.h"
#include "hw/rio/sm-rio-switch.h"

#ifndef BRC1_RIO_ENDPT_ERR_DEBUG
#define BRC1_RIO_ENDPT_ERR_DEBUG 0
#endif

#define IOMEM_REGION_SIZE   0x01000000 /* 16 MB */

int brc1_rio_endpt_in(BRC1RIOEndpt *s, RIOTx *tx)
{
    s->regs[R_RAW_MSG_OUT] = tx->payload;
    qemu_set_irq(s->event_irq, 1);
    return 0;
}

static uint64_t brc1_rio_endpt_read(void *opaque, hwaddr addr, unsigned size)
{
    BRC1RIOEndpt *s = BRC1_RIO_ENDPT(opaque);
    RegisterInfo *r = &s->regs_info[addr / 4];
    return register_read(r, ~0, NULL, false);
}

static void brc1_rio_endpt_write(void *opaque, hwaddr addr, uint64_t value,
                      unsigned size)
{
    BRC1RIOEndpt *s = BRC1_RIO_ENDPT(opaque);
    RegisterInfo *r = &s->regs_info[addr / 4];
    register_write(r, value, ~0, NULL, false);
}

static void brc1_rio_endpt_halt_post_write(RegisterInfo *reg, uint64_t val)
{
    BRC1RIOEndpt *s = BRC1_RIO_ENDPT(reg->opaque);

    RIOTx tx = {
        .dest = val & 0xff,
        .payload = val >> 8,
    };

    int rc = sm_rio_switch_in(s->rio_switch, &tx);
    if (rc) {
        qemu_log("ERROR: failed to pass transaction to the switch");
    }
}

static uint64_t brc1_rio_endpt_msg_out_post_read(RegisterInfo *reg, uint64_t val)
{
    BRC1RIOEndpt *s = BRC1_RIO_ENDPT(reg->opaque);
	(void)s;
	return val;
}

static const RegisterAccessInfo brc1_rio_endpt_regs_info[] = {
    { .name = "RAW_MSG_IN",  .addr = A_RAW_MSG_IN,
        .reset = 0x0,
        .rsvd = 0x0,
        .post_write = brc1_rio_endpt_halt_post_write,
    },{
        .name = "RAW_MSG_OUT",  .addr = A_RAW_MSG_OUT,
        .reset = 0x0,
        .rsvd = 0x0,
        .post_read = brc1_rio_endpt_msg_out_post_read,
    },
};

static const MemoryRegionOps brc1_rio_endpt_ops = {
    .read = brc1_rio_endpt_read,
    .write = brc1_rio_endpt_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};


static void brc1_rio_endpt_reset(DeviceState *dev)
{
    BRC1RIOEndpt *s = BRC1_RIO_ENDPT(dev);
    unsigned int i;

    for (i = 0; i < ARRAY_SIZE(s->regs_info); ++i) {
        register_reset(&s->regs_info[i]);
    }
}

static void brc1_rio_endpt_realize(DeviceState *dev, Error **errp)
{
    BRC1RIOEndpt *s = BRC1_RIO_ENDPT(dev);
    unsigned int i;

    for (i = 0; i < ARRAY_SIZE(brc1_rio_endpt_regs_info); ++i) {
        RegisterInfo *r = &s->regs_info[brc1_rio_endpt_regs_info[i].addr / 4];

        *r = (RegisterInfo) {
            .data = (uint8_t *)&s->regs[brc1_rio_endpt_regs_info[i].addr / 4],
            .data_size = sizeof(uint32_t),
            .access = &brc1_rio_endpt_regs_info[i],
            .opaque = s,
        };
    }
}

static void brc1_rio_endpt_init(Object *obj)
{
    BRC1RIOEndpt *s = BRC1_RIO_ENDPT(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &brc1_rio_endpt_ops,
                          s, TYPE_BRC1_RIO_ENDPT, IOMEM_REGION_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(s), &s->event_irq);
}

static const VMStateDescription vmstate_brc1_rio_endpt = {
    .name = TYPE_BRC1_RIO_ENDPT,
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, BRC1RIOEndpt, REG_MAX),
        VMSTATE_END_OF_LIST(),
    }
};

static Property brc1_rio_endpt_properties[] = {
#if 0
    DEFINE_PROP_LINK("port", BRC1RIOEndpt, port,
                     TYPE_BRC1_RIO_PORT, RIOPort *),
#else
    DEFINE_PROP_LINK("switch", BRC1RIOEndpt, rio_switch,
                     TYPE_SM_RIO_SWITCH, SMRIOSwitch *),
    DEFINE_PROP_UINT8("port-idx", BRC1RIOEndpt, rio_switch_port_idx, 0),
#endif
    DEFINE_PROP_END_OF_LIST(),
};

static void brc1_rio_endpt_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = brc1_rio_endpt_reset;
    dc->realize = brc1_rio_endpt_realize;
    dc->props = brc1_rio_endpt_properties;
    dc->vmsd = &vmstate_brc1_rio_endpt;
}

static const TypeInfo brc1_rio_endpt_info = {
    .name          = TYPE_BRC1_RIO_ENDPT,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(BRC1RIOEndpt),
    .class_init    = brc1_rio_endpt_class_init,
    .instance_init = brc1_rio_endpt_init,
};

static void brc1_rio_endpt_register_types(void)
{
    type_register_static(&brc1_rio_endpt_info);
}

type_init(brc1_rio_endpt_register_types)
