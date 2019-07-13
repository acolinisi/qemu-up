#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "hw/register.h"
#include "qemu/bitops.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qapi/qmp/qerror.h"
#include "hw/rio/sm-rio-switch.h"
#include "hw/rio/brc1-rio-endpt.h"

#include "hw/fdt_generic_util.h"

#ifndef SM_RIO_SWITCH_ERR_DEBUG
#define SM_RIO_SWITCH_ERR_DEBUG 0
#endif

#define IOMEM_REGION_SIZE   0x00200000 /* 2 MB */

#define PORT_INVALID 0xff

int sm_rio_switch_in(SMRIOSwitch *s, RIOTx *tx)
{
    uint8_t port_idx = s->routing_table[tx->dest];
    BRC1RIOEndpt *ep = s->ports[port_idx];
    brc1_rio_endpt_in(ep, tx);
    return 0;
}

static uint64_t sm_rio_switch_read(void *opaque, hwaddr addr, unsigned size)
{
    SMRIOSwitch *s = SM_RIO_SWITCH(opaque);
    RegisterInfo *r = &s->regs_info[addr / 4];
    return register_read(r, ~0, NULL, false);
}

static void sm_rio_switch_write(void *opaque, hwaddr addr, uint64_t value,
                      unsigned size)
{
    SMRIOSwitch *s = SM_RIO_SWITCH(opaque);
    RegisterInfo *r = &s->regs_info[addr / 4];
    register_write(r, value, ~0, NULL, false);
}

static void sm_rio_switch_halt_post_write(RegisterInfo *reg, uint64_t val)
{
    //SMRIOSwitch *s = SM_RIO_SWITCH(reg->opaque);
}

static const RegisterAccessInfo sm_rio_switch_regs_info[] = {
    { .name = "TODO",  .addr = A_TODO,
        .reset = 0x0,
        .rsvd = 0x0,
        .post_write = sm_rio_switch_halt_post_write,
    },
};

static const MemoryRegionOps sm_rio_switch_ops = {
    .read = sm_rio_switch_read,
    .write = sm_rio_switch_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};


static void sm_rio_switch_reset(DeviceState *dev)
{
    SMRIOSwitch *s = SM_RIO_SWITCH(dev);
    unsigned int i;

    for (i = 0; i < ARRAY_SIZE(s->regs_info); ++i) {
        register_reset(&s->regs_info[i]);
    }
}

static void sm_rio_switch_realize(DeviceState *dev, Error **errp)
{
    SMRIOSwitch *s = SM_RIO_SWITCH(dev);
    unsigned int i;

    for (i = 0; i < ARRAY_SIZE(sm_rio_switch_regs_info); ++i) {
        RegisterInfo *r = &s->regs_info[sm_rio_switch_regs_info[i].addr / 4];

        *r = (RegisterInfo) {
            .data = (uint8_t *)&s->regs[sm_rio_switch_regs_info[i].addr / 4],
            .data_size = sizeof(uint32_t),
            .access = &sm_rio_switch_regs_info[i],
            .opaque = s,
        };
    }

#if 0
    for (i = 0; i < s->num_ports; ++i) {
    }
#endif
}

static void sm_rio_switch_init(Object *obj)
{
    SMRIOSwitch *s = SM_RIO_SWITCH(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    int i;
    
    for (i = 0; i < SM_RIO_SWITCH_MAX_PORTS; ++i) {
        s->routing_table[i] = PORT_INVALID;
    }

    // TODO: expose via registers
    s->routing_table[0x0] = 0;
    s->routing_table[0x1] = 1;

    memory_region_init_io(&s->iomem, obj, &sm_rio_switch_ops,
                          s, TYPE_SM_RIO_SWITCH, IOMEM_REGION_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(s), &s->event_irq);
}

static const VMStateDescription vmstate_sm_rio_switch = {
    .name = TYPE_SM_RIO_SWITCH,
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, SMRIOSwitch, SM_RIO_SWITCH_REG_MAX),
        VMSTATE_END_OF_LIST(),
    }
};

static Property sm_rio_switch_properties[] = {
#if 0
    DEFINE_PROP_LINK("port", SMRIOSwitch, port,
                     TYPE_BRC1_RIO_PORT, RIOPort *),
#else
    // TODO: array
    DEFINE_PROP_LINK("port0", SMRIOSwitch, ports[0],
                     TYPE_BRC1_RIO_ENDPT, BRC1RIOEndpt *),
    DEFINE_PROP_LINK("port1", SMRIOSwitch, ports[1],
                     TYPE_BRC1_RIO_ENDPT, BRC1RIOEndpt *),
#endif
    DEFINE_PROP_UINT8("num-ports", SMRIOSwitch, num_ports, 2),
    DEFINE_PROP_END_OF_LIST(),
};

static void sm_rio_switch_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = sm_rio_switch_reset;
    dc->realize = sm_rio_switch_realize;
    dc->props = sm_rio_switch_properties;
    dc->vmsd = &vmstate_sm_rio_switch;
}

static const TypeInfo sm_rio_switch_info = {
    .name          = TYPE_SM_RIO_SWITCH,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(SMRIOSwitch),
    .class_init    = sm_rio_switch_class_init,
    .instance_init = sm_rio_switch_init,
};

static void sm_rio_switch_register_types(void)
{
    type_register_static(&sm_rio_switch_info);
}

type_init(sm_rio_switch_register_types)
