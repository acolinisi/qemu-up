#ifndef HW_RIO_SM_RIO_SWITCH_H
#define HW_RIO_SM_RIO_SWITCH_H

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "hw/register.h"
#include "hw/rio/rio.h"
#include "hw/rio/brc1-rio-endpt.h"

#define TYPE_SM_RIO_SWITCH "praesum,sm-rio-switch"

#define SM_RIO_SWITCH(obj) \
     OBJECT_CHECK(SMRIOSwitch, (obj), TYPE_SM_RIO_SWITCH)


REG32(TODO, 0x0)
    FIELD(TODO, TODO, 0, 32)

#define SM_RIO_SWITCH_REG_MAX (R_TODO + 1)

#define SM_RIO_SWITCH_MAX_PORTS 8

typedef struct SMRIOSwitch {
    SysBusDevice parent_obj;
    MemoryRegion iomem;

    uint8_t num_ports;
    struct BRC1RIOEndpt *ports[SM_RIO_SWITCH_MAX_PORTS];
    uint8_t routing_table[SM_RIO_SWITCH_MAX_PORTS];

    /* Output control signals connected to inputs on the CPUs */
    qemu_irq event_irq;

    uint32_t regs[SM_RIO_SWITCH_REG_MAX];
    RegisterInfo regs_info[SM_RIO_SWITCH_REG_MAX];
} SMRIOSwitch;

int sm_rio_switch_in(SMRIOSwitch *s, RIOTx *tx);

#endif /* HW_RIO_SM_RIO_SWITCH_H */
