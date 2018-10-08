#ifndef HPSC_MBOX_H
#define HPSC_MBOX_H

#include "hw/sysbus.h"
#if 0
#include "exec/address-spaces.h"
#endif

#define TYPE_HPSC_MBOX "hpsc-mbox"
#define HPSC_MBOX(obj) \
        OBJECT_CHECK(HPSCMboxState, (obj), TYPE_HPSC_MBOX)

#define REG_OWNER             0x00
#define REG_INT_ENABLE        0x04
#define REG_INT_CAUSE         0x08
#define REG_INT_STATUS        0x0C
#define REG_INT_STATUS_CLEAR  0x08 /* TODO: is this overlap by design */
#define REG_INT_STATUS_SET    0x0C
#define REG_DESTINATION       0x1C
#define REG_DATA              0x20

#define HPSC_MBOX_DATA_REGS 16
#define HPSC_MBOX_INTS 2
#define HPSC_MBOX_INSTANCES 32
#define HPSC_MBOX_INSTANCE_REGION (REG_DATA + HPSC_MBOX_DATA_REGS * 4)

typedef struct {
    uint32_t owner;
    uint32_t dest;
    uint32_t int_enabled;
    uint32_t int_status;
    uint32_t data[HPSC_MBOX_DATA_REGS];
    qemu_irq arm_irq[HPSC_MBOX_INTS];

} HPSCMboxInstance;

typedef struct {
    SysBusDevice busdev;
#if 0
    MemoryRegion mbox_mr_real; /* TODO remove */
    MemoryRegion *mbox_mr;
    AddressSpace mbox_as;
#endif
    MemoryRegion iomem;

    HPSCMboxInstance mbox[HPSC_MBOX_INSTANCES];
} HPSCMboxState;

#endif
