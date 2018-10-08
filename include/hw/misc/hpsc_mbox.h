#ifndef HPSC_MBOX_H
#define HPSC_MBOX_H

#include "hw/sysbus.h"
#if 0
#include "exec/address-spaces.h"
#endif

#define TYPE_HPSC_MBOX "hpsc-mbox"
#define HPSC_MBOX(obj) \
        OBJECT_CHECK(HPSCMboxState, (obj), TYPE_HPSC_MBOX)

#define HPSC_MBOX_DATA_REGS 16
#define HPSC_MBOX_INTS 2
#define HPSC_MBOX_INSTANCES 32

typedef struct {
    uint32_t owner;
    uint32_t dest;
    uint32_t int_enabled;
    uint32_t int_status;
    uint32_t data[HPSC_MBOX_DATA_REGS];
    qemu_irq arm_irq[HPSC_MBOX_INTS];

    MemoryRegion iomem;
} HPSCMboxInstance;

typedef struct {
    SysBusDevice busdev;
#if 0
    MemoryRegion mbox_mr_real; /* TODO remove */
    MemoryRegion *mbox_mr;
    AddressSpace mbox_as;
#endif

    HPSCMboxInstance mbox[HPSC_MBOX_INSTANCES];
} HPSCMboxState;

#endif
