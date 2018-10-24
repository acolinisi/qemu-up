#ifndef HPSC_MBOX_H
#define HPSC_MBOX_H

#include "hw/sysbus.h"
#if 0
#include "exec/address-spaces.h"
#endif

#define TYPE_HPSC_MBOX "hpsc-mbox"
#define HPSC_MBOX(obj) \
        OBJECT_CHECK(HPSCMboxState, (obj), TYPE_HPSC_MBOX)

#define REG_CONFIG              0x00
#define REG_EVENT_CAUSE         0x04
#define REG_EVENT_CLEAR         0x04
#define REG_EVENT_STATUS        0x08
#define REG_EVENT_SET           0x08
#define REG_INT_ENABLE          0x0C
#define REG_DATA                0x10

#define REG_CONFIG__UNSECURE      0x1
#define REG_CONFIG__OWNER__SHIFT  8
#define REG_CONFIG__OWNER__MASK   0x0000ff00
#define REG_CONFIG__SRC__SHIFT    16
#define REG_CONFIG__SRC__MASK     0x00ff0000
#define REG_CONFIG__DEST__SHIFT   24
#define REG_CONFIG__DEST__MASK    0xff000000

#define HPSC_MBOX_DATA_REGS 16
#define HPSC_MBOX_INTS 16
#define HPSC_MBOX_EVENTS 2
#define HPSC_MBOX_EVENTS_MASK 0x3
#define HPSC_MBOX_INSTANCES 32
#define HPSC_MBOX_INSTANCE_REGION (REG_DATA + HPSC_MBOX_DATA_REGS * 4)

typedef struct {
    unsigned owner;
    unsigned src;
    unsigned dest;
    bool unsecure;
    uint32_t event_status;
    uint32_t int_enable; // maps event X to interrupt K
    uint32_t data[HPSC_MBOX_DATA_REGS];

} HPSCMboxInstance;

typedef struct {
    SysBusDevice busdev;
#if 0
    MemoryRegion mbox_mr_real; /* TODO remove */
    MemoryRegion *mbox_mr;
    AddressSpace mbox_as;
#endif
    MemoryRegion iomem;

    qemu_irq arm_irq[HPSC_MBOX_INTS];

    HPSCMboxInstance mbox[HPSC_MBOX_INSTANCES];
} HPSCMboxState;

#endif
