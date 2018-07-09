/*
 * Raspberry Pi emulation (c) 2012 Gregory Estrade
 * This code is licensed under the GNU GPLv2 and later.
 */

#ifndef HPSC_MBOX_H
#define HPSC_MBOX_H

#include "hpsc_mbox_defs.h"
#include "hw/sysbus.h"
#include "exec/address-spaces.h"

#define TYPE_BCM2835_MBOX "hpsc-mbox"
#define BCM2835_MBOX(obj) \
        OBJECT_CHECK(BCM2835MboxState, (obj), TYPE_BCM2835_MBOX)

typedef struct {
    uint32_t reg[MBOX_SIZE];
    uint32_t count;
    uint32_t status;
    uint32_t config;
} BCM2835Mbox;

typedef struct {
    /*< private >*/
    SysBusDevice busdev;
    /*< public >*/
    MemoryRegion mbox_mr_real;
    MemoryRegion *mbox_mr;
    AddressSpace mbox_as;
    MemoryRegion iomem;
    qemu_irq arm_irq[NUM_BOXES];

    bool mbox_irq_disabled;
    bool available[MBOX_CHAN_COUNT];
    BCM2835Mbox mbox[NUM_BOXES];
} BCM2835MboxState;

#endif
