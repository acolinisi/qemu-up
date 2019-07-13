#ifndef HW_RIO_BRC1_RIO_ENDPT
#define HW_RIO_BRC1_RIO_ENDPT

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "hw/register.h"
#include "hw/rio/rio.h"
#include "hw/rio/sm-rio-switch.h"

#define TYPE_BRC1_RIO_ENDPT "praesum,brc1-rio-endpt"

#define BRC1_RIO_ENDPT(obj) \
     OBJECT_CHECK(BRC1RIOEndpt, (obj), TYPE_BRC1_RIO_ENDPT)

REG32(RAW_MSG_IN, 0x0)
    FIELD(RAW_MSG_IN, DATA, 0, 32)
REG32(RAW_MSG_OUT, 0x4)
    FIELD(RAW_MSG_OUT, DATA, 0, 32)

#define REG_MAX (R_RAW_MSG_OUT + 1)

typedef struct BRC1RIOEndpt {
    SysBusDevice parent_obj;
    MemoryRegion iomem;

#if 0
    RIOPort *port;
#else
    struct SMRIOSwitch *rio_switch;
    uint8_t rio_switch_port_idx;
#endif

    /* Output control signals connected to inputs on the CPUs */
    qemu_irq event_irq;

    uint32_t regs[REG_MAX];
    RegisterInfo regs_info[REG_MAX];
} BRC1RIOEndpt;

int brc1_rio_endpt_in(BRC1RIOEndpt *s, RIOTx *tx);

#endif // HW_RIO_BRC1_RIO_ENDPT
