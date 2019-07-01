/*
 * Bridge between interfaces without TrustZone and with TrustZone.
 *
 * See description in the implementation source.
 */

#ifndef TZ_MPC_H
#define TZ_MPC_H

#include "hw/sysbus.h"

#define TYPE_TZ_BRIDGE "tz-bridge"
#define TYPE_TZ_BRIDGE_IOMMU_MEMORY_REGION "tz-bridge-iommu-memory-region"
#define TZ_BRIDGE(obj) OBJECT_CHECK(TzBridge, (obj), TYPE_TZ_BRIDGE)

typedef struct TzBridge {
    /*< private >*/
    SysBusDevice parent_obj;

    /*< public >*/

    IOMMUMemoryRegion upstream;
    AddressSpace downstream_as;

    /* Properties */
    MemoryRegion *downstream;
    bool out_secure;
} TzBridge;

#endif
