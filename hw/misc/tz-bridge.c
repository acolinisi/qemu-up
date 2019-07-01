/*
 * Bridge between interfaces without TrustZone and with TrustZone.
 *
 * This device modifies memory transactions that arrive at its input interface
 * and sends the modified transaction to its output interface. The device
 * modifies the secure flag in the transaction to the value given at the
 * time this device is instantiated (via a property).
 *
 * For example, assume an SoC has a CPU without TrustZone support that can only
 * output non-secure transactions (e.g. ARM CPUs without TrustZone suppork like
 * Cortex-M4, Cortex-R52) but is authorized to access resources that do support
 * TrustZone and are in the secure domain. Insert this device between the CPU
 * output interface and the interconnect's input interface and set the security
 * property of this device to true. Then, non-secure transactions that
 * originate ffrom that CPU without TrustZone will be converted into secure
 * transactions.
 */

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "hw/fdt_generic_util.h"

#include "hw/misc/tz-bridge.h"

#ifndef TZ_BRIDGE_ERR_DEBUG
#define TZ_BRIDGE_ERR_DEBUG 0
#endif

static int tz_bridge_attrs_to_index(IOMMUMemoryRegion *iommu, MemTxAttrs attrs)
{
    return 0;
}

static IOMMUTLBEntry tz_bridge_translate_with_attrs(IOMMUMemoryRegion *mr,
        hwaddr addr, IOMMUAccessFlags flag, int iommu_idx, MemTxAttrs *attrs)
{
    TzBridge *s = TZ_BRIDGE(container_of(mr, TzBridge, upstream));

    attrs->secure = s->out_secure;

    IOMMUTLBEntry iotlb = {
        .iova = addr,
        .translated_addr = addr,
        //.addr_mask = (1ULL << 12) - 1,
        .addr_mask = 0xffffffff,
        .perm = IOMMU_RW, /* TODO: can we get this from attr? */
        .target_as = &s->downstream_as,
    };
    return iotlb;
}

static IOMMUTLBEntry tz_bridge_translate(IOMMUMemoryRegion *mr,
        hwaddr addr, IOMMUAccessFlags flag, int iommu_idx)
{
    TzBridge *s = TZ_BRIDGE(container_of(mr, TzBridge, upstream));
    IOMMUTLBEntry iotlb = {
        .iova = addr,
        .translated_addr = addr,
        //.addr_mask = (1ULL << 12) - 1,
        .addr_mask = 0xffffffff,
        .perm = IOMMU_RW, /* TODO: can we get this from attr? */
        .target_as = &s->downstream_as,
    };
    return iotlb;
}

static void tz_bridge_init(Object *obj)
{
    TzBridge *s = TZ_BRIDGE(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_iommu(&s->upstream, sizeof(s->upstream),
                             TYPE_TZ_BRIDGE_IOMMU_MEMORY_REGION,
                             OBJECT(s), "tz-bridge-upstream",
                            //UINT64_MAX);
                            UINT32_MAX);
    sysbus_init_mmio(sbd, MEMORY_REGION(&s->upstream));
}

static void tz_bridge_realize(DeviceState *dev, Error **errp)
{
    TzBridge *s = TZ_BRIDGE(dev);

    address_space_init(&s->downstream_as, s->downstream, NULL);
}

static void tz_bridge_iommu_memory_region_class_init(ObjectClass *klass,
                                                   void *data)
{
    IOMMUMemoryRegionClass *imrc = IOMMU_MEMORY_REGION_CLASS(klass);

    imrc->attrs_to_index = tz_bridge_attrs_to_index;
    imrc->translate = tz_bridge_translate;
    imrc->translate_with_attrs = tz_bridge_translate_with_attrs;
}

static Property tz_bridge_properties[] = {
    DEFINE_PROP_LINK("downstream", TzBridge, downstream,
                     TYPE_MEMORY_REGION, MemoryRegion *),
    DEFINE_PROP_BOOL("out-secure", TzBridge, out_secure, false),
    DEFINE_PROP_END_OF_LIST(),
};

static void tz_bridge_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->props = tz_bridge_properties;
    dc->realize = tz_bridge_realize;
}

static const TypeInfo tz_bridge_info = {
    .name          = TYPE_TZ_BRIDGE,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(TzBridge),
    .class_init    = tz_bridge_class_init,
    .instance_init = tz_bridge_init,
};

static const TypeInfo tz_bridge_iommu_memory_region_info = {
    .name = TYPE_TZ_BRIDGE_IOMMU_MEMORY_REGION,
    .parent = TYPE_IOMMU_MEMORY_REGION,
    .class_init = tz_bridge_iommu_memory_region_class_init,
};

static void tz_bridge_register_types(void)
{
    type_register_static(&tz_bridge_info);
    type_register_static(&tz_bridge_iommu_memory_region_info);
}

type_init(tz_bridge_register_types)
