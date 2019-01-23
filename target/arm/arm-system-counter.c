#include "hw/arm/arm-system-counter.h"

static const TypeInfo arm_system_counter_info[] = {
    {
        .name       = TYPE_ARM_SYSTEM_COUNTER,
        .parent     = TYPE_INTERFACE,
        .class_size = sizeof(ARMSystemCounterClass),
    },
    {
        .name       = TYPE_ARM_SYSTEM_COUNTER_EVENT,
        .parent     = TYPE_OBJECT,
    },
};

DEFINE_TYPES(arm_system_counter_info)
