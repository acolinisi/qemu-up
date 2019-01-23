#ifndef HW_ARM_ARM_SYSTEM_COUNTER
#define HW_ARM_ARM_SYSTEM_COUNTER

#include "qemu/osdep.h"
#include "qom/object.h"
#include "qemu-common.h"

#define TYPE_ARM_SYSTEM_COUNTER         "arm-system-counter"
#define TYPE_ARM_SYSTEM_COUNTER_EVENT   "arm-system-counter-event"

#define ARM_SYSTEM_COUNTER_CLASS(klass) \
     OBJECT_CLASS_CHECK(ARMSystemCounterClass, (klass), TYPE_ARM_SYSTEM_COUNTER)
#define ARM_SYSTEM_COUNTER_GET_CLASS(obj) \
    OBJECT_GET_CLASS(ARMSystemCounterClass, (obj), TYPE_ARM_SYSTEM_COUNTER)
#define ARM_SYSTEM_COUNTER(obj) \
     INTERFACE_CHECK(ARMSystemCounter, (obj), TYPE_ARM_SYSTEM_COUNTER)
#define ARM_SYSTEM_COUNTER_EVENT(obj) \
     OBJECT_CHECK(ARMSystemCounterEvent, (obj), TYPE_ARM_SYSTEM_COUNTER_EVENT)

typedef void (ARMSystemCounterEventCb)(void *arg);

typedef struct ARMSystemCounter { /* This is an interface */
    /*< private >*/
    Object parent_obj;
} ARMSystemCounter;

typedef struct ARMSystemCounterEvent { /* This is an object */
    /*< private >*/
    Object parent_obj;

    ARMSystemCounter *sc;
} ARMSystemCounterEvent;

typedef struct ARMSystemCounterClass {
    /*< private >*/
    InterfaceClass parent_class;

    /* Maximum value the counter can take (< 2^64 - 1 when not full width) */
    uint64_t (*max_count)(ARMSystemCounter *s);

    /* The maximum delta by which the counter increments on each clock cycle.
     * The maximum is w.r.t. different clock frequencies supported.
     * This is > 1 for counters that automatically scale in order to maintain
     * fixed units regardless of the clock frequency. */
    unsigned (*max_delta)(ARMSystemCounter *s);

    uint64_t (*count)(ARMSystemCounter *s);

    ARMSystemCounterEvent * (*event_create)(ARMSystemCounter *asc,
                                        ARMSystemCounterEventCb *cb, void *arg);
    void (*event_destroy)(ARMSystemCounterEvent *e);

    void (*event_schedule)(ARMSystemCounterEvent *e, uint64_t time);
    void (*event_cancel)(ARMSystemCounterEvent *e);
} ARMSystemCounterClass;

#endif // HW_ARM_ARM_SYSTEM_COUNTER
