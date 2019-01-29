#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "hw/sysbus.h"
#include "hw/char/serial.h"

#define TYPE_SERIAL_SYSBUS "sysbus-serial"
#define SERIAL_SYSBUS(obj) \
        OBJECT_CHECK(SerialSysBusState, (obj), TYPE_SERIAL_SYSBUS)

#define REGISTERS_8BIT   0
#define REGISTERS_16BIT  1
#define REGISTERS_32BIT  2

#define REGISTER_SHIFT REGISTERS_32BIT

#define REGION_SIZE (8 << REGISTER_SHIFT)

typedef struct {
    SysBusDevice busdev;
    SerialState sdev;
    unsigned baudrate;
    qemu_irq irq;
} SerialSysBusState;

static const VMStateDescription vmstate_serial_sysbus = {
    .name = TYPE_SERIAL_SYSBUS,
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .fields      = (VMStateField[]) {
        VMSTATE_END_OF_LIST()
    }
};

static Property serial_sysbus_props[] = {
    DEFINE_PROP_CHR("chardev",      SerialSysBusState, sdev.chr),
    /* not setting sdev.baudbase directly because no macro for 'int' type */
    DEFINE_PROP_UINT32("baudrate",  SerialSysBusState, baudrate,      115200),
    DEFINE_PROP_END_OF_LIST()
};

static void serial_sysbus_init(Object *obj)
{
    SerialSysBusState *s = SERIAL_SYSBUS(obj);
    SerialState *sdev = &s->sdev;
    enum device_endian end = DEVICE_NATIVE_ENDIAN;

    sysbus_init_irq(SYS_BUS_DEVICE(s), &s->irq);

    memory_region_init_io(&sdev->io, NULL, &serial_mm_ops[DEVICE_NATIVE_ENDIAN],
                          sdev, "sysbus-serial", REGION_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &sdev->io);
}

static void serial_sysbus_realize(DeviceState *dev, Error **errp)
{
    SerialSysBusState *s = SERIAL_SYSBUS(dev);
    SerialState *sdev = &s->sdev;

    sdev->it_shift = REGISTER_SHIFT;
    sdev->baudbase = s->baudrate;

    serial_realize_core(sdev, errp);
}

static void serial_sysbus_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = serial_sysbus_realize;
    /* serial.h doesn't provide a reset method */
    dc->vmsd = &vmstate_serial_sysbus;
    dc->props = serial_sysbus_props;
}

static TypeInfo serial_sysbus_info = {
    .name          = TYPE_SERIAL_SYSBUS,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(SerialSysBusState),
    .class_init    = serial_sysbus_class_init,
    .instance_init = serial_sysbus_init,
};

static void serial_sysbus_register_types(void)
{
    type_register_static(&serial_sysbus_info);
}

type_init(serial_sysbus_register_types)
