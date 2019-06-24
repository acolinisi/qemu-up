#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "hw/register.h"
#include "qemu/bitops.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qapi/qmp/qerror.h"

#include "hw/fdt_generic_util.h"

#ifndef HPSC_RESET_CTRL_ERR_DEBUG
#define HPSC_RESET_CTRL_ERR_DEBUG 0
#endif

#define TYPE_HPSC_RESET_CTRL "hpsc,reset-ctrl"

#define HPSC_R52
#ifdef HPSC_R52
#include "hw/intc/arm_gicv3.h"
#endif
#define HPSC_RESET_CTRL(obj) \
     OBJECT_CHECK(HPSCResetCtrl, (obj), TYPE_HPSC_RESET_CTRL)

#define NUM_CPUS 2 /* TODO: make a DT property */
#define NUM_INTC 1 /* TODO: make a DT property */

/* The register interface exposed to SW separates the control over halt and
 * reset. This may change in the future (halt is a hardware detail that might
 * not need to be exposed to sofware). */
REG32(N_CPU_HALT, 0x0)
    FIELD(N_CPU_HALT, N_CPU_HALT, 0, NUM_CPUS)
REG32(CPU_RESET, 0x4)
    FIELD(CPU_RESET, CPU_RESET, 0, NUM_CPUS)
REG32(INTC_RESET, 0x8)
    FIELD(INTC_RESET, INTC_RESET, 0, NUM_INTC)

#define REG_MAX (R_INTC_RESET + 1)

typedef struct HPSCResetCtrl {
    SysBusDevice parent_obj;
    MemoryRegion iomem;

    /* Output control signals connected to inputs on the CPUs */
    qemu_irq ncpuhalt_irqs[NUM_CPUS];
    qemu_irq reset_irqs[NUM_CPUS];
    qemu_irq intc_reset_irqs[NUM_CPUS];

    /* Output signal that exposes CPU WFI states outside this reset controler */
    qemu_irq wfi_out[NUM_CPUS];

    uint32_t regs[REG_MAX];
    RegisterInfo regs_info[REG_MAX];
} HPSCResetCtrl;

static void handle_wfi(void *opaque, int irq, int level)
{
    HPSCResetCtrl *s = HPSC_RESET_CTRL(opaque);

    /* for now, simply forward */
    qemu_set_irq(s->wfi_out[irq], level);
}

static void hpsc_reset_halt_post_write(RegisterInfo *reg, uint64_t val)
{
    HPSCResetCtrl *s = HPSC_RESET_CTRL(reg->opaque);
    unsigned cpu_idx;

    for (cpu_idx = 0; cpu_idx < NUM_CPUS; ++cpu_idx) {
        /* invert polarity to match CPU interface */
        qemu_set_irq(s->ncpuhalt_irqs[cpu_idx], !(val & (1 << cpu_idx)));
    }
}

static void hpsc_reset_reset_post_write(RegisterInfo *reg, uint64_t val)
{
    HPSCResetCtrl *s = HPSC_RESET_CTRL(reg->opaque);
    unsigned cpu_idx;

    for (cpu_idx = 0; cpu_idx < NUM_CPUS; ++cpu_idx) {
        qemu_set_irq(s->reset_irqs[cpu_idx], val & (1 << cpu_idx));
    }
}

static void hpsc_reset_intc_reset_post_write(RegisterInfo *reg, uint64_t val)
{
    HPSCResetCtrl *s = HPSC_RESET_CTRL(reg->opaque);
    unsigned intc_idx;

    for (intc_idx = 0; intc_idx < NUM_CPUS; ++intc_idx) {
        qemu_set_irq(s->intc_reset_irqs[intc_idx], val & (1 << intc_idx));
    }
}

static uint64_t hpsc_reset_ctrl_read(void *opaque, hwaddr addr, unsigned size)
{
    HPSCResetCtrl *s = HPSC_RESET_CTRL(opaque);
    RegisterInfo *r = &s->regs_info[addr / 4];
    return register_read(r, ~0, NULL, false);
}

static void hpsc_reset_ctrl_write(void *opaque, hwaddr addr, uint64_t value,
                      unsigned size)
{
    HPSCResetCtrl *s = HPSC_RESET_CTRL(opaque);
    RegisterInfo *r = &s->regs_info[addr / 4];
    register_write(r, value, ~0, NULL, false);
}

static const RegisterAccessInfo hpsc_reset_ctrl_regs_info[] = {
    { .name = "N_CPU_HALT",  .addr = A_N_CPU_HALT,
        .reset = 0x0,
        .rsvd = 0xfffffffc, /* TODO: function of NUM_CPUS */
        .post_write = hpsc_reset_halt_post_write,
    },{
        .name = "CPU_RESET",  .addr = A_CPU_RESET,
        .reset = 0x0,
        .rsvd = 0xffffffff, /* TODO: function of NUM_CPUS */
        .post_write = hpsc_reset_reset_post_write,
    },{
        .name = "INTC_RESET",  .addr = A_INTC_RESET,
        .reset = 0x0,
        .rsvd = 0xffffffff, /* TODO: function of NUM_INTC */
        .post_write = hpsc_reset_intc_reset_post_write,
    }
};
static const MemoryRegionOps hpsc_reset_ctrl_ops = {
    .read = hpsc_reset_ctrl_read,
    .write = hpsc_reset_ctrl_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void hpsc_reset_ctrl_reset(DeviceState *dev)
{
    HPSCResetCtrl *s = HPSC_RESET_CTRL(dev);
    unsigned int i;

    for (i = 0; i < ARRAY_SIZE(s->regs_info); ++i) {
        register_reset(&s->regs_info[i]);
    }
}

static void hpsc_reset_ctrl_realize(DeviceState *dev, Error **errp)
{
    HPSCResetCtrl *s = HPSC_RESET_CTRL(dev);
    unsigned int i;

    for (i = 0; i < ARRAY_SIZE(hpsc_reset_ctrl_regs_info); ++i) {
        RegisterInfo *r = &s->regs_info[hpsc_reset_ctrl_regs_info[i].addr / 4];

        *r = (RegisterInfo) {
            .data = (uint8_t *)&s->regs[hpsc_reset_ctrl_regs_info[i].addr / 4],
            .data_size = sizeof(uint32_t),
            .access = &hpsc_reset_ctrl_regs_info[i],
            .opaque = s,
        };
    }
}

static void hpsc_reset_ctrl_init(Object *obj)
{
    HPSCResetCtrl *s = HPSC_RESET_CTRL(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &hpsc_reset_ctrl_ops,
                          s, TYPE_HPSC_RESET_CTRL, REG_MAX * 4);
    sysbus_init_mmio(sbd, &s->iomem);

    /* wfi_in is the signal output by the CPU when it enters WFI state */
    qdev_init_gpio_in_named(DEVICE(obj), handle_wfi, "wfi_in", NUM_CPUS);

    /* wfi_out exposes the WFI state of the CPUs out of the reset controller */
    qdev_init_gpio_out_named(DEVICE(obj), s->wfi_out, "wfi_out", NUM_CPUS);

    /* output control signals connected to the inputs on the CPUs */
    qdev_init_gpio_out_named(DEVICE(obj), s->ncpuhalt_irqs, "ncpuhalt", NUM_CPUS);
    qdev_init_gpio_out_named(DEVICE(obj), s->reset_irqs, "reset", NUM_CPUS);
    qdev_init_gpio_out_named(DEVICE(obj), s->intc_reset_irqs, "intc_reset", NUM_INTC);
}

static const VMStateDescription vmstate_hpsc_reset_ctrl = {
    .name = TYPE_HPSC_RESET_CTRL,
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, HPSCResetCtrl, REG_MAX),
        VMSTATE_END_OF_LIST(),
    }
};

static const FDTGenericGPIOSet hpsc_reset_ctrl_controller_gpios [] = {
    {
        .names = &fdt_generic_gpio_name_set_gpio,
        .gpios = (FDTGenericGPIOConnection [])  {
            { .name = "n_cpu_halt",           .fdt_index = 0 * NUM_CPUS, .range = NUM_CPUS },
            { .name = "cpu_reset",              .fdt_index = 1 * NUM_CPUS, .range = NUM_CPUS },
            { .name = "wfi_in",             .fdt_index = 2 * NUM_CPUS, .range = NUM_CPUS },
            { .name = "intc_reset",         .fdt_index = 3 * NUM_CPUS, .range = NUM_INTC },
            { },
        },
    },
    { },
};

static const FDTGenericGPIOSet hpsc_reset_ctrl_client_gpios [] = {
    {
        .names = &fdt_generic_gpio_name_set_gpio,
        .gpios = (FDTGenericGPIOConnection [])  {
            { .name = "wfi_out",            .fdt_index = 0, .range = NUM_CPUS },
            { },
        },
    },
    { },
};

static void hpsc_reset_ctrl_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    FDTGenericGPIOClass *fggc = FDT_GENERIC_GPIO_CLASS(klass);

    dc->reset = hpsc_reset_ctrl_reset;
    dc->realize = hpsc_reset_ctrl_realize;
    dc->vmsd = &vmstate_hpsc_reset_ctrl;
    fggc->controller_gpios = hpsc_reset_ctrl_controller_gpios;
    fggc->client_gpios = hpsc_reset_ctrl_client_gpios;
}

static const TypeInfo hpsc_reset_ctrl_info = {
    .name          = TYPE_HPSC_RESET_CTRL,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(HPSCResetCtrl),
    .class_init    = hpsc_reset_ctrl_class_init,
    .instance_init = hpsc_reset_ctrl_init,
    .interfaces    = (InterfaceInfo[]) {
        { TYPE_FDT_GENERIC_GPIO },
        { }
    },
};

static void hpsc_reset_ctrl_register_types(void)
{
    type_register_static(&hpsc_reset_ctrl_info);
}

type_init(hpsc_reset_ctrl_register_types)
