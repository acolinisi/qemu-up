/*
 * QEMU model of Primcell PL35X family of memory controllers
 *
 * Copyright (c) 2012 Xilinx Inc.
 * Copyright (c) 2012 Peter Crosthwaite <peter.crosthwaite@xilinx.com>.
 * Copyright (c) 2011 Edgar E. Iglesias.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "qemu/osdep.h"
#include "hw/hw.h"
#include "hw/block/flash.h"
#include "qapi/error.h"
#include "qemu/timer.h"
#include "sysemu/block-backend.h"
#include "exec/address-spaces.h"
#include "qemu/host-utils.h"
#include "hw/sysbus.h"

#include "sysemu/blockdev.h"

#include "hw/block/offchip_sram.h"

#define ECC_BYTES_PER_SUBPAGE 3
#define ECC_CODEWORD_SIZE 512

#define PL35x_REG_PERIPH_ID         0xFE0

static const uint32_t pl35x_id[] = {
    0x53, 0x13, 0x64, 0x00, 0x0D, 0xF0, 0x05, 0xB1 /* SMC-353 r2p2 */
};

#ifdef PL35X_ERR_DEBUG
#define DB_PRINT(...) do { \
    fprintf(stderr,  ": %s: ", __func__); \
    fprintf(stderr, ## __VA_ARGS__); \
    } while (0);
#else
    #define DB_PRINT(...)
#endif

#define TYPE_PL35X "arm.pl35x"

#define PL35X(obj) \
     OBJECT_CHECK(PL35xState, (obj), TYPE_PL35X)

typedef struct PL35xItf {
    MemoryRegion mm;
    DeviceState *dev;
    uint8_t nand_pending_addr_cycles;
    uint8_t ecc_digest[16 * 1024];
    uint8_t ecc_oob[16 * 1024];
    uint32_t ecc_pos, ecc_subpage_offset;
    void * parent;
    uint32_t ecc_r_data_size;
    uint32_t ecc_w_data_size;
    bool r_new_ecc;
    bool w_new_ecc;
    uint32_t buf_count;
    uint8_t buff[2048];
} PL35xItf;

typedef struct PL35xState {
    SysBusDevice parent_obj;

    MemoryRegion mmio;

    /* FIXME: add support for multiple chip selects/interface */

    PL35xItf itf[2];

    /* FIXME: add Interrupt support */

    uint32_t regs[0x450 >> 2];

    uint8_t x; /* the "x" in pl35x */
} PL35xState;

static int pl35x_device_list_sram(Object * obj, void *opaque)
{
    GSList **list = opaque;

    if (object_dynamic_cast(obj, "offchip-sram")) {
        *list = g_slist_append(*list, DEVICE(obj));
    }
    object_child_foreach(obj, pl35x_device_list_sram, opaque);
    return 0;
}

static GSList *pl35x_get_device_list_sram(Object * obj)
{
    GSList *list = NULL;

    object_child_foreach(obj, pl35x_device_list_sram, &list);
    return list;
}

static int pl35x_device_list_nand(Object * obj, void *opaque)
{
    GSList **list = opaque;

    if (object_dynamic_cast(obj, "nand")) {
        *list = g_slist_append(*list, DEVICE(obj));
    }
    object_child_foreach(obj, pl35x_device_list_nand, opaque);
    return 0;
}

static GSList *pl35x_get_device_list_nand(Object * obj)
{
    GSList *list = NULL;

    object_child_foreach(obj, pl35x_device_list_nand, &list);
    return list;
}

static void pl35x_ecc_init(PL35xItf *s)
{
    /* FIXME: Bad performance */
    memset(s->ecc_digest, 0xFF, 16 * 1024);
    memset(s->ecc_oob, 0xFF, 16 * 1024);
    s->ecc_pos = 0;
    s->ecc_subpage_offset = 0;
}

static void pl35x_ecc_save(PL35xItf *s) {
    PL35xState * ps = s->parent;
    int i, j, shift;

    for (i = 0; i < 4 ; i++) {
        uint32_t r32 = (0x40 << 24);	// always ecc is correct
        for (j = 0, shift = 0; j < ECC_BYTES_PER_SUBPAGE; j++) {
            uint8_t r8 = s->ecc_digest[i*ECC_BYTES_PER_SUBPAGE+j];
            r32 |= (~r8 & 0xff) << shift;	// inverse the bits
            shift += 8;
        }
        int ecc1_block_idx = 0x418 + (i << 2);
        ps->regs[ecc1_block_idx >> 2] = r32;
    }
    DB_PRINT("ECC: @0x418 = 0x%08x, @0x41c = 0x%08x, @0x420 = 0x%08x, @0x424 = 0x%08x\n",
       ps->regs[0x418 >>2], ps->regs[0x41c >>2], ps->regs[0x420 >>2], ps->regs[0x424 >>2]);
}

static void pl35x_ecc_digest(PL35xItf *s, uint8_t data)
{
    uint32_t ecc_bytes_per_subpage = ECC_BYTES_PER_SUBPAGE;

    s->ecc_digest[s->ecc_pos++] ^= ~data;
    if (!(s->ecc_pos % ecc_bytes_per_subpage)) {
        s->ecc_pos -= ecc_bytes_per_subpage;
    }

    s->ecc_subpage_offset++;
    if (s->ecc_subpage_offset == ECC_CODEWORD_SIZE) {
        s->ecc_subpage_offset = 0;
        do {
            s->ecc_pos++;
        } while (s->ecc_pos % ecc_bytes_per_subpage);
    }
}

static uint64_t pl35x_read(void *opaque, hwaddr addr,
                         unsigned int size)
{
    PL35xState *s = opaque;
    uint32_t r = 0;

    if (addr >= PL35x_REG_PERIPH_ID && addr < PL35x_REG_PERIPH_ID + 32) {
        return pl35x_id[(addr - PL35x_REG_PERIPH_ID) >> 2];
    }

    switch (addr) {
    case 0x0:
      {
        int rdy;
        if (s->itf[0].dev && object_dynamic_cast(OBJECT(s->itf[0].dev),
                                                      "nand")) {
            nand_getpins(s->itf[0].dev, &rdy);
            r |= (!!rdy) << 5;
        }
        if (s->itf[1].dev && object_dynamic_cast(OBJECT(s->itf[1].dev),
                                                      "nand")) {
            nand_getpins(s->itf[1].dev, &rdy);
            r |= (!!rdy) << 6;
        }
        }
        break;

    case 0x4:
    case 0x20:
    case 0x24:
         r = s->regs[addr >> 2];
    case 0x100:
    case 0x120:
    case 0x140:
    case 0x160:
    case 0x180:
    case 0x1a0:
    case 0x1c0:
    case 0x1e0: {
         r = s->regs[addr >> 2];
	break;
    }
    case 0x300:
    case 0x304:
    case 0x308:
    case 0x30c:
    case 0x310:
    case 0x314:
    case 0x318:
    case 0x31c:
    case 0x320:
    case 0x324:
    case 0x328:
    case 0x400:
    case 0x404:
    case 0x408:
    case 0x40c:
    case 0x410:
    case 0x414:
        r = s->regs[addr >> 2];
        break;
    case 0x418:
    case 0x41c:
    case 0x420:
    case 0x424:
        r = s->regs[addr >> 2];
        break;
    case 0x428:
        r = s->regs[addr >> 2];
	break;
    default:
        DB_PRINT("Unimplemented SMC read access reg=" TARGET_FMT_plx "\n",
                 addr);
        break;
    }
    DB_PRINT("=== addr = 0x%lx val = 0x%x\n", addr, r);
    return r;
}

static void pl35x_write(void *opaque, hwaddr addr, uint64_t value64,
                      unsigned int size)
{
    DB_PRINT("=== addr = 0x%x v = 0x%x\n", (unsigned)addr, (unsigned)value64);
    PL35xState *s = opaque;
    switch (addr) {
    case 0x8:
    case 0xc:
    case 0x10:
    case 0x14:
    case 0x18:
    case 0x1c:
    case 0x20:
    case 0x24:
        s->regs[addr >> 2] = value64;
        break;
    case 0x100:
    case 0x120:
    case 0x140:
    case 0x160:
    case 0x180:
    case 0x1a0:
    case 0x1c0:
    case 0x1e0: {
        s->regs[addr >> 2] = value64;
	break;
    }
    case 0x300:
    case 0x304:
    case 0x308:
    case 0x30c:
    case 0x310:
    case 0x314:
    case 0x318:
    case 0x31c:
    case 0x320:
    case 0x324:
    case 0x328:
    case 0x400:
    case 0x404:
    case 0x408:
    case 0x40c:
    case 0x410:
    case 0x414:
    case 0x418:
    case 0x41c:
    case 0x420:
    case 0x424:
    case 0x428: {
        s->regs[addr >> 2] = value64;
	break;
    }
    default:
        DB_PRINT("Unimplemented SMC read access reg=" TARGET_FMT_plx "\n",
                 addr);
        break;
    }
}

static const MemoryRegionOps pl35x_ops = {
    .read = pl35x_read,
    .write = pl35x_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4
    }
};

static uint64_t nand_read(void *opaque, hwaddr addr,
                           unsigned int size)
{
    PL35xItf *s = opaque;
    unsigned int len = size;
    int i;
    int shift = 0;
    uint32_t r = 0;
    bool data_phase;
    uint16_t end_cmd;

    /* No nand device present */
    if (s->dev == NULL)
        return 0x0;

    data_phase = (addr >> 19) & 1;
    end_cmd = (addr >> 11) & 0xff;

    int page_size = nand_page_size(s->dev);
    int nand_remain_data = nand_iolen(s->dev);
    if (nand_remain_data > 0 && nand_remain_data < 4 && size >= 4) {
	/* previous read_byte didn't clean the buffer. flush it. */
        while(nand_remain_data--) {
            nand_getio(s->dev);
        }
    }
    while (len--) {
        uint8_t r8;

        r8 = nand_getio(s->dev) & 0xff;
        r |= r8 << shift;
        shift += 8;
    }
    if (data_phase && end_cmd != 0) {
        if (s->r_new_ecc) {
            pl35x_ecc_init(s);
            s->ecc_r_data_size = 0;
            s->r_new_ecc = false;
            s->buf_count = 0;
        }
        s->ecc_r_data_size += size;
        for (i = 0; i < size; i++) {
            s->buff[s->buf_count++] = (uint8_t) (r >> (i * 8) & 0xff);
            pl35x_ecc_digest(s, (uint8_t) (r >> (i * 8) & 0xff));
        }
        if (s->ecc_r_data_size == page_size) {	/* assume PAGE_SIZE = 2048 */
            /* save ecc value to the registers */
            pl35x_ecc_save(s);
	    s->r_new_ecc = true;
        }
    }
    return r;
}

static void nand_write(void *opaque, hwaddr addr, uint64_t value64,
                       unsigned int size)
{
    struct PL35xItf *s = opaque;
    bool data_phase, ecmd_valid;
    unsigned int addr_cycles = 0;
    uint16_t start_cmd, end_cmd;
    uint32_t value = value64;
    uint32_t nandaddr = value;

    /* No nand device present */
    if (s->dev == NULL)
        return;

    /* Decode the various signals.  */
    data_phase = (addr >> 19) & 1;
    ecmd_valid = (addr >> 20) & 1;
    start_cmd = (addr >> 3) & 0xff;
    end_cmd = (addr >> 11) & 0xff;

    DB_PRINT("addr=%x v32=%x v64=%lx size=%d, data_phase(0x%x), "
	     "ecmd_vali(0x%x), start_cmd(0x%x), end_cmd(0x%x)\n",
	     (unsigned)addr, (unsigned)value, value64, size, data_phase,
	     ecmd_valid, start_cmd, end_cmd);

    if (!data_phase) {
        addr_cycles = (addr >> 21) & 7;
    }

    if (!data_phase) {
        DB_PRINT("start_cmd=%x end_cmd=%x (valid=%d) acycl=%d\n",
                start_cmd, end_cmd, ecmd_valid, addr_cycles);
        switch(start_cmd) {
            case 0x80 : /* NAND_CMD_PAGEPROGRAM1 */
                s->w_new_ecc = true;
                DB_PRINT("New write ECC starts\n");
                pl35x_ecc_init(s);
                s->ecc_w_data_size = 0;
                break;
            default:
                break;
        }
    }

    /* Writing data to the NAND.  */
    if (data_phase) {
    DB_PRINT("data_phase: writing data to NAND\n");
        nand_setpins(s->dev, 0, 0, 0, 1, 0);
        while (size--) {
            nand_setio(s->dev, value & 0xff);
            if (s->w_new_ecc) {
		assert(s->ecc_w_data_size <= nand_page_size(s->dev));
                pl35x_ecc_digest(s, (uint8_t) (value & 0xff));
                s->ecc_w_data_size++;
            }
            value >>= 8;
        }
        if (s->w_new_ecc && s->ecc_w_data_size == nand_page_size(s->dev)) { /* save ecc */
            DB_PRINT("ECC is saved\n");
            pl35x_ecc_save(s);
            s->w_new_ecc = false;
        }
    }

    /* Writing Start cmd.  */
    if (!data_phase && !s->nand_pending_addr_cycles) {
    DB_PRINT("writing Start cmd \n");
        nand_setpins(s->dev, 1, 0, 0, 1, 0);
        nand_setio(s->dev, start_cmd);
    }
    else {
        DB_PRINT("Start data phase or pending address cycles\n");
    }
    if (!addr_cycles) {
        s->nand_pending_addr_cycles = 0;
    }
    if (s->nand_pending_addr_cycles) {
        addr_cycles = s->nand_pending_addr_cycles;
        s->nand_pending_addr_cycles = 0;
    }
    if (addr_cycles > 4) {
        s->nand_pending_addr_cycles = addr_cycles - 4;
        addr_cycles = 4;
    }
    while (addr_cycles) {
        nand_setpins(s->dev, 0, 1, 0, 1, 0);
        DB_PRINT("nand cycl=%d addr=%x\n", addr_cycles, nandaddr & 0xff);
        nand_setio(s->dev, nandaddr & 0xff);
        nandaddr >>= 8;
        addr_cycles--;
    }
    /* Writing commands. One or two (Start and End).  */
    if (ecmd_valid && !s->nand_pending_addr_cycles) {
        s->r_new_ecc = true;
        DB_PRINT("writing commands. One or two (Start and End)\n");
        nand_setpins(s->dev, 1, 0, 0, 1, 0);
        nand_setio(s->dev, end_cmd);
    }
}

static const MemoryRegionOps nand_ops = {
    .read = nand_read,
    .write = nand_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4
    }
};

static void pl35x_reset(DeviceState *dev)
{
    PL35xState *s = PL35X(dev);
    /* if child for sram exists */
    GSList * list = pl35x_get_device_list_sram(OBJECT(dev));
    if (list) { /* sram exists */
        if (s->itf[0].dev == NULL) {
            s->itf[0].dev = DEVICE(list->data);
            uint32_t high_addr = object_property_get_int(OBJECT(list->data),
                                        "start_addr_high", NULL);
            uint32_t low_addr = object_property_get_int(OBJECT(list->data),
                                        "start_addr_low", NULL);
            uint32_t region_size = object_property_get_int(OBJECT(list->data),
                                        "region_size", NULL);
            memory_region_init_io(&s->itf[0].mm, OBJECT(list->data),
                                  &offchip_sram_ops, OBJECT(list->data),
                                  "pl35x.sram", region_size);
            if (s->itf[0].mm.container == NULL) { /* SRAM */
	        uint64_t address = (uint64_t) high_addr << 32 | low_addr;
                memory_region_add_subregion(s->mmio.container, address, &s->itf[0].mm);
            }
        }
    }
    /* if child for nand exists */
    list = pl35x_get_device_list_nand(OBJECT(dev));
    if (list) { /* nand exists */
        if (s->itf[1].dev == NULL) {
            s->itf[1].dev = DEVICE(list->data);
            uint32_t high_addr = object_property_get_int(OBJECT(list->data),
                                        "start_addr_high", NULL);
            uint32_t low_addr = object_property_get_int(OBJECT(list->data),
                                        "start_addr_low", NULL);
            uint32_t region_size = object_property_get_int(OBJECT(list->data),
                                        "region_size", NULL);
            memory_region_init_io(&s->itf[1].mm, OBJECT(dev), &nand_ops,
                                  &s->itf[1], "pl35x.nand", region_size);
            sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->itf[1].mm);
            if (!s->itf[1].mm.container) { /* NAND */
	        uint64_t address = (uint64_t) high_addr << 32 | low_addr;
                memory_region_add_subregion(s->mmio.container, address, &s->itf[1].mm);
            }
        }
        s->itf[1].ecc_r_data_size = 0;
        s->itf[1].ecc_w_data_size = 0;
        s->itf[1].r_new_ecc = false;
        s->itf[1].w_new_ecc = false;
        pl35x_ecc_init(&s->itf[1]);
    }
}

static void pl35x_init_sram(SysBusDevice *dev, PL35xItf *itf)
{
    /* all of the functionalities are moved to pl35x_reset() */
    return;
}

static void pl35x_init_nand(SysBusDevice *dev, PL35xItf *itf)
{
    /* all of the functionalities are moved to pl35x_reset() */
    return;
}

static void pl35x_init(Object *obj)
{
    SysBusDevice *dev = SYS_BUS_DEVICE(obj);
    PL35xState *s = PL35X(obj);

    memory_region_init_io(&s->mmio, OBJECT(dev), &pl35x_ops, s, "pl35x_io",
                          0x1000);
    sysbus_init_mmio(dev, &s->mmio);
    if (s->x == 1) { /* PL351 have NAND */
        pl35x_init_nand(dev, &s->itf[0]);
    }
    if (s->x == 0x2) {
        pl35x_init_sram(dev, &s->itf[0]);
    }
    if (s->x == 0x3) { /* PL353 have SRAM and NAND */
        pl35x_init_nand(dev, &s->itf[1]);
        pl35x_init_sram(dev, &s->itf[0]);
    } else if (s->x == 4) { /* PL354 has a second SRAM */
        pl35x_init_sram(dev, &s->itf[0]);
        pl35x_init_sram(dev, &s->itf[1]);
    }
    s->itf[0].parent = s;
    s->itf[1].parent = s;

    object_property_add_link(obj, "dev0", TYPE_DEVICE,
                             (Object **)&s->itf[0].dev,
                             object_property_allow_set_link,
                             OBJ_PROP_LINK_STRONG,
                             &error_abort);
    object_property_add_link(obj, "dev1", TYPE_DEVICE,
                             (Object **)&s->itf[1].dev,
                             object_property_allow_set_link,
                             OBJ_PROP_LINK_STRONG,
                             &error_abort);
}

static Property pl35x_properties[] = {
    DEFINE_PROP_UINT8("x", PL35xState, x, 3),
    DEFINE_PROP_END_OF_LIST(),
};

static const VMStateDescription vmstate_pl35x = {
    .name = "pl35x",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT8(itf[0].nand_pending_addr_cycles, PL35xState),
        VMSTATE_UINT8(itf[1].nand_pending_addr_cycles, PL35xState),
        VMSTATE_END_OF_LIST()
    }
};

static void pl35x_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = pl35x_reset;
    dc->props = pl35x_properties;
    dc->vmsd = &vmstate_pl35x;
}

static TypeInfo pl35x_info = {
    .name           = TYPE_PL35X,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(PL35xState),
    .class_init     = pl35x_class_init,
    .instance_init  = pl35x_init,
};

static void pl35x_register_types(void)
{
    type_register_static(&pl35x_info);
}

type_init(pl35x_register_types)
