#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "hw/register-dep.h"
#include "qemu/bitops.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "qemu/main-loop.h"
#include "hw/ptimer.h"
#include "hw/fdt_generic_util.h"

#ifndef HPSC_WDT_TIMER_ERR_DEBUG
#define HPSC_WDT_TIMER_ERR_DEBUG 0
#endif

#define TYPE_HPSC_WDT_TIMER "hpsc,hpsc-wdt"

#define DB_PRINT_L(lvl, fmt, args...) do {\
    if (HPSC_WDT_TIMER_ERR_DEBUG >= lvl) {\
        qemu_log(TYPE_HPSC_WDT_TIMER ": %s:" fmt, __func__, ## args);\
    } \
} while (0);

#define DB_PRINT(fmt, args...) DB_PRINT_L(1, fmt, ## args)

#define GPIO_NAME_LAST_TIMEOUT "LAST_TIMEOUT"

#define HPSC_WDT_TIMER(obj) \
     OBJECT_CHECK(HPSCWDTimer, (obj), TYPE_HPSC_WDT_TIMER)

#define CONCAT2_(x, y) x ## y
#define CONCAT2(x, y) CONCAT2_(x, y)
#define CONCAT4(x, y, v, w) CONCAT2(CONCAT2(x, y), CONCAT2(v, w))
#define CONCAT5(x, y, z, v, w) CONCAT2(CONCAT4(x, y, z, v), w)

// Implements variant B (aka. Concept B) of the spec, where there is a separate
// counter per stage. In Concept A, there is a single counter and a compare
// register per stage. We can model both concepts with the the model impelementation
// that uses a separate backend ptimer per stage. For Concept A, we would need
// to need to change the (now single) count register to return the sum of all
// backend ptimers, and similarly change the terminal registers to return the
// cummulative terminal values from all backend ptimers.

// NOTE: The following declarations of registers and the associate register
// info structure assume two stages, but the rest of the code is generic
// for any number of stages defined by NUM_STAGES.

#define NUM_STAGES 2

#if NUM_STAGES <= 0 || NUM_STAGES >= 31 // only so many unused bits in the status register
#error HPSC WDT Timer cannot support the given number of stages
#endif // NUM_STAGES

// TODO: is there support for 64-bit registers? to avoid HI,LO

#define DEP_ST_REG_(reg, addr) DEP_REG32(reg, addr)
#define DEP_ST_REG(reg, stage, addr) DEP_ST_REG_(CONCAT4(REG_ST, stage, _, reg), addr)

#define REGS_STAGE(stage) \
    DEP_ST_REG(TERMINAL_LO, stage, stage * STAGE_REGS_SIZE + 0x00) \
    DEP_ST_REG(TERMINAL_HI, stage, stage * STAGE_REGS_SIZE + 0x04) \
    DEP_ST_REG(COUNT_LO,    stage, stage * STAGE_REGS_SIZE + 0x08) \
    DEP_ST_REG(COUNT_HI,    stage, stage * STAGE_REGS_SIZE + 0x0c) \

#define NUM_STAGE_REGS 4
#define STAGE_REGS_SIZE (NUM_STAGE_REGS * 4) // register space size per stage

#if NUM_STAGES >= 1
REGS_STAGE(0)
#endif
#if NUM_STAGES >= 2
REGS_STAGE(1)
#endif
#if NUM_STAGES >= 3
REGS_STAGE(2)
#endif
#if NUM_STAGES >= 4
REGS_STAGE(3)
#endif
#if NUM_STAGES >= 5
#error Manual declarations not setup for given number of stages, can add them.
#endif // NUM_STAGES

#define GLOBAL_FRAME (NUM_STAGES * STAGE_REGS_SIZE)

DEP_REG32(REG_CONFIG, GLOBAL_FRAME + 0x0)
    DEP_FIELD(REG_CONFIG, EN, 1, 0)
    DEP_FIELD(REG_CONFIG, TICKDIV, 8, 2) // 8-bits: max divider 1GHz/3,906,250Hz = 256
DEP_REG32(REG_STATUS, GLOBAL_FRAME + 0x04)
    DEP_FIELD(REG_STATUS, TIMEOUT, NUM_STAGES, 0)
    DEP_FIELD(REG_STATUS, DBGDIS, 1, 31)

DEP_REG32(REG_CMD_ARM,  GLOBAL_FRAME + 0x08)
DEP_REG32(REG_CMD_FIRE, GLOBAL_FRAME + 0x0c)

#define R_MAX (R_REG_CMD_FIRE + 1)

#define CLK_FREQ_HZ 3906250 // TODO: take from DT, either via a ref to a clk node, or a value

// Note: with concept B, choose a width such that, SW can add counts from stages without overflow,
//       i.e. at most (64 - (NUM_STAGES - 1))
//       with concept A, this can be <=64, since SW never needs to add stages together
// Note: there is an additional limitation from the Qemu timer.h/ptimer.h backend: period[ns] * count
//       must not overflow a int64_t. Period is at most 1/CLK_FREQ_HZ * 10^9 ns (= 256, i.e. 8 bits),
//       so count must be 8 bits smaller (and another -1 bit due to int64_t instead of uint64_t).
#define COUNTER_WIDTH (64 - (NUM_STAGES - 1) - 8 - 1)

#define PTIMER_MDOE_ONE_SHOT 1

typedef enum {
    SCMD_INVALID = 0,
    SCMD_CAPTURE,
    SCMD_LOAD,
    SCMD_CLEAR,
} stage_cmd_t;

typedef enum {
    CMD_INVALID = 0,
    CMD_DISABLE, // TODO: not explicitly listed in spec, but implied
} cmd_t;

#if NUM_STAGES <= 2
// TODO: Codes from spec, once spec has them
#define CMD_CAPTURE_ST0_ARM  0xCD01
#define CMD_CAPTURE_ST1_ARM  0xCD02
#define CMD_LOAD_ST0_ARM     0xCD03
#define CMD_LOAD_ST1_ARM     0xCD04
#define CMD_CLEAR_ST0_ARM    0xCD05
#define CMD_CLEAR_ST1_ARM    0xCD06
#define CMD_DISABLE_ARM      0xCD07

#define CMD_CAPTURE_ST0_FIRE 0x01CD
#define CMD_CAPTURE_ST1_FIRE 0x02CD
#define CMD_LOAD_ST0_FIRE    0x03CD
#define CMD_LOAD_ST1_FIRE    0x04CD
#define CMD_CLEAR_ST0_FIRE   0x05CD
#define CMD_CLEAR_ST1_FIRE   0x06CD
#define CMD_DISABLE_FIRE     0x07CD
#else // NUM_STAGES
#error ARM-FIRE command codes not defined for given number of stages
#endif // NUM_STAGES

#define STAGE_CMD_CODE(cmd, stage, op) CMD_ ## cmd ## _ST ## stage ## _ ## op

#define PTIMER_MODE_ONE_SHOT 1

struct HPSCWDTimer;

typedef struct {
    struct HPSCWDTimer *timer;
    unsigned stage;
} stage_ctx_t;

typedef struct HPSCWDTimer {
    SysBusDevice parent_obj;
    MemoryRegion iomem;

    QEMUBH *bhs[NUM_STAGES];
    ptimer_state *ptimers[NUM_STAGES];
    stage_ctx_t stage_ctxs[NUM_STAGES];
    
    qemu_irq irqs[NUM_STAGES];
    uint64_t terminals[NUM_STAGES];

    // same as irqs[NUM_STAGES-1] but as GPIO, for driving RESET input on CPU
    qemu_irq last_timeout_gpio;

    bool enabled;
    uint64_t tick_offset;

    uint32_t regs[R_MAX];
    DepRegisterInfo regs_info[R_MAX];
} HPSCWDTimer;

static void update_irq(HPSCWDTimer *s, unsigned stage, bool set)
{
    assert(stage < R_REG_STATUS_TIMEOUT_LENGTH);
    uint32_t status_bit = 1 << (R_REG_STATUS_TIMEOUT_SHIFT + stage);
    if (set)
        s->regs[R_REG_STATUS] |= status_bit;
    else
        s->regs[R_REG_STATUS] &= ~status_bit;
    qemu_set_irq(s->irqs[stage], set);

    if (stage == NUM_STAGES - 1)
        qemu_set_irq(s->last_timeout_gpio, set);
}

static uint64_t get_staging_terminal(HPSCWDTimer *s, unsigned stage) {
    uint64_t terminal;
    unsigned reg_offset = stage * NUM_STAGE_REGS;
    terminal = ((uint64_t)(s->regs[reg_offset + R_REG_ST0_TERMINAL_HI]) << 32) |
        (s->regs[reg_offset + R_REG_ST0_TERMINAL_LO]);
    return terminal;
}

static void set_enabled_state(HPSCWDTimer *s, bool enabled)
{
    s->enabled = enabled;
    if (enabled)
        s->regs[R_REG_CONFIG] |= R_REG_CONFIG_EN_MASK;
    else
        s->regs[R_REG_CONFIG] &= ~R_REG_CONFIG_EN_MASK;
}

static void timer_reload(HPSCWDTimer *s, unsigned stage)
{
    unsigned st;
    DB_PRINT("%s: stage %u: reload count <- %lx\n",
            object_get_canonical_path(OBJECT(s)),
            stage, s->terminals[stage]);

    // Stop downstream stages. The stopping has to be the responsibility of HW,
    // because CLEAR cmd (even though it's per-stage) does not stop the timer.

    // We have to temporarily stop and restart the current stage in order to
    // make the stopping of downstream stages and reloading of the current
    // stage atomic; otherwise after we stop the downstream stages, the currents
    // stage might expire and start a downstream stage, which would cause us to
    // exit this method with multiple stages counting, which violates our
    // invariant that only one timer is ever ticking at any given time.
    bool enabled = s->enabled;
    if (enabled) {
        ptimer_stop(s->ptimers[stage]);

        for (st = NUM_STAGES - 1; st > stage; --st)
            ptimer_stop(s->ptimers[st]);
    }

    ptimer_set_count(s->ptimers[stage], s->terminals[stage]);

    assert(s->enabled == enabled);
    if (enabled)
        ptimer_run(s->ptimers[stage], PTIMER_MODE_ONE_SHOT);
}

static void timer_adjust(HPSCWDTimer *s, unsigned stage, uint32_t terminal)
{
    // TODO: spec unclear whether loading terminal clears the elapsed or not, assume no

    ptimer_state *pt = s->ptimers[stage];
    if (s->enabled)
        ptimer_stop(pt); // freeze the simulation while we calculate offset

    uint64_t elapsed = s->terminals[stage] - ptimer_get_count(pt);
    uint64_t remaining = terminal >= elapsed ? terminal - elapsed : 0;
    DB_PRINT("%s: stage %u: adjust count <- %lx\n",
            object_get_canonical_path(OBJECT(s)), stage, remaining);
    ptimer_set_count(pt, remaining);

    if (s->enabled)
        ptimer_run(pt, PTIMER_MODE_ONE_SHOT); // unfreeze simulation
}

static void timer_update_freq(HPSCWDTimer *s)
{
    unsigned stage;
    // TODO: +1 in field semantics?
    unsigned tickdiv = extract32(s->regs[R_REG_CONFIG],
            R_REG_CONFIG_TICKDIV_SHIFT, R_REG_CONFIG_TICKDIV_LENGTH) + 1;
    uint32_t freq = CLK_FREQ_HZ / tickdiv;

    DB_PRINT("%s: update freq <- %u\n",
            object_get_canonical_path(OBJECT(s)), freq);

    for (stage = 0; stage < NUM_STAGES; ++stage)
        ptimer_set_freq(s->ptimers[stage], freq);
}


static void wdt_enable(HPSCWDTimer *s)
{
    set_enabled_state(s, true);
    ptimer_run(s->ptimers[0], PTIMER_MODE_ONE_SHOT);
}

static void wdt_disable(HPSCWDTimer *s)
{
    int stage;
    set_enabled_state(s, false);
    // only one stage is running, but we don't keep state of which one
    for (stage = NUM_STAGES - 1; stage >= 0; --stage)
        ptimer_stop(s->ptimers[stage]);
}

static void execute_stage_cmd(HPSCWDTimer *s, stage_cmd_t cmd, unsigned stage)
{
    uint64_t staging_terminal, count;
    unsigned reg_offset = stage * NUM_STAGE_REGS;
    switch (cmd) {
        case SCMD_CAPTURE: // convert down to up
                count = s->terminals[stage] - ptimer_get_count(s->ptimers[stage]);
                s->regs[reg_offset + R_REG_ST0_COUNT_LO] = (uint32_t)(count & 0xffffffff);
                s->regs[reg_offset + R_REG_ST0_COUNT_HI] = (uint32_t)(count >> 32);
                DB_PRINT("%s: stage cmd: stage %u: capture: count <- %lx\n",
                         object_get_canonical_path(OBJECT(s)), stage, count);
                break;
        case SCMD_LOAD:
                staging_terminal = get_staging_terminal(s, stage);
                timer_adjust(s, stage,  staging_terminal);
                s->terminals[stage] = staging_terminal;
                DB_PRINT("%s: stage cmd: stage %u: load: terminal <- %lx\n",
                         object_get_canonical_path(OBJECT(s)), stage, staging_terminal);
                break;
        case SCMD_CLEAR:
                DB_PRINT("%s: stage cmd: stage %u: clear\n",
                         object_get_canonical_path(OBJECT(s)), stage);
                timer_reload(s, stage);
                update_irq(s, stage, 0);
                break;
        default:
                assert(false && "unhandled stage cmd");
    }
}

static void execute_cmd(HPSCWDTimer *s, cmd_t cmd)
{
    switch (cmd) {
        case CMD_DISABLE:
                wdt_disable(s);
                break;
        default:
            assert(false && "unhandled cmd");
    }
}

#if 0
static void post_write_arm_code(DepRegisterInfo *reg, uint64_t val64)
{
    HPSCWDTimer *s = HPSC_WDT_TIMER(reg->opaque);
    switch (val64) {
#define STAGE_CMD_CASE(cmd, s) \
        case CMD_ ## cmd ## _ST ## s ## _ARM:
#define STAGE_CMD_CASES(s) \
        STAGE_CMD_CASE(CAPTURE, s) \
        STAGE_CMD_CASE(LOAD,    s) \
        STAGE_CMD_CASE(CLEAR,   s) \

#if NUM_STAGES >= 1
        STAGE_CMD_CASES(0)
#endif
#if NUM_STAGES >= 2
        STAGE_CMD_CASES(1)
#endif
#if NUM_STAGES >= 3
        STAGE_CMD_CASES(2)
#endif
#if NUM_STAGES >= 4
        STAGE_CMD_CASES(3)
#endif
#if NUM_STAGES >= 5
#error Manual declarations not setup for given number of stages, can add them.
#endif
#undef STAGE_CMD_CASES
#undef STAGE_CMD_CASE

        case CMD_DISABLE_ARM:
            DB_PRINT("%s: cmd armed: %" PRIx64 "\n",
                     object_get_canonical_path(OBJECT(s)), val64);
            s->arm_code = val64;
            break;
        default:
            DB_PRINT_mask(LOG_GUEST_ERROR,
                    "%s: unrecognized cmd arm code: %" PRIx64 "\n",
                     object_get_canonical_path(OBJECT(s)), val64);
    }
}
#endif

static void post_write_cmd_fire(DepRegisterInfo *reg, uint64_t val64)
{
    HPSCWDTimer *s = HPSC_WDT_TIMER(reg->opaque);
    unsigned stage;
    cmd_t cmd = CMD_INVALID;
    stage_cmd_t scmd = SCMD_INVALID;
    uint32_t arm_code = s->regs[R_REG_CMD_ARM];
    DB_PRINT("%s: arm code: %x\n",
             object_get_canonical_path(OBJECT(s)), arm_code);
    switch (val64) {
        case CMD_DISABLE_FIRE:
            if (arm_code == CMD_DISABLE_ARM)
                cmd = CMD_DISABLE;
            break;

#define STAGE_CMD_CASE(cmd, s) \
        case STAGE_CMD_CODE(cmd, s, FIRE): \
            if (arm_code == STAGE_CMD_CODE(cmd, s, ARM)) { scmd = SCMD_ ## cmd; stage = s; } break;

#define STAGE_CMD_CASES(s) \
        STAGE_CMD_CASE(CAPTURE, s) \
        STAGE_CMD_CASE(LOAD, s) \
        STAGE_CMD_CASE(CLEAR, s)

#if NUM_STAGES >= 1
        STAGE_CMD_CASES(0)
#endif
#if NUM_STAGES >= 2
        STAGE_CMD_CASES(1)
#endif
#if NUM_STAGES >= 3
        STAGE_CMD_CASES(2)
#endif
#if NUM_STAGES >= 4
        STAGE_CMD_CASES(3)
#endif
#if NUM_STAGES >= 5
#error Manual declarations not setup for given number of stages, can add them.
#endif
#undef STAGE_CMD_CASES
#undef STAGE_CMD_CASE

        default:
            qemu_log_mask(LOG_GUEST_ERROR,
                    "%s: unrecognized cmd fire code: %" PRIx64 "\n",
                     object_get_canonical_path(OBJECT(s)), val64);
    }

    if (cmd != CMD_INVALID) {
            DB_PRINT("%s: cmd fired: %u\n",
                     object_get_canonical_path(OBJECT(s)), cmd);
            execute_cmd(s, cmd);
    } else if (scmd != SCMD_INVALID) {
            DB_PRINT("%s: stage cmd fired: scmd %u stage %u\n",
                     object_get_canonical_path(OBJECT(s)), scmd, stage);
            execute_stage_cmd(s, scmd, stage);
    } else {
        qemu_log_mask(LOG_GUEST_ERROR,
                "%s: mismatched fire code %" PRIx64 " for arm code %u\n",
                object_get_canonical_path(OBJECT(s)), val64, arm_code);
    }
}

static uint64_t pre_write_config(DepRegisterInfo *reg, uint64_t val64)
{
    HPSCWDTimer *s = HPSC_WDT_TIMER(reg->opaque);
    bool cur_en = extract32(s->regs[R_REG_CONFIG],
            R_REG_CONFIG_EN_SHIFT, R_REG_CONFIG_EN_LENGTH);
    bool new_en = extract32(val64,
            R_REG_CONFIG_EN_SHIFT, R_REG_CONFIG_EN_LENGTH);
    if (cur_en && !new_en) {
        qemu_log("%s: ignored attempt to disable WDT via config register\n",
                object_get_canonical_path(OBJECT(s)));
        val64 |= 1 << R_REG_CONFIG_EN_SHIFT;
    }
    return val64;
}

static void post_write_config(DepRegisterInfo *reg, uint64_t val64)
{
    HPSCWDTimer *s = HPSC_WDT_TIMER(reg->opaque);
    bool enabled = extract32(s->regs[R_REG_CONFIG],
            R_REG_CONFIG_EN_SHIFT, R_REG_CONFIG_EN_LENGTH);
    timer_update_freq(s); // TODO: is this update indeed a NOP if value unchanged?

    // Either currently disabled, or we're asking to enable, not to disable.
    // Asking to disable should not reach hear since overwridden by pre_write_config.
    assert(!s->enabled || enabled);

    if (enabled != s->enabled)
        wdt_enable(s);
}

static uint64_t pre_write_status(DepRegisterInfo *reg, uint64_t val64)
{
    // ignore attempts to set the timeout status bit
    return val64 & ~R_REG_STATUS_TIMEOUT_MASK;
}

static void post_write_status(DepRegisterInfo *reg, uint64_t val64)
{
    HPSCWDTimer *s = HPSC_WDT_TIMER(reg->opaque);
    bool timeout = extract32(s->regs[R_REG_STATUS],
            R_REG_STATUS_TIMEOUT_SHIFT, R_REG_STATUS_TIMEOUT_LENGTH);
    unsigned stage;
    for (stage = 0; stage < NUM_STAGES; ++stage)
        if (!(timeout & (1 << stage)))
            qemu_set_irq(s->irqs[stage], 0);
}

// Terminal values reset to max by spec
static DepRegisterAccessInfo hpsc_wdt_regs_info[] = {

#define REG_INFO_STAGE_INNER(reg, stage, defval) \
    {   .name = "REG_ST" #stage "_" #reg, \
        .decode.addr = CONCAT5(A_, REG_ST, stage, _, reg), \
        .reset = defval, \
        .rsvd = (~0ULL << COUNTER_WIDTH), \
    }
#define REG_INFO_STAGE(reg, stage, defval) REG_INFO_STAGE_INNER(reg, stage, defval)

#define REGS_INFO_STAGE(stage) \
    REG_INFO_STAGE(TERMINAL_LO, stage, (~0ULL >> (64 - COUNTER_WIDTH)) & 0xffffffff), \
    REG_INFO_STAGE(TERMINAL_HI, stage, (~0ULL >> (64 - COUNTER_WIDTH)) >> 32), \
    REG_INFO_STAGE(COUNT_LO,    stage, 0x0), \
    REG_INFO_STAGE(COUNT_HI,    stage, 0x0), \

#if NUM_STAGES >= 1
REGS_INFO_STAGE(0)
#endif
#if NUM_STAGES >= 2
REGS_INFO_STAGE(1)
#endif
#if NUM_STAGES >= 3
REGS_INFO_STAGE(2)
#endif
#if NUM_STAGES >= 4
REGS_INFO_STAGE(3)
#endif
#if NUM_STAGES >= 5
#error Manual declarations not setup for given number of stages, can add them.
#endif // NUM_STAGES

    { .name = "REG_CONFIG",
        .decode.addr = A_REG_CONFIG,
        .reset = 0x0,
        .rsvd = ~(R_REG_CONFIG_EN_MASK | R_REG_CONFIG_TICKDIV_MASK),
        .pre_write = pre_write_config,
        .post_write = post_write_config,
    },{ .name = "REG_STATUS",
        .decode.addr = A_REG_STATUS,
        .reset = 0x0,
        .rsvd = ~(R_REG_STATUS_DBGDIS_MASK | R_REG_STATUS_TIMEOUT_MASK),
        .pre_write = pre_write_status,
        .post_write = post_write_status,
    },{ .name = "REG_CMD_ARM",
        .decode.addr = A_REG_CMD_ARM,
        .reset = 0x0,
        //.post_write = post_write_arm_code,
    },{ .name = "REG_CMD_FIRE",
        .decode.addr = A_REG_CMD_FIRE,
        .reset = 0x0,
        .post_write = post_write_cmd_fire,
   }
};

static void hpsc_wdt_reset(DeviceState *dev)
{
    HPSCWDTimer *s = HPSC_WDT_TIMER(dev);
    unsigned int i;
    unsigned int stage;

    for (i = 0; i < ARRAY_SIZE(s->regs_info); ++i)
        dep_register_reset(&s->regs_info[i]);

    s->enabled = false;
    timer_update_freq(s);

    for (stage = 0; stage < NUM_STAGES; ++stage) {
        s->terminals[stage] = get_staging_terminal(s, stage);
        DB_PRINT("%s: stage %u: reset: terminal <- %lx (width %u)\n",
                object_get_canonical_path(OBJECT(s)),
                stage, s->terminals[stage], COUNTER_WIDTH);
        timer_reload(s, stage);
        update_irq(s, stage, 0);
    }
}

static uint64_t hpsc_wdt_read(void *opaque, hwaddr addr, unsigned size)
{
    HPSCWDTimer *s = HPSC_WDT_TIMER(opaque);
    DepRegisterInfo *r = &s->regs_info[addr / 4];

    if (!r->data) {
        qemu_log_mask(LOG_GUEST_ERROR,
                "%s: Decode error: read from %" HWADDR_PRIx "\n",
                object_get_canonical_path(OBJECT(s)), addr);
        return 0;
    }
    return dep_register_read(r);
}

static void hpsc_wdt_write(void *opaque, hwaddr addr, uint64_t value,
                      unsigned size)
{
    HPSCWDTimer *s = HPSC_WDT_TIMER(opaque);
    DepRegisterInfo *r = &s->regs_info[addr / 4];

    if (!r->data) {
        qemu_log_mask(LOG_GUEST_ERROR,
                "%s: Decode error: write to %" HWADDR_PRIx "=%" PRIx64 "\n",
                object_get_canonical_path(OBJECT(s)), addr, value);
        return;
    }
    dep_register_write(r, value, ~0);
}

static void hpsc_wdt_access(MemoryTransaction *tr)
{
#if 0
    MemTxAttrs attr = tr->attr;
#endif
    void *opaque = tr->opaque;
    hwaddr addr = tr->addr;
    unsigned size = tr->size;
    uint64_t value = tr->data.u64;;
    bool is_write = tr->rw;

#if 0 // TODO
    if (!attr.secure) {
        qemu_log_mask(LOG_GUEST_ERROR, "unsecure access to timer denied\n");
        return;
    }
#endif

    if (is_write) {
        hpsc_wdt_write(opaque, addr, value, size);
    } else {
        tr->data.u64 = hpsc_wdt_read(opaque, addr, size);
    }
}

static void timer_tick(void *opaque)
{
    stage_ctx_t *ctx = opaque;
    HPSCWDTimer *s = ctx->timer;
    unsigned stage = ctx->stage;

    DB_PRINT("%s: stage %u: tick\n",
            object_get_canonical_path(OBJECT(s)), stage);

    // Probably not possible, if the tick callback is serialized with the other
    // calls that block the target. But, just in case, don't set the irq.
    if (!s->enabled)
        return;

    update_irq(s, stage, 1);

    if (stage < NUM_STAGES - 1) {
        unsigned next_stage = stage + 1;
        ptimer_set_count(s->ptimers[next_stage], s->terminals[next_stage]);
        ptimer_run(s->ptimers[next_stage], PTIMER_MODE_ONE_SHOT);
    } else { // last stage
        set_enabled_state(s, false);
        if (s->last_timeout_gpio) // whether hooked up determines behavior
            hpsc_wdt_reset((DeviceState *)s);
    }
    // ptimer already disabled because it is one shot
}

static const MemoryRegionOps hpsc_wdt_ops = {
    .access = hpsc_wdt_access,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static const FDTGenericGPIOSet wdt_gpios[] = {
    {
        .names = &fdt_generic_gpio_name_set_gpio,
        .gpios = (FDTGenericGPIOConnection[]) {
            { .name = GPIO_NAME_LAST_TIMEOUT,     .fdt_index = 0,     .range = 1 },
            { }
        }
    },
    { }
};

static void hpsc_wdt_realize(DeviceState *dev, Error **errp)
{
    HPSCWDTimer *s = HPSC_WDT_TIMER(dev);
    const char *prefix = object_get_canonical_path(OBJECT(dev));
    unsigned int i;

    for (i = 0; i < ARRAY_SIZE(hpsc_wdt_regs_info); ++i) {
        DepRegisterInfo *r =
                    &s->regs_info[hpsc_wdt_regs_info[i].decode.addr / 4];

        *r = (DepRegisterInfo) {
            .data = (uint8_t *)&s->regs[
                    hpsc_wdt_regs_info[i].decode.addr/4],
            .data_size = sizeof(uint32_t),
            .access = &hpsc_wdt_regs_info[i],
            .debug = HPSC_WDT_TIMER_ERR_DEBUG,
            .prefix = prefix,
            .opaque = s,
        };
        dep_register_init(r);
        qdev_pass_all_gpios(DEVICE(r), dev);
    }

    for (i = 0; i < NUM_STAGES; ++i) {
        stage_ctx_t *ctx = &s->stage_ctxs[i];
        ctx->timer = s;
        ctx->stage = i;
        s->bhs[i] = qemu_bh_new(timer_tick, ctx);
        s->ptimers[i] = ptimer_init(s->bhs[i], PTIMER_POLICY_DEFAULT);
    }
}

static void hpsc_wdt_init(Object *obj)
{
    HPSCWDTimer *s = HPSC_WDT_TIMER(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    unsigned stage;

    memory_region_init_io(&s->iomem, obj, &hpsc_wdt_ops, s,
                          TYPE_HPSC_WDT_TIMER, R_MAX * 4);
    sysbus_init_mmio(sbd, &s->iomem);

    for (stage = 0; stage < NUM_STAGES; ++stage)
        sysbus_init_irq(SYS_BUS_DEVICE(s), &s->irqs[stage]);

    qdev_init_gpio_out_named(DEVICE(obj), &s->last_timeout_gpio,
                            GPIO_NAME_LAST_TIMEOUT, 1);
}

static const VMStateDescription vmstate_hpsc_wdt = {
    .name = TYPE_HPSC_WDT_TIMER,
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, HPSCWDTimer, R_MAX),
        VMSTATE_END_OF_LIST(),
    }
};

static void hpsc_wdt_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    FDTGenericGPIOClass *fggc = FDT_GENERIC_GPIO_CLASS(klass);

    dc->reset = hpsc_wdt_reset;
    dc->realize = hpsc_wdt_realize;
    dc->vmsd = &vmstate_hpsc_wdt;

    fggc->controller_gpios = wdt_gpios;
}

static const TypeInfo hpsc_wdt_info = {
    .name          = TYPE_HPSC_WDT_TIMER,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(HPSCWDTimer),
    .class_init    = hpsc_wdt_class_init,
    .instance_init = hpsc_wdt_init,
    .interfaces    = (InterfaceInfo[]) {
        { TYPE_FDT_GENERIC_GPIO },
        { }
    },
};

static void hpsc_wdt_register_types(void)
{
    type_register_static(&hpsc_wdt_info);
}

type_init(hpsc_wdt_register_types)
