/*
 * Copyright (c) 2025 Institute of Computing Technology,
 * Chinese Academy of Sciences
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Provides a board compatible with the RVTNG-V0 SoC
 * XCVU13P FPGA prototype platform:
 *
 * 0) Mailbox memory
 * 1) NPU-to-host interrupt handler
 * 2) Boot ROM (Stage-1)
 * 3) Dayan CLINT (Core-Local Interruptor)
 * 4) Atomic-cache backing memory
 * 5) Per-core tensor URAM
 * 6) DRAM
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2 or later, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "qemu/osdep.h"
#include "qemu/error-report.h"
#include "qemu/log.h"
#include "qemu/main-loop.h"
#include "qapi/error.h"
#include "system/address-spaces.h"
#include "hw/boards.h"
#include "hw/intc/riscv_aclint.h"
#include "hw/qdev-properties.h"
#include "hw/riscv/boot.h"
#include "hw/riscv/rvtng_v0.h"
#include "hw/riscv/rvtng-intf.h"
#include "hw/riscv/rvtng_fuse.h"
#include "hw/riscv/riscv_hart.h"
#include "system/reset.h"
#include "system/system.h"
#include "target/riscv/cpu.h"

#define DAYAN_CLINT_IPI_BASE          0x100
#define DAYAN_CLINT_IPI_HART_SIZE     0x10
#define DAYAN_CLINT_IPI_STATUS_OFF    0x0
#define DAYAN_CLINT_IPI_ENABLE_OFF    0x4
#define DAYAN_CLINT_IPI_PENDING_OFF   0x8
#define DAYAN_CLINT_IPI_CLEAR_OFF     0xc
#define DAYAN_CLINT_MTIMECMP_OFF      0x4000
#define DAYAN_CLINT_MTIME_OFF         0xbff8
#define DAYAN_CLINT_IPI_APERTURE_SIZE DAYAN_CLINT_MTIMECMP_OFF
#define DAYAN_CLINT_MTIMER_SIZE       0xc000
#define DAYAN_CLINT_TIMEBASE_FREQ     1000000
#define RVTNG_V0_URAM_STRIDE          0x200000

static const MemMapEntry rvtng_v0_memmap[] = {
    [RVTNG_V0_MAILBOX]           = {     0x1000,        0x1000 },
    [RVTNG_V0_INTERRUPT_HANDLER] = {     0x2000,         0x200 },
    [RVTNG_V0_ATOMIC_CACHE_CONTROL] = {  0x4000,        0x1000 },
    [RVTNG_V0_MMU]               = {     0x5000,        0x1000 },
    [RVTNG_V0_RESOURCE_CONTROL]  = {     0x6000,        0x1000 },
    [RVTNG_V0_ROM]               = {    0x10000,       0x10000 },
    [RVTNG_V0_CLINT]             = {    0x20000,       0x10000 },
    [RVTNG_V0_ATOMIC_CACHE]      = {   0x400000,      0x400000 },
    [RVTNG_V0_URAM]              = {   0x800000,      0x180000 },
    [RVTNG_V0_DRAM]              = { 0x80000000, 0x100000000UL },
};

/*
 * The debug controller, error device, PCU, and PLIC apertures
 * intentionally remain unassigned until those models are implemented.
 * The ClusterMMU aperture is modeled by TYPE_RVTNG_MMU at 0x5000.
 */

static MemTxResult rvtng_resource_control_read(void *opaque, hwaddr offset,
                                              uint64_t *value, unsigned size,
                                              MemTxAttrs attrs)
{
    *value = 0;
    return MEMTX_OK;
}

static MemTxResult rvtng_resource_control_write(void *opaque, hwaddr offset,
                                               uint64_t value, unsigned size,
                                               MemTxAttrs attrs)
{
    if (offset != 0) {
        return MEMTX_OK;
    }
    return rvtng_cmodel_resource_clear((uint32_t)value) ? MEMTX_OK : MEMTX_ERROR;
}

static const MemoryRegionOps rvtng_resource_control_ops = {
    .read_with_attrs = rvtng_resource_control_read,
    .write_with_attrs = rvtng_resource_control_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 8 },
    .impl = { .min_access_size = 4, .max_access_size = 8 },
};

static uint64_t rvtng_atomic_cache_control_read(void *opaque, hwaddr offset,
                                                unsigned size)
{
    RVTNGV0SoCState *s = opaque;

    (void)size;
    switch (offset) {
    case RVTNG_ATOMIC_CACHE_STATUS_OFF:
        return s->atomic_cache_status;
    case RVTNG_ATOMIC_CACHE_BACKING_OFF:
        return s->atomic_cache_backing_base;
    default:
        return 0;
    }
}

static void rvtng_atomic_cache_control_write(void *opaque, hwaddr offset,
                                             uint64_t value, unsigned size)
{
    RVTNGV0SoCState *s = opaque;

    if (offset == RVTNG_ATOMIC_CACHE_BACKING_OFF &&
        size == sizeof(s->atomic_cache_backing_base)) {
        s->atomic_cache_backing_base = value;
        s->atomic_cache_status = RVTNG_ATOMIC_CACHE_STATUS_DONE;
    } else if (offset == RVTNG_ATOMIC_CACHE_STATUS_OFF && (value & 1)) {
        /* The functional cache is not modeled; complete a flush immediately. */
        s->atomic_cache_status = RVTNG_ATOMIC_CACHE_STATUS_DONE;
    }
}

static const MemoryRegionOps rvtng_atomic_cache_control_ops = {
    .read = rvtng_atomic_cache_control_read,
    .write = rvtng_atomic_cache_control_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 8,
        .unaligned = true,
    },
    .impl = {
        .min_access_size = 1,
        .max_access_size = 8,
        .unaligned = true,
    },
};

static void rvtng_intc_emit_rising_irq(uint32_t prev_status,
                                       uint32_t prev_enable,
                                       uint32_t new_status,
                                       uint32_t new_enable)
{
    uint32_t prev_irq = prev_status & prev_enable;
    uint32_t new_irq = new_status & new_enable;
    uint32_t rising = new_irq & ~prev_irq;

    for (int i = 0; i < RVTNG_INTC_MAILBOX_COUNT; i++) {
        if (rising & (1u << i)) {
            rvtng_fuse_emit_event(i, 1);
        }
    }
}

static uint32_t rvtng_intc_ctrl_read(RVTNGV0SoCState *s, hwaddr offset)
{
    switch (offset) {
    case RVTNG_INTC_STATUS_OFF:
    case RVTNG_INTC_SET_OFF:
    case RVTNG_INTC_CLEAR_OFF:
        return s->intc_status;
    case RVTNG_INTC_ENABLE_OFF:
    case RVTNG_INTC_ENABLE_SET_OFF:
    case RVTNG_INTC_ENABLE_CLEAR_OFF:
        return s->intc_enable;
    default:
        return 0;
    }
}

static uint8_t rvtng_intc_readb(RVTNGV0SoCState *s, hwaddr offset)
{
    hwaddr mailbox_end = RVTNG_INTC_MAILBOX0_OFF +
                         RVTNG_INTC_MAILBOX_COUNT * sizeof(uint64_t);

    if (offset < RVTNG_INTC_MAILBOX0_OFF) {
        hwaddr reg_offset = offset & ~0x3;
        unsigned shift = (offset & 0x3) * 8;

        return rvtng_intc_ctrl_read(s, reg_offset) >> shift;
    }

    if (offset < mailbox_end) {
        hwaddr rel = offset - RVTNG_INTC_MAILBOX0_OFF;
        unsigned index = rel / sizeof(uint64_t);
        unsigned shift = (rel % sizeof(uint64_t)) * 8;

        return s->intc_mailbox[index] >> shift;
    }

    return 0;
}

static void rvtng_intc_writeb(RVTNGV0SoCState *s, hwaddr offset,
                              uint8_t value)
{
    hwaddr mailbox_end = RVTNG_INTC_MAILBOX0_OFF +
                         RVTNG_INTC_MAILBOX_COUNT * sizeof(uint64_t);

    if (offset < RVTNG_INTC_MAILBOX0_OFF) {
        hwaddr reg_offset = offset & ~0x3;
        unsigned shift = (offset & 0x3) * 8;
        uint32_t bits = (uint32_t)value << shift;

        switch (reg_offset) {
        case RVTNG_INTC_SET_OFF:
            s->intc_status |= bits;
            break;
        case RVTNG_INTC_CLEAR_OFF:
            s->intc_status &= ~bits;
            break;
        case RVTNG_INTC_ENABLE_SET_OFF:
            s->intc_enable |= bits;
            break;
        case RVTNG_INTC_ENABLE_CLEAR_OFF:
            s->intc_enable &= ~bits;
            break;
        default:
            /* STATUS and ENABLE are read-only; 0x18-0x1f is reserved. */
            break;
        }
        return;
    }

    if (offset < mailbox_end) {
        hwaddr rel = offset - RVTNG_INTC_MAILBOX0_OFF;
        unsigned index = rel / sizeof(uint64_t);
        unsigned shift = (rel % sizeof(uint64_t)) * 8;
        uint64_t mask = UINT64_C(0xff) << shift;

        s->intc_mailbox[index] = (s->intc_mailbox[index] & ~mask) |
                                 ((uint64_t)value << shift);
    }
}

/*
 * Interrupt-handler MMIO bridge shared by guest accesses and xdma0_user.
 * The register layout follows TLInterruptHandler with 32 interrupt sources.
 */
static void rvtng_intc_write(void *opaque, hwaddr offset, uint64_t value,
                             unsigned size)
{
    RVTNGV0SoCState *s = opaque;
    uint32_t prev_status = s->intc_status;
    uint32_t prev_enable = s->intc_enable;

    for (unsigned i = 0; i < size; i++) {
        rvtng_intc_writeb(s, offset + i, value >> (i * 8));
    }

    rvtng_intc_emit_rising_irq(prev_status, prev_enable,
                               s->intc_status, s->intc_enable);
}

static uint64_t rvtng_intc_read(void *opaque, hwaddr offset, unsigned size)
{
    RVTNGV0SoCState *s = opaque;
    uint64_t value = 0;

    for (unsigned i = 0; i < size; i++) {
        value |= (uint64_t)rvtng_intc_readb(s, offset + i) << (i * 8);
    }

    return value;
}

static const MemoryRegionOps rvtng_intc_ops = {
    .read = rvtng_intc_read,
    .write = rvtng_intc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 8,
        .unaligned = true,
    },
    .impl = {
        .min_access_size = 1,
        .max_access_size = 8,
        .unaligned = true,
    },
};

static void rvtng_clint_update_msip(RVTNGV0SoCState *s, unsigned hart)
{
    bool level = (s->clint_ipi_pending[hart] &
                  s->clint_ipi_enable[hart]) != 0;

    qemu_set_irq(s->clint_msip_irq[hart], level);
}

static uint32_t rvtng_clint_ipi_read_reg(RVTNGV0SoCState *s,
                                         unsigned hart, hwaddr offset)
{
    switch (offset) {
    case DAYAN_CLINT_IPI_STATUS_OFF:
        return s->clint_ipi_pending[hart] & s->clint_ipi_enable[hart];
    case DAYAN_CLINT_IPI_ENABLE_OFF:
        return s->clint_ipi_enable[hart];
    case DAYAN_CLINT_IPI_PENDING_OFF:
    case DAYAN_CLINT_IPI_CLEAR_OFF:
        return s->clint_ipi_pending[hart];
    default:
        return 0;
    }
}

static uint8_t rvtng_clint_ipi_readb(RVTNGV0SoCState *s, hwaddr offset)
{
    if (offset < s->num_harts * sizeof(uint32_t)) {
        unsigned hart = offset / sizeof(uint32_t);
        unsigned shift = (offset % sizeof(uint32_t)) * 8;
        uint32_t status = s->clint_ipi_pending[hart] &
                          s->clint_ipi_enable[hart];

        return status >> shift;
    }

    if (offset >= DAYAN_CLINT_IPI_BASE) {
        hwaddr rel = offset - DAYAN_CLINT_IPI_BASE;
        unsigned hart = rel / DAYAN_CLINT_IPI_HART_SIZE;

        if (hart < s->num_harts) {
            hwaddr hart_offset = rel % DAYAN_CLINT_IPI_HART_SIZE;
            hwaddr reg_offset = hart_offset & ~0x3;
            unsigned shift = (hart_offset & 0x3) * 8;

            return rvtng_clint_ipi_read_reg(s, hart, reg_offset) >> shift;
        }
    }

    return 0;
}

static void rvtng_clint_ipi_writeb(RVTNGV0SoCState *s, hwaddr offset,
                                   uint8_t value)
{
    if (offset < s->num_harts * sizeof(uint32_t)) {
        unsigned hart = offset / sizeof(uint32_t);

        /* Only bit 0 of each legacy MSIP alias is writable. */
        if ((offset & 0x3) == 0) {
            if (value & 1) {
                s->clint_ipi_pending[hart] |= 1;
            } else {
                s->clint_ipi_pending[hart] = 0;
            }
        }
        return;
    }

    if (offset >= DAYAN_CLINT_IPI_BASE) {
        hwaddr rel = offset - DAYAN_CLINT_IPI_BASE;
        unsigned hart = rel / DAYAN_CLINT_IPI_HART_SIZE;

        if (hart < s->num_harts) {
            hwaddr hart_offset = rel % DAYAN_CLINT_IPI_HART_SIZE;
            hwaddr reg_offset = hart_offset & ~0x3;
            unsigned shift = (hart_offset & 0x3) * 8;
            uint32_t mask = UINT32_C(0xff) << shift;
            uint32_t bits = (uint32_t)value << shift;

            switch (reg_offset) {
            case DAYAN_CLINT_IPI_ENABLE_OFF:
                s->clint_ipi_enable[hart] =
                    (s->clint_ipi_enable[hart] & ~mask) | bits | 1;
                break;
            case DAYAN_CLINT_IPI_PENDING_OFF:
                s->clint_ipi_pending[hart] |= bits;
                break;
            case DAYAN_CLINT_IPI_CLEAR_OFF:
                s->clint_ipi_pending[hart] &= ~bits;
                break;
            default:
                /* IPI status is read-only. */
                break;
            }
        }
    }
}

static bool rvtng_clint_is_pending_set_byte(RVTNGV0SoCState *s,
                                             hwaddr offset)
{
    hwaddr rel;
    unsigned hart;
    hwaddr hart_offset;

    if (offset < DAYAN_CLINT_IPI_BASE) {
        return false;
    }

    rel = offset - DAYAN_CLINT_IPI_BASE;
    hart = rel / DAYAN_CLINT_IPI_HART_SIZE;
    hart_offset = rel % DAYAN_CLINT_IPI_HART_SIZE;
    return hart < s->num_harts &&
           (hart_offset & ~0x3) == DAYAN_CLINT_IPI_PENDING_OFF;
}

static uint64_t rvtng_clint_ipi_read(void *opaque, hwaddr offset,
                                     unsigned size)
{
    RVTNGV0SoCState *s = opaque;
    uint64_t value = 0;

    for (unsigned i = 0; i < size; i++) {
        value |= (uint64_t)rvtng_clint_ipi_readb(s, offset + i) <<
                 (i * 8);
    }

    return value;
}

static void rvtng_clint_ipi_write(void *opaque, hwaddr offset,
                                  uint64_t value, unsigned size)
{
    RVTNGV0SoCState *s = opaque;

    /*
     * A wide write can cover PENDING/SET and CLEAR together.  Match the RTL
     * by applying clears first and sets second, so SET wins on overlap.
     */
    for (unsigned i = 0; i < size; i++) {
        if (!rvtng_clint_is_pending_set_byte(s, offset + i)) {
            rvtng_clint_ipi_writeb(s, offset + i, value >> (i * 8));
        }
    }
    for (unsigned i = 0; i < size; i++) {
        if (rvtng_clint_is_pending_set_byte(s, offset + i)) {
            rvtng_clint_ipi_writeb(s, offset + i, value >> (i * 8));
        }
    }
    for (unsigned hart = 0; hart < s->num_harts; hart++) {
        rvtng_clint_update_msip(s, hart);
    }
}

static const MemoryRegionOps rvtng_clint_ipi_ops = {
    .read = rvtng_clint_ipi_read,
    .write = rvtng_clint_ipi_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 8,
        .unaligned = true,
    },
    .impl = {
        .min_access_size = 1,
        .max_access_size = 8,
        .unaligned = true,
    },
};

static bool rvtng_clint_connect_harts(RVTNGV0SoCState *s, Error **errp)
{
    for (unsigned hart = 0; hart < s->num_harts; hart++) {
        CPUState *cpu = cpu_by_arch_id(hart);

        if (!cpu) {
            error_setg(errp, "rvtng-v0 CLINT cannot find hart %u", hart);
            return false;
        }
        if (riscv_cpu_claim_interrupts(RISCV_CPU(cpu), MIP_MSIP) < 0) {
            error_setg(errp, "rvtng-v0 hart %u MSIP is already claimed", hart);
            return false;
        }
        s->clint_msip_irq[hart] =
            qdev_get_gpio_in(DEVICE(cpu), IRQ_M_SOFT);
    }

    return true;
}

static void rvtng_clint_reset_mtimecmp(RVTNGV0SoCState *s)
{
    const MemMapEntry *memmap = rvtng_v0_memmap;

    for (unsigned hart = 0; hart < s->num_harts; hart++) {
        MemTxResult result;
        hwaddr address = memmap[RVTNG_V0_CLINT].base +
                         DAYAN_CLINT_MTIMECMP_OFF +
                         hart * sizeof(uint64_t);

        address_space_stq_le(&address_space_memory, address, UINT64_MAX,
                             MEMTXATTRS_UNSPECIFIED, &result);
        if (result != MEMTX_OK) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "rvtng-v0: failed to reset mtimecmp for hart %u\n",
                          hart);
        }
    }
}

static void rvtng_v0_soc_reset_hold(Object *obj, ResetType type)
{
    RVTNGV0SoCState *s = RVTNG_V0_SOC(obj);

    rvtng_fuse_reset_events();
    s->intc_status = 0;
    s->intc_enable = 0;
    s->atomic_cache_status = 0;
    s->atomic_cache_backing_base = 0;
    for (unsigned i = 0; i < RVTNG_INTC_MAILBOX_COUNT; i++) {
        s->intc_mailbox[i] = 1;
    }

    for (unsigned hart = 0; hart < RVTNG_MAX_CALC_CORES; hart++) {
        s->clint_ipi_pending[hart] = 0;
        s->clint_ipi_enable[hart] = 1;
        if (hart < s->num_harts && s->clint_msip_irq[hart]) {
            qemu_irq_lower(s->clint_msip_irq[hart]);
        }
    }

    if (s->num_harts) {
        rvtng_clint_reset_mtimecmp(s);
    }
}

/* Global address-space bridge used by the FUSE xdma0_user endpoint. */
AddressSpace *fuse_as;

static void rvtng_v0_soc_realize(DeviceState *dev, Error **errp)
{
    MachineState *ms = MACHINE(qdev_get_machine());
    RVTNGV0SoCState *s = RVTNG_V0_SOC(dev);
    const MemMapEntry *memmap = rvtng_v0_memmap;
    MemoryRegion *system_memory = get_system_memory();
    uint32_t num_harts = ms->smp.cpus;

    if (num_harts > RVTNG_MAX_CALC_CORES) {
        error_setg(errp, "rvtng-v0 supports at most %u harts",
                   RVTNG_MAX_CALC_CORES);
        return;
    }

    g_assert(memmap[RVTNG_V0_INTERRUPT_HANDLER].base == RVTNG_INTC_BASE);
    g_assert(memmap[RVTNG_V0_INTERRUPT_HANDLER].size == RVTNG_INTC_SIZE);
    g_assert(memmap[RVTNG_V0_ATOMIC_CACHE_CONTROL].base ==
             RVTNG_ATOMIC_CACHE_CONTROL_BASE);
    g_assert(memmap[RVTNG_V0_ATOMIC_CACHE_CONTROL].size ==
             RVTNG_ATOMIC_CACHE_CONTROL_SIZE);
    g_assert(memmap[RVTNG_V0_MMU].size == RVTNG_MMU_APB_SIZE);
    g_assert(DAYAN_CLINT_MTIMECMP_OFF + DAYAN_CLINT_MTIMER_SIZE ==
             memmap[RVTNG_V0_CLINT].size);
    s->num_harts = num_harts;
    fuse_as = &address_space_memory;

    qdev_prop_set_uint32(DEVICE(&s->cpus), "num-harts", num_harts);
    qdev_prop_set_uint32(DEVICE(&s->cpus), "hartid-base", 0);
    qdev_prop_set_string(DEVICE(&s->cpus), "cpu-type",
                         TYPE_RISCV_CPU_RVTNG_CALC_V0);
    qdev_prop_set_uint64(DEVICE(&s->cpus), "resetvec",
                         memmap[RVTNG_V0_ROM].base);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->cpus), errp)) {
        return;
    }
    for (int core_idx = 0; core_idx < num_harts; core_idx++) {
        s->tensor_irqs[core_idx] = qdev_get_gpio_in(
            DEVICE(&s->cpus.harts[core_idx]), IRQ_M_EXT);
    }

    /* Dayan CLINT IPI registers and software-interrupt outputs. */
    if (!rvtng_clint_connect_harts(s, errp)) {
        return;
    }
    memory_region_init_io(&s->clint_ipi, OBJECT(dev),
                          &rvtng_clint_ipi_ops, s, "rvtng.v0.clint.ipi",
                          DAYAN_CLINT_IPI_APERTURE_SIZE);
    memory_region_add_subregion(system_memory, memmap[RVTNG_V0_CLINT].base,
                                &s->clint_ipi);

    /* The standard ACLINT timer backend matches Dayan's timer layout. */
    riscv_aclint_mtimer_create(memmap[RVTNG_V0_CLINT].base +
                               DAYAN_CLINT_MTIMECMP_OFF,
                               DAYAN_CLINT_MTIMER_SIZE, 0, num_harts, 0,
                               DAYAN_CLINT_MTIME_OFF -
                               DAYAN_CLINT_MTIMECMP_OFF,
                               DAYAN_CLINT_TIMEBASE_FREQ, true);

    /* ROM */
    if (!memory_region_init_rom(&s->rom, OBJECT(dev), "rvtng.v0.rom",
                                memmap[RVTNG_V0_ROM].size, errp)) {
        return;
    }
    memory_region_add_subregion(system_memory,
                                memmap[RVTNG_V0_ROM].base, &s->rom);

    /* MAILBOX */
    if (!memory_region_init_ram(&s->mailbox, OBJECT(dev),
                                "rvtng.v0.mailbox",
                                memmap[RVTNG_V0_MAILBOX].size, errp)) {
        return;
    }
    memory_region_add_subregion(system_memory,
                                memmap[RVTNG_V0_MAILBOX].base, &s->mailbox);

    /* NPU-to-host interrupt-handler, also exposed through xdma0_user. */
    memory_region_init_io(&s->intc, OBJECT(dev), &rvtng_intc_ops, s,
                         "rvtng.v0.interrupt-handler",
                         memmap[RVTNG_V0_INTERRUPT_HANDLER].size);
    memory_region_add_subregion(
        system_memory, memmap[RVTNG_V0_INTERRUPT_HANDLER].base, &s->intc);

    /* Atomic-cache control page. Cache data behavior remains abstracted. */
    memory_region_init_io(&s->atomic_cache_control, OBJECT(dev),
                          &rvtng_atomic_cache_control_ops, s,
                          "rvtng.v0.atomic-cache-control",
                          memmap[RVTNG_V0_ATOMIC_CACHE_CONTROL].size);
    memory_region_add_subregion(
        system_memory, memmap[RVTNG_V0_ATOMIC_CACHE_CONTROL].base,
        &s->atomic_cache_control);

    /* Shared Resource table initialization, forwarded to the CModel. */
    memory_region_init_io(&s->resource_control, OBJECT(dev),
                          &rvtng_resource_control_ops, s,
                          "rvtng.v0.resource-control",
                          memmap[RVTNG_V0_RESOURCE_CONTROL].size);
    memory_region_add_subregion(system_memory,
                                memmap[RVTNG_V0_RESOURCE_CONTROL].base,
                                &s->resource_control);

    /* Atomic-cache data window is currently modeled as fixed RAM. */
    if (!memory_region_init_ram(&s->atomic_cache, OBJECT(dev),
                                "rvtng.v0.atomic-cache",
                                memmap[RVTNG_V0_ATOMIC_CACHE].size, errp)) {
        return;
    }
    memory_region_add_subregion(system_memory,
                                memmap[RVTNG_V0_ATOMIC_CACHE].base,
                                &s->atomic_cache);

    for (unsigned core_idx = 0; core_idx < num_harts; core_idx++) {
        char shm_name[32];
        char region_name[32];
        void *uram;

        snprintf(shm_name, sizeof(shm_name), "/uram_shm_%u", core_idx);
        uram = frontend_get_shm_ptr(shm_name);
        snprintf(region_name, sizeof(region_name), "rvtng.v0.uram_%u",
                 core_idx);
        if (uram) {
            memory_region_init_ram_ptr(&s->uram[core_idx], OBJECT(dev),
                                       region_name,
                                       memmap[RVTNG_V0_URAM].size, uram);
        } else if (!memory_region_init_ram(&s->uram[core_idx], OBJECT(dev),
                                           region_name,
                                           memmap[RVTNG_V0_URAM].size,
                                           errp)) {
            return;
        }
        memory_region_add_subregion(
            system_memory,
            memmap[RVTNG_V0_URAM].base +
            core_idx * RVTNG_V0_URAM_STRIDE,
            &s->uram[core_idx]);
    }

    rvtng_v0_soc_reset_hold(OBJECT(s), RESET_TYPE_COLD);
    qemu_register_resettable(OBJECT(dev));
}

static void rvtng_v0_soc_unrealize(DeviceState *dev)
{
    qemu_unregister_resettable(OBJECT(dev));
}

int frontend_interrupt_update_service(int core_idx, int level)
{
    MachineState *machine;
    RVTNGV0State *s;

    if (core_idx < 0 || core_idx >= RVTNG_MAX_CALC_CORES) {
        return 0;
    }

    bql_lock();
    machine = MACHINE(qdev_get_machine());
    s = RVTNG_V0_MACHINE(machine);
    if (core_idx >= s->soc.cpus.num_harts) {
        bql_unlock();
        return 0;
    }
    qemu_set_irq(s->soc.tensor_irqs[core_idx], level != 0);
    bql_unlock();
    return 1;
}

static void rvtng_v0_soc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = rvtng_v0_soc_reset_hold;
    dc->realize = rvtng_v0_soc_realize;
    dc->unrealize = rvtng_v0_soc_unrealize;
    dc->user_creatable = false;
}

static void rvtng_v0_soc_instance_init(Object *obj)
{
    RVTNGV0SoCState *s = RVTNG_V0_SOC(obj);

    object_initialize_child(obj, "cpus", &s->cpus, TYPE_RISCV_HART_ARRAY);
    object_initialize_child(obj, "mmu", &s->mmu, TYPE_RVTNG_MMU);
}

static const TypeInfo rvtng_v0_soc_info = {
    .name = TYPE_RVTNG_V0_SOC,
    .parent = TYPE_DEVICE,
    .instance_size = sizeof(RVTNGV0SoCState),
    .instance_init = rvtng_v0_soc_instance_init,
    .class_init = rvtng_v0_soc_class_init,
};

static void rvtng_v0_soc_register_types(void)
{
    type_register_static(&rvtng_v0_soc_info);
}
type_init(rvtng_v0_soc_register_types)

static void rvtng_v0_machine_init(MachineState *machine)
{
    RVTNGV0State *s = RVTNG_V0_MACHINE(machine);
    const MemMapEntry *memmap = rvtng_v0_memmap;
    MemoryRegion *system_memory = get_system_memory();
    void *dram;

    /* Initialize SoC */
    object_initialize_child(OBJECT(machine), "soc", &s->soc,
                            TYPE_RVTNG_V0_SOC);
    qdev_realize(DEVICE(&s->soc), NULL, &error_fatal);

    /* Shared DRAM */
    dram = frontend_get_shm_ptr("/dram_shm_0");
    if (dram == NULL) {
        error_report("Unable to find shared dram pointer");
        exit(EXIT_FAILURE);
    }
    memory_region_init_ram_ptr(&s->dram, OBJECT(machine), "rvtng.dram",
                               machine->ram_size, dram);
    memory_region_add_subregion(system_memory,
                                memmap[RVTNG_V0_DRAM].base, &s->dram);

    /*
     * ClusterMMU (scalar ports 0..3).  Realized here rather than in the
     * SoC because the PTW reader binds the shared DRAM image, which only
     * exists once the region above is created.
     */
    object_property_set_link(OBJECT(&s->soc.mmu), "dram-mr",
                             OBJECT(&s->dram), &error_fatal);
    qdev_prop_set_uint64(DEVICE(&s->soc.mmu), "dram-base",
                         memmap[RVTNG_V0_DRAM].base);
    qdev_prop_set_uint64(DEVICE(&s->soc.mmu), "dram-size",
                         machine->ram_size);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->soc.mmu), &error_fatal)) {
        return;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->soc.mmu), 0,
                    memmap[RVTNG_V0_MMU].base);
    for (unsigned i = 0; i < machine->smp.cpus; i++) {
        CPUState *cs = cpu_by_arch_id(i);
        RISCVCPU *cpu = cs ? RISCV_CPU(cs) : NULL;

        if (!cpu) {
            error_report("rvtng-v0: ClusterMMU cannot find hart %u", i);
            return;
        }
        cpu->cluster_mmu = DEVICE(&s->soc.mmu);
        cpu->cluster_mmu_port = i & 3;
    }

    if (machine->firmware) {
        hwaddr firmware_entry = memmap[RVTNG_V0_ROM].base;

        riscv_find_and_load_firmware(machine, machine->firmware,
                                     &firmware_entry, NULL);
    } else {
        error_report("Unable to find the stage-0 Bootloader binary \"%s\"",
                     machine->firmware);
        exit(EXIT_FAILURE);
    }
}

static void rvtng_v0_machine_class_init(ObjectClass *klass, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(klass);
    static const char *const valid_cpu_types[] = {
        TYPE_RISCV_CPU_RVTNG_CALC_V0,
        NULL
    };

    mc->desc = "RVTNG Accelerator FPGA prototype compatible " \
               "QEMU Emulating Platform";
    mc->init = rvtng_v0_machine_init;
    mc->max_cpus = RVTNG_MAX_CALC_CORES;
    mc->default_cpu_type = TYPE_RISCV_CPU_RVTNG_CALC_V0;
    mc->valid_cpu_types = valid_cpu_types;
    mc->default_ram_id = "rvtng.v0.ram";
}

static const TypeInfo rvtng_v0_machine_info = {
    .name = TYPE_RVTNG_V0_MACHINE,
    .parent = TYPE_MACHINE,
    .instance_size = sizeof(RVTNGV0State),
    .class_init = rvtng_v0_machine_class_init,
};

static void rvtng_v0_machine_register_types(void)
{
    type_register_static(&rvtng_v0_machine_info);
}
type_init(rvtng_v0_machine_register_types)
