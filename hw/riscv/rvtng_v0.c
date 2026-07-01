/*
 * Copyright (c) 2025 Institute of Computing Technology, Chinese Academy of Sciences
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Provides a board compatible with the RVTNG-V0 SoC
 * XCVU13P FPGA prototype platform:
 *
 * 0) Mailbox Register
 * 1) Boot Rom (Stage-1)
 * 2) CLINT (Core-Local Interruptor)
 * 3) PCIE XDMA Engine (DMA Copy-In / Copy-Out)
 * 4) DRAM
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
#include "qapi/error.h"
#include "system/address-spaces.h"
#include "hw/boards.h"
#include "hw/char/serial-mm.h"
#include "hw/intc/riscv_aclint.h"
#include "hw/qdev-properties.h"
#include "hw/riscv/boot.h"
#include "hw/riscv/rvtng_v0.h"
#include "hw/riscv/rvtng-intf.h"
#include "hw/riscv/rvtng_fuse.h"
#include "hw/riscv/riscv_hart.h"
#include "system/system.h"

static const MemMapEntry rvtng_v0_memap[] = {
    [RVTNG_V0_MAILBOX] = {     0x1000,        0x1000},
    [RVTNG_V0_ROM]     = {    0x10000,        0x1000},
    [RVTNG_V0_CLINT]   = {    0x20000,       0x10000},
    [RVTNG_V0_URAM]    = {   0x800000,      0x100000}, // 1MB
    [RVTNG_V0_DRAM]    = { 0x80000000,   0x100000000UL},
};

static void rvtng_intc_emit_rising_irq(uint32_t prev_status, uint32_t prev_enable,
                                       uint32_t new_status, uint32_t new_enable)
{
    uint32_t prev_irq = prev_status & prev_enable;
    uint32_t new_irq = new_status & new_enable;
    uint32_t rising = new_irq & ~prev_irq;
    for (int i = 0; i < RVTNG_MAX_CALC_CORES && i < RVTNG_INTC_MAILBOX_COUNT; i++) {
        if (rising & (1u << i)) {
            printf("QEMU INTC: irq rising core=%d mask=0x%x\n", i, rising);
            rvtng_fuse_emit_event(i, 1);
        }
    }
}

/* INTC MMIO write bridge:
 * - xdma0_user and guest MMIO access this region directly
 * - rising edge on (ENABLE & STATUS) emits one host event token via FUSE
 */
static void rvtng_intc_write(void *opaque, hwaddr offset, uint64_t value,
                             unsigned size)
{
    RVTNGV0SoCState *s = (RVTNGV0SoCState *)opaque;
    uint32_t prev_status = s->intc_status;
    uint32_t prev_enable = s->intc_enable;

    if (offset == RVTNG_INTC_ENABLE_OFF && size == 4) {
        s->intc_enable = (uint32_t)value;
        printf("QEMU INTC: write ENABLE=0x%x\n", s->intc_enable);
    } else if (offset == RVTNG_INTC_SET_OFF && size == 4) {
        s->intc_status |= (uint32_t)value;
        printf("QEMU INTC: write SET mask=0x%x new_status=0x%x\n",
               (uint32_t)value, s->intc_status);
    } else if (offset == RVTNG_INTC_CLEAR_OFF && size == 4) {
        s->intc_status &= ~((uint32_t)value);
        printf("QEMU INTC: write CLEAR mask=0x%x new_status=0x%x\n",
               (uint32_t)value, s->intc_status);
    } else if (offset >= RVTNG_INTC_MAILBOX0_OFF &&
               offset < RVTNG_INTC_MAILBOX0_OFF + RVTNG_INTC_MAILBOX_COUNT * sizeof(uint64_t)) {
        hwaddr rel = offset - RVTNG_INTC_MAILBOX0_OFF;
        uint32_t idx = (uint32_t)(rel / sizeof(uint64_t));
        uint32_t pos = (uint32_t)(rel % sizeof(uint64_t));
        if (size == 8 && pos == 0) {
            s->intc_mailbox[idx] = value;
        } else if (size == 4 && (pos == 0 || pos == 4)) {
            uint64_t old = s->intc_mailbox[idx];
            if (pos == 0) {
                s->intc_mailbox[idx] = (old & 0xffffffff00000000ULL) | (value & 0xffffffffULL);
            } else {
                s->intc_mailbox[idx] = (old & 0x00000000ffffffffULL) | ((value & 0xffffffffULL) << 32);
            }
        } else {
            printf("QEMU INTC: invalid mailbox write offset=0x%lx size=%u\n",
                   (unsigned long)offset, size);
            return;
        }
    } else {
        printf("QEMU INTC: ignored write offset=0x%lx size=%u value=0x%lx\n",
               (unsigned long)offset, size, (unsigned long)value);
        return;
    }

    rvtng_intc_emit_rising_irq(prev_status, prev_enable, s->intc_status, s->intc_enable);
}

static uint64_t rvtng_intc_read(void *opaque, hwaddr offset, unsigned size)
{
    RVTNGV0SoCState *s = (RVTNGV0SoCState *)opaque;
    if (offset == RVTNG_INTC_STATUS_OFF && size == 4) {
        return s->intc_status;
    }
    if (offset == RVTNG_INTC_ENABLE_OFF && size == 4) {
        return s->intc_enable;
    }
    if (offset >= RVTNG_INTC_MAILBOX0_OFF &&
        offset < RVTNG_INTC_MAILBOX0_OFF + RVTNG_INTC_MAILBOX_COUNT * sizeof(uint64_t)) {
        hwaddr rel = offset - RVTNG_INTC_MAILBOX0_OFF;
        uint32_t idx = (uint32_t)(rel / sizeof(uint64_t));
        uint32_t pos = (uint32_t)(rel % sizeof(uint64_t));
        if (size == 8 && pos == 0) {
            return s->intc_mailbox[idx];
        }
        if (size == 4 && (pos == 0 || pos == 4)) {
            return (s->intc_mailbox[idx] >> (pos * 8)) & 0xffffffffULL;
        }
    }
    return 0;
}

static const MemoryRegionOps rvtng_intc_ops = {
    .read = rvtng_intc_read,
    .write = rvtng_intc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 8 },
};

// Global Intf for inter-call
AddressSpace *fuse_as;

static void rvtng_v0_soc_realize(DeviceState *dev, Error **errp)
{
    MachineState *ms = MACHINE(qdev_get_machine());
    RVTNGV0SoCState *s = RVTNG_V0_SOC(dev);
    const MemMapEntry *memmap = rvtng_v0_memap;
    MemoryRegion *system_memory = get_system_memory();
    uint32_t num_harts = ms->smp.cpus;

    fuse_as = &address_space_memory;

    qdev_prop_set_uint32(DEVICE(&s->cpus), "num-harts", num_harts);
    qdev_prop_set_uint32(DEVICE(&s->cpus), "hartid-base", 0);
    qdev_prop_set_string(DEVICE(&s->cpus), "cpu-type",
                         TYPE_RISCV_CPU_RVTNG_CALC_V0);
    qdev_prop_set_uint64(DEVICE(&s->cpus), "resetvec", memmap[RVTNG_V0_ROM].base);
    sysbus_realize(SYS_BUS_DEVICE(&s->cpus), &error_fatal);

    /* CLINT */
    riscv_aclint_swi_create(memmap[RVTNG_V0_CLINT].base,
                            0, num_harts, false);
    riscv_aclint_mtimer_create(memmap[RVTNG_V0_CLINT].base +
                               RISCV_ACLINT_SWI_SIZE,
                               RISCV_ACLINT_DEFAULT_MTIMER_SIZE,
                               0, num_harts, RISCV_ACLINT_DEFAULT_MTIMECMP,
                               RISCV_ACLINT_DEFAULT_MTIME,
                               1000000, true);

    /* ROM */
    memory_region_init_rom(&s->rom, OBJECT(dev), "rvtng.v0.rom",
                           memmap[RVTNG_V0_ROM].size, &error_fatal);
    memory_region_add_subregion(system_memory,
                                memmap[RVTNG_V0_ROM].base, &s->rom);

    /* MAILBOX */
    memory_region_init_ram(&s->mailbox, OBJECT(dev), "rvtng.v0.mailbox",
                           memmap[RVTNG_V0_MAILBOX].size, &error_fatal);
    memory_region_add_subregion(system_memory,
                                memmap[RVTNG_V0_MAILBOX].base, &s->mailbox);

    s->intc_status = 0;
    s->intc_enable = 0;
    memset(s->intc_mailbox, 0, sizeof(s->intc_mailbox));

    /* INTC window for NPU->host interrupt signaling and xdma0_user MMIO view. */
    memory_region_init_io(&s->intc, OBJECT(dev), &rvtng_intc_ops, s,
                         "rvtng.v0.intc", RVTNG_INTC_SIZE);
    memory_region_add_subregion(system_memory, RVTNG_INTC_BASE, &s->intc);
    printf("QEMU INTC: mapped base=0x%lx size=0x%lx\n",
           (unsigned long)RVTNG_INTC_BASE, (unsigned long)RVTNG_INTC_SIZE);

    for(int core_idx = 0; core_idx < num_harts; core_idx ++) {
        char name[128];
        /* URAM */
        snprintf(name, sizeof(name), "/uram_shm_%d", core_idx);
        void *uram = frontend_get_shm_ptr(name);
        if(uram == NULL) {
            uram = malloc(1024 * 1024);
        }
        snprintf(name, sizeof(name), "rvtng.v0.uram_%d", core_idx);
        memory_region_init_ram_ptr(&s->uram[core_idx], OBJECT(dev), name,
                                memmap[RVTNG_V0_URAM].size, uram
                                );
        memory_region_add_subregion(system_memory,
                                    memmap[RVTNG_V0_URAM].base +
                                    core_idx * memmap[RVTNG_V0_URAM].size, &s->uram[core_idx]
                                );
    }
}

static void rvtng_v0_soc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = rvtng_v0_soc_realize;
    dc->user_creatable = false;
}

static void rvtng_v0_soc_instance_init(Object *obj)
{
    RVTNGV0SoCState *s = RVTNG_V0_SOC(obj);

    object_initialize_child(obj, "cpus", &s->cpus, TYPE_RISCV_HART_ARRAY);
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

extern int fuse_init(void);
static void rvtng_v0_machine_init(MachineState *machine)
{
    RVTNGV0State *s = RVTNG_V0_MACHINE(machine);
    const MemMapEntry *memmap = rvtng_v0_memap;
    MemoryRegion *system_memory = get_system_memory();

    /* Initialize SoC */
    object_initialize_child(OBJECT(machine), "soc", &s->soc,
                            TYPE_RVTNG_V0_SOC);
    qdev_realize(DEVICE(&s->soc), NULL, &error_fatal);

    /* Register RAM */
    // memory_region_add_subregion(system_memory,
    //                             memmap[RVTNG_V0_DRAM].base,
    //                             machine->ram);

    /* Shared DRAM */
    void *dram = frontend_get_shm_ptr("/dram_shm_0");
    if(dram == NULL) {
        error_report("Unable to find shared dram pointer");
        exit(1);
        dram = malloc(1024 * 1024);
    }
    memory_region_init_ram_ptr(&s->dram, OBJECT(machine), "rvtng.dram",
                               16ul * 1024ul * 1024ul * 1024ul, dram
                            );
    memory_region_add_subregion(system_memory,
                                memmap[RVTNG_V0_DRAM].base, &s->dram
                            );

    if (machine->firmware) {
        hwaddr firmware_entry = memmap[RVTNG_V0_ROM].base;
        riscv_find_and_load_firmware(machine, machine->firmware, &firmware_entry, NULL);
    } else {
        error_report("Unable to find the stage-0 Bootloader binary \"%s\"",
                     machine->firmware);
        exit(1);
    }

    /* FUSE hook is initialized by the dl launcher before qemu_init(). */
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
