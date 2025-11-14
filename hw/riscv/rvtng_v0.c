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
#include "hw/riscv/riscv_hart.h"
#include "system/system.h"

static const MemMapEntry rvtng_v0_memap[] = {
    [RVTNG_V0_MAILBOX] = {     0x1000,        0x1000},
    [RVTNG_V0_ROM]     = {    0x10000,        0x1000},
    [RVTNG_V0_CLINT]   = {  0x2000000,       0xc0000},
    [RVTNG_V0_URAM]    = { 0x10000000,      0x100000}, // 1MB
    [RVTNG_V0_DRAM]    = { 0x80000000,           0x0},
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

    /* URAM */
    void *uram = frontend_get_shm_ptr("/uram_shm_0");
    if(uram == NULL) {
        uram = malloc(1024 * 1024);
    }
    memory_region_init_ram_ptr(&s->uram, OBJECT(dev), "rvtng.v0.uram",
                               memmap[RVTNG_V0_URAM].size, uram
                            );
    memory_region_add_subregion(system_memory,
                                memmap[RVTNG_V0_URAM].base, &s->uram
                            );
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
    memory_region_add_subregion(system_memory,
                                memmap[RVTNG_V0_DRAM].base,
                                machine->ram);

    if (machine->firmware) {
        hwaddr firmware_entry = memmap[RVTNG_V0_ROM].base;
        riscv_find_and_load_firmware(machine, machine->firmware, &firmware_entry, NULL);
    } else {
        error_report("Unable to find the stage-0 Bootloader binary \"%s\"",
                     machine->firmware);
        exit(1);
    }

    /* Init Fuse Hook */
    fuse_init();
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
