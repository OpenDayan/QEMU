/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (c) 2025 Institute of Computing Technology,
 * Chinese Academy of Sciences
 *
 * Provides a board compatible with the dayan-v0 SoC
 * XCVU13P FPGA prototype platform:
 *
 */

#ifndef HW_DAYAN_V0_H
#define HW_DAYAN_V0_H

#include "hw/boards.h"
#include "hw/irq.h"
#include "hw/riscv/riscv_hart.h"
#include "hw/riscv/dayan-intf.h"
#include "hw/riscv/dayan_fuse.h"
#include "hw/riscv/dayan_mmu.h"

typedef struct DayanV0SoCState {
    /*< private >*/
    DeviceState parent_obj;

    /*< public >*/
    RISCVHartArrayState cpus;
    qemu_irq tensor_irqs[DAYAN_MAX_CALC_CORES];
    MemoryRegion rom;
    MemoryRegion mailbox;
    MemoryRegion intc;
    uint32_t intc_status;
    uint32_t intc_enable;
    uint64_t intc_mailbox[DAYAN_INTC_MAILBOX_COUNT];
    MemoryRegion atomic_cache_control;
    MemoryRegion resource_control;
    uint64_t atomic_cache_status;
    uint64_t atomic_cache_backing_base;
    MemoryRegion clint_ipi;
    uint32_t clint_ipi_pending[DAYAN_MAX_CALC_CORES];
    uint32_t clint_ipi_enable[DAYAN_MAX_CALC_CORES];
    qemu_irq clint_msip_irq[DAYAN_MAX_CALC_CORES];
    uint32_t num_harts;
    MemoryRegion atomic_cache;
    MemoryRegion uram[DAYAN_MAX_CALC_CORES];
    DayanMmuState mmu;
} DayanV0SoCState;

#define TYPE_DAYAN_V0_SOC "dayan.v0.soc"
DECLARE_INSTANCE_CHECKER(DayanV0SoCState, DAYAN_V0_SOC,
                         TYPE_DAYAN_V0_SOC)

typedef struct DayanV0State {
    /*< private >*/
    MachineState parent_obj;

    /*< public >*/
    DayanV0SoCState soc;
    MemoryRegion dram;
} DayanV0State;

#define TYPE_DAYAN_V0_MACHINE MACHINE_TYPE_NAME("dayan-v0")
DECLARE_INSTANCE_CHECKER(DayanV0State, DAYAN_V0_MACHINE,
                         TYPE_DAYAN_V0_MACHINE)

enum {
    DAYAN_V0_MAILBOX,
    DAYAN_V0_INTERRUPT_HANDLER,
    DAYAN_V0_ATOMIC_CACHE_CONTROL,
    DAYAN_V0_MMU,
    DAYAN_V0_RESOURCE_CONTROL,
    DAYAN_V0_ROM,
    DAYAN_V0_CLINT,
    DAYAN_V0_ATOMIC_CACHE,
    DAYAN_V0_URAM,
    DAYAN_V0_DRAM,
};

#endif
