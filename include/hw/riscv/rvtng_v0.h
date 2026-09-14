/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (c) 2025 Institute of Computing Technology,
 * Chinese Academy of Sciences
 *
 * Provides a board compatible with the RVTNG-V0 SoC
 * XCVU13P FPGA prototype platform:
 *
 */

#ifndef HW_RVTNG_V0_H
#define HW_RVTNG_V0_H

#include "hw/boards.h"
#include "hw/irq.h"
#include "hw/riscv/riscv_hart.h"
#include "hw/riscv/rvtng-intf.h"
#include "hw/riscv/rvtng_fuse.h"
#include "hw/riscv/rvtng_mmu.h"

typedef struct RVTNGV0SoCState {
    /*< private >*/
    DeviceState parent_obj;

    /*< public >*/
    RISCVHartArrayState cpus;
    qemu_irq tensor_irqs[RVTNG_MAX_CALC_CORES];
    MemoryRegion rom;
    MemoryRegion mailbox;
    MemoryRegion intc;
    uint32_t intc_status;
    uint32_t intc_enable;
    uint64_t intc_mailbox[RVTNG_INTC_MAILBOX_COUNT];
    MemoryRegion atomic_cache_control;
    MemoryRegion resource_control;
    uint64_t atomic_cache_status;
    uint64_t atomic_cache_backing_base;
    MemoryRegion clint_ipi;
    uint32_t clint_ipi_pending[RVTNG_MAX_CALC_CORES];
    uint32_t clint_ipi_enable[RVTNG_MAX_CALC_CORES];
    qemu_irq clint_msip_irq[RVTNG_MAX_CALC_CORES];
    uint32_t num_harts;
    MemoryRegion atomic_cache;
    MemoryRegion uram[RVTNG_MAX_CALC_CORES];
    RvtngMmuState mmu;
} RVTNGV0SoCState;

#define TYPE_RVTNG_V0_SOC "rvtng.v0.soc"
DECLARE_INSTANCE_CHECKER(RVTNGV0SoCState, RVTNG_V0_SOC,
                         TYPE_RVTNG_V0_SOC)

typedef struct RVTNGV0State {
    /*< private >*/
    MachineState parent_obj;

    /*< public >*/
    RVTNGV0SoCState soc;
    MemoryRegion dram;
} RVTNGV0State;

#define TYPE_RVTNG_V0_MACHINE MACHINE_TYPE_NAME("rvtng-v0")
DECLARE_INSTANCE_CHECKER(RVTNGV0State, RVTNG_V0_MACHINE,
                         TYPE_RVTNG_V0_MACHINE)

enum {
    RVTNG_V0_MAILBOX,
    RVTNG_V0_INTERRUPT_HANDLER,
    RVTNG_V0_ATOMIC_CACHE_CONTROL,
    RVTNG_V0_MMU,
    RVTNG_V0_RESOURCE_CONTROL,
    RVTNG_V0_ROM,
    RVTNG_V0_CLINT,
    RVTNG_V0_ATOMIC_CACHE,
    RVTNG_V0_URAM,
    RVTNG_V0_DRAM,
};

#endif
