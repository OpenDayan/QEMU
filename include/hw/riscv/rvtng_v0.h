/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (c) 2025 Institute of Computing Technology, Chinese Academy of Sciences
 *
 * Provides a board compatible with the RVTNG-V0 SoC
 * XCVU13P FPGA prototype platform:
 *
 */

#ifndef HW_RVTNG_V0_H
#define HW_RVTNG_V0_H

#include "hw/boards.h"
#include "hw/riscv/riscv_hart.h"

#define RVTNG_MAX_CALC_CORES 1

typedef struct RVTNGV0SoCState {
    /*< private >*/
    DeviceState parent_obj;

    /*< public >*/
    RISCVHartArrayState cpus;
    MemoryRegion rom;
    MemoryRegion mailbox;
} RVTNGV0SoCState;

#define TYPE_RVTNG_V0_SOC "rvtng.v0.soc"
DECLARE_INSTANCE_CHECKER(RVTNGV0SoCState, RVTNG_V0_SOC,
                         TYPE_RVTNG_V0_SOC)

typedef struct RVTNGV0State {
    /*< private >*/
    MachineState parent_obj;

    /*< public >*/
    RVTNGV0SoCState soc;
} RVTNGV0State;

#define TYPE_RVTNG_V0_MACHINE MACHINE_TYPE_NAME("rvtng-v0")
DECLARE_INSTANCE_CHECKER(RVTNGV0State, RVTNG_V0_MACHINE,
                         TYPE_RVTNG_V0_MACHINE)

enum {
    RVTNG_V0_MAILBOX,
    RVTNG_V0_ROM,
    RVTNG_V0_UART0,
    RVTNG_V0_CLINT,
    RVTNG_V0_DRAM,
};

#endif
