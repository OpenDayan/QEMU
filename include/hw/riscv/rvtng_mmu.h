/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * RVTNG-V0 ClusterMMU (scalar side) SysBus device
 *
 * One instance models the ClusterMMU of one cluster: the APB4 register
 * image, the shared L2/PDC caches and the 8-port translation engine.
 * The QEMU vCPUs (scalar ports 0..3) run their load/store/fetch
 * addresses through rvtng_cluster_mmu_xlate(); the CModel tensor LSU
 * ports 4..7 keep their own per-process instance.
 */

#ifndef HW_RISCV_RVTNG_MMU_H
#define HW_RISCV_RVTNG_MMU_H

#include "hw/sysbus.h"
#include "exec/memattrs.h"
#include "hw/riscv/rvtng_mmu_core.h"
#include "qemu/queue.h"

#define TYPE_RVTNG_MMU "rvtng.v0.mmu"
OBJECT_DECLARE_SIMPLE_TYPE(RvtngMmuState, RVTNG_MMU)

/* Marker nibble in MemTxAttrs._reserved1 identifying entries the
 * ClusterMMU installed into the redirect-shim window. */
#define RVTNG_MMU_REDIRECT_ATTR_MARK 0x5

typedef struct RvtngMmuState {
    /*< private >*/
    SysBusDevice parent_obj;

    /*< public >*/
    MemoryRegion iomem;
    MemoryRegion redirect_iomem;
    QemuMutex lock;

    /* Functional model state (guarded by @lock). */
    RvtngMmuCore core;

    /* Properties. */
    uint64_t dram_base;        /* bus base of the DRAM image          */
    uint64_t dram_size;        /* real shm size (bounds PTW reads)    */
    bool eager_tlb_flush;      /* flush QEMU TLBs on every config write */
    bool forward_config;       /* mirror config writes to the CModel  */
    char *translate_scope;     /* "all" | "dram-uram"                 */

    /* Derived from translate-scope at realize time. */
    bool scope_all;

    MemoryRegion *dram_mr;     /* link, owned by the machine          */
} RvtngMmuState;

/*
 * Translate one CPU bus address through the ClusterMMU.  Returns the
 * physical address to access; on translation fault *@fault is set and
 * the caller must not map the result directly (use
 * rvtng_mmu_redirect_tlb_pa() instead), because a redirect replaces the
 * master AR/AW address as a whole and cannot be expressed as an
 * ordinary page mapping.
 *
 * dev == NULL passes the address through unchanged.
 */
uint64_t rvtng_cluster_mmu_xlate(DeviceState *dev, uint64_t gpa,
                                 RvtngMmuAccess acc, unsigned port,
                                 RvtngMmuFault *fault);

/*
 * Page-granular PA to install in the vCPU TLB for a faulting access of
 * the given class, together with rvtng_mmu_redirect_tlb_attrs(): the
 * entry points into the private shim window at an offset that encodes
 * the full ClusterMMU VA, whose handler re-runs the translation per
 * access (a cached page mapping would re-add the original page
 * offset).  The attributes are what distinguishes a redirected entry
 * from an ordinary PA translation onto the window; unmarked accesses
 * are rejected like unassigned memory.
 */
hwaddr rvtng_mmu_redirect_tlb_pa(DeviceState *dev, RvtngMmuFault fault,
                                 uint64_t va);
MemTxAttrs rvtng_mmu_redirect_tlb_attrs(unsigned port, RvtngMmuAccess acc,
                                        uint64_t va);

#endif /* HW_RISCV_RVTNG_MMU_H */
