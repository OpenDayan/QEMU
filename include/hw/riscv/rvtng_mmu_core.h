/*
 * RVTNG ClusterMMU functional model - pure C core
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Single behavioral implementation of the ClusterMMU, shared by the QEMU
 * rvtng_mmu device, the Dayan-emu C++ shell (emu/mmu.cpp) and the
 * golden-vector suites on both sides.  RTL boundary behaviors (FixMap
 * 39-bit arithmetic, PDC non-G-restoring hits, ERROR_SLV_MASK stored but
 * unconsumed, negative entries only for PTW faults) are modeled here.
 * The file deliberately includes no QEMU headers so that it can also be
 * compiled stand-alone by tests/unit and shared with other simulators.
 *
 * Behavioral baseline:
 *   - 56-bit physical address space, Sv39 39-bit virtual addresses.
 *   - Only bus addresses at/above RVTNG_MMU_DRAM_BASE (2 GiB) cross the
 *     translation engine; lower addresses (URAM aperture, control space,
 *     the APB page) pass through identity on every port.
 *   - Lookup order: FixMap -> per-port L1 -> shared L2 -> blocking PTW.
 *   - Faults do not abort an access; they redirect it to a fault-specific
 *     base address, exactly like the hardware master AR/AW substitution.
 */

#ifndef HW_RISCV_RVTNG_MMU_CORE_H
#define HW_RISCV_RVTNG_MMU_CORE_H

#include <stdbool.h>
#include <stdint.h>

#define RVTNG_MMU_PA_MASK        0x00ffffffffffffffull
#define RVTNG_MMU_VA_MASK        ((1ull << 39) - 1)
#define RVTNG_MMU_PAGE_MASK      0xfffull
#define RVTNG_MMU_DRAM_BASE      0x80000000ull
#define RVTNG_MMU_DEFAULT_DRAM_SIZE (4ull * 1024 * 1024 * 1024)

/* The APB4 register image occupies a 4 KiB page. */
#define RVTNG_MMU_APB_SIZE       0x1000

/* ClusterMMU port topology: 0..3 scalar, 4..7 tensor. */
#define RVTNG_MMU_NUM_PORTS      8
#define RVTNG_MMU_SCALAR_PORTS   4
#define RVTNG_MMU_L1_ENTRIES_SCALAR 16
#define RVTNG_MMU_L1_ENTRIES_TENSOR 32
#define RVTNG_MMU_L1_ENTRIES_MAX    RVTNG_MMU_L1_ENTRIES_TENSOR

typedef enum {
    RVTNG_MMU_READ = 0,
    RVTNG_MMU_WRITE = 1,
    RVTNG_MMU_EXECUTE = 2,
} RvtngMmuAccess;

typedef enum {
    RVTNG_MMU_FAULT_NONE = 0,
    RVTNG_MMU_FAULT_PTE_INVALID = 1,
    RVTNG_MMU_FAULT_READ_PERM = 2,
    RVTNG_MMU_FAULT_WRITE_PERM = 3,
    RVTNG_MMU_FAULT_EXEC_PERM = 4,
    RVTNG_MMU_FAULT_ILLEGAL_PTE = 5,
    RVTNG_MMU_FAULT_PTW_BUS_ERROR = 9,
    RVTNG_MMU_FAULT_PTW_TIMEOUT = 10,
} RvtngMmuFault;

typedef enum {
    RVTNG_MMU_PAGE_4K = 0,
    RVTNG_MMU_PAGE_2M = 1,
    RVTNG_MMU_PAGE_1G = 2,
} RvtngMmuPageSize;

/* PTW read response codes, mirroring the AXI RRESP handling. */
typedef enum {
    RVTNG_MMU_PTE_OKAY = 0,
    RVTNG_MMU_PTE_BUS_ERROR = 1,
    RVTNG_MMU_PTE_TIMEOUT = 2,
} RvtngMmuPteResponse;

typedef struct RvtngMmuPteRead {
    RvtngMmuPteResponse response;
    uint64_t data;
} RvtngMmuPteRead;

/* Optional injected PTW transport; default is the bound DRAM image. */
typedef RvtngMmuPteRead (*RvtngMmuPteReaderFn)(void *opaque, uint64_t pa);

/*
 * APB4 register offsets (12-bit intra-page addresses).  The golden
 * vector suites (tests/unit/test-rvtng-mmu.c and
 * dayan-emu/tests/mmu_test.cpp) guard this table against RTL drift.
 */
#define RVTNG_MMU_REG_PTW_BASE_LO          0x000
#define RVTNG_MMU_REG_PTW_BASE_HI          0x004
#define RVTNG_MMU_REG_FIX_MAP_BASE_LO      0x008
#define RVTNG_MMU_REG_FIX_MAP_BASE_HI      0x00c
#define RVTNG_MMU_REG_N_FIX_MAP_EN         0x010
#define RVTNG_MMU_REG_TLB_FLUSH            0x014
#define RVTNG_MMU_REG_FLUSH_ACK            0x018
#define RVTNG_MMU_REG_FLUSH_DONE           0x01c
#define RVTNG_MMU_REG_FLUSH_BUSY           0x020
#define RVTNG_MMU_REG_FLUSH_ERR_CODE       0x024
#define RVTNG_MMU_REG_GLOBAL_ASID          0x028
#define RVTNG_MMU_REG_ERROR_SLV_MASK_LO    0x048
#define RVTNG_MMU_REG_ERROR_SLV_MASK_HI    0x04c
#define RVTNG_MMU_REG_ERROR_SLV_OFF_MODE   0x050
#define RVTNG_MMU_REG_ERROR_SLV_CFG_ERR    0x070
#define RVTNG_MMU_REG_ERROR_SLV_CFG_CLR    0x074
#define RVTNG_MMU_REG_REDIR_PAGE_FAULT_LO  0x080
#define RVTNG_MMU_REG_REDIR_PAGE_FAULT_HI  0x084
#define RVTNG_MMU_REG_REDIR_READ_PERM_LO   0x088
#define RVTNG_MMU_REG_REDIR_READ_PERM_HI   0x08c
#define RVTNG_MMU_REG_REDIR_WRITE_PERM_LO  0x090
#define RVTNG_MMU_REG_REDIR_WRITE_PERM_HI  0x094
#define RVTNG_MMU_REG_REDIR_EXEC_PERM_LO   0x098
#define RVTNG_MMU_REG_REDIR_EXEC_PERM_HI   0x09c
#define RVTNG_MMU_REG_REDIR_PTW_ERR_LO     0x0a0
#define RVTNG_MMU_REG_REDIR_PTW_ERR_HI     0x0a4
#define RVTNG_MMU_REG_REDIR_DEFAULT_LO     0x0a8
#define RVTNG_MMU_REG_REDIR_DEFAULT_HI     0x0ac
#define RVTNG_MMU_REG_FIX_MAP_SIZE_LO      0x0b0
#define RVTNG_MMU_REG_FIX_MAP_SIZE_HI      0x0b4

/*
 * Classification used by the QEMU device to decide which accepted APB
 * writes are mirrored to the Dayan-emu side (see docs qemu-MMU guide
 * section 6.10.3): translation-semantic configuration goes out as a raw
 * register write, flush events are reported as their own message class
 * and pure status registers are never forwarded.
 */
typedef enum {
    RVTNG_MMU_FWD_NONE = 0,      /* status / undefined: do not forward  */
    RVTNG_MMU_FWD_REG_WRITE,     /* translation-semantic config write   */
    RVTNG_MMU_FWD_FLUSH,         /* TLB_FLUSH trigger                   */
} RvtngMmuFwdClass;

typedef struct RvtngMmuTlbEntry {
    bool valid;
    bool negative;
    uint8_t fault;                 /* RvtngMmuFault                    */
    uint64_t vpn;                  /* 27-bit                           */
    uint64_t ppn;                  /* 44-bit                           */
    uint8_t asid;
    uint8_t page_size;             /* RvtngMmuPageSize                 */
    bool global;
    bool r, w, x, u, a, d;
} RvtngMmuTlbEntry;

typedef struct RvtngMmuPdcEntry {
    bool valid;
    uint16_t vpn2, vpn1;
    uint8_t asid;
    bool global;
    uint64_t ppn;
} RvtngMmuPdcEntry;

/*
 * Extended translate provenance: which PTE bus reads the page walk
 * issued, the leaf PTE observed, and whether the result came from the
 * fault redirect path.  Plain rvtng_mmu_core_translate() discards
 * these; the Dayan-emu's translation result exposes them.
 */
typedef struct RvtngMmuTranslateDetail {
    uint64_t walk_addresses[3];
    unsigned walk_depth;
    uint64_t pte_snapshot;
    bool global_mapping;
    bool redirected;
} RvtngMmuTranslateDetail;

typedef struct RvtngMmuCore {
    /* Config image (what the APB registers alias). */
    uint64_t ptw_base;
    uint64_t fixmap_base;
    uint64_t fixmap_size_pages;
    bool n_fixmap_en;
    uint8_t global_asid;
    uint64_t redir[6];             /* page/read/write/exec/ptw/default */
    bool redir_offset_enable;
    uint64_t error_slv_mask;       /* stored only, never consumed      */

    /* Lookup structures. */
    RvtngMmuTlbEntry l1[RVTNG_MMU_NUM_PORTS][RVTNG_MMU_L1_ENTRIES_MAX];
    RvtngMmuTlbEntry l2_4k[128];                 /* 128 sets x 1 way  */
    RvtngMmuTlbEntry l2_2m[256 * 4];             /* 256 sets x 4 ways */
    RvtngMmuTlbEntry l2_1g[128];                 /* 128 sets x 1 way  */
    RvtngMmuPdcEntry pdc_l0[16];
    RvtngMmuPdcEntry pdc_l1[16];
    uint16_t l1_lfsr;
    uint8_t repl_lfsr;

    /* Flush status bits (APB view). */
    bool flush_done;
    bool flush_busy;
    uint8_t flush_error_code;

    /* PTW transport: bound DRAM image, or an injected reader. */
    RvtngMmuPteReaderFn reader;
    void *reader_opaque;
    const void *dram_host;
    uint64_t dram_base;
    uint64_t dram_size;

    /* Statistics (best effort, not architectural). */
    uint64_t stat_xlate;
    uint64_t stat_redirect;
    uint64_t stat_fault[16];
} RvtngMmuCore;

#ifdef __cplusplus
extern "C" {
#endif

/* Reset all architectural and cache state to the power-on image
 * (FixMap full-range identity, empty caches). */
void rvtng_mmu_core_init(RvtngMmuCore *c);

/* Bind the shared DRAM image used by PTW reads.  PTW reads outside
 * [base, base + size - 8] return bus_error, matching the Dayan-emu
 * make_dram_reader(). */
void rvtng_mmu_core_bind_dram(RvtngMmuCore *c, const void *host,
                              uint64_t base, uint64_t size);
void rvtng_mmu_core_set_reader(RvtngMmuCore *c, RvtngMmuPteReaderFn fn,
                               void *opaque);

/* One PTW bus read against the bound DRAM image.  Exposed so callers
 * that only need the DRAM path need not duplicate its bounds. */
RvtngMmuPteRead rvtng_mmu_core_dram_read(const RvtngMmuCore *c, uint64_t pa);

/* APB4 register image.  write_reg returns false only for an unsupported
 * address; alignment-constrained writes are accepted but silently
 * ignored, matching the RTL. */
bool rvtng_mmu_core_write_reg(RvtngMmuCore *c, uint16_t off, uint32_t val);
uint32_t rvtng_mmu_core_read_reg(const RvtngMmuCore *c, uint16_t off);
RvtngMmuFwdClass rvtng_mmu_core_classify_write(uint16_t off);

/*
 * Translate one access.  The configuration image is snapshotted at call
 * entry (software equivalent of the slave AR/AW acceptance handshake).
 * On fault the returned physical address is the redirect PA, so callers
 * can keep treating the result as a plain address.  @fault and
 * @page_size may be NULL.
 */
uint64_t rvtng_mmu_core_translate(RvtngMmuCore *c, uint64_t va,
                                  RvtngMmuAccess acc, unsigned port,
                                  RvtngMmuFault *fault,
                                  RvtngMmuPageSize *page_size);

/*
 * Translate with explicit provenance and an optional caller-pinned
 * config image.  @cfg == NULL snapshots @c at entry, like
 * rvtng_mmu_core_translate().  A non-NULL @cfg is the two-phase
 * handshake form: config fields (ptw_base, FixMap, ASID, redirect
 * bases) are read from it after masking into a local copy, while TLB,
 * PDC and the PTW transport always come from @c.  @detail, if given,
 * receives the walk provenance of this access.
 */
uint64_t rvtng_mmu_core_translate_full(RvtngMmuCore *c,
                                       const RvtngMmuCore *cfg,
                                       uint64_t va, RvtngMmuAccess acc,
                                       unsigned port,
                                       RvtngMmuFault *fault_out,
                                       RvtngMmuPageSize *page_size_out,
                                       RvtngMmuTranslateDetail *detail_out);

/* Flush semantics mirror the Dayan-emu: full or ASID-selective clear,
 * global entries survive ASID flushes, sticky DONE/ERR status. */
void rvtng_mmu_core_flush(RvtngMmuCore *c, bool asid_sel, uint8_t asid);
void rvtng_mmu_core_ack_flush(RvtngMmuCore *c);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* HW_RISCV_RVTNG_MMU_CORE_H */
