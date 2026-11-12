/*
 * Dayan ClusterMMU functional model - pure C core
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Single behavioral implementation of the ClusterMMU, shared by the QEMU
 * dayan_mmu device, the Dayan-emu C++ shell (emu/mmu.cpp) and the
 * golden-vector suites on both sides.  RTL boundary behaviors (FixMap
 * 39-bit arithmetic, PDC non-G-restoring hits, ERROR_SLV_MASK stored but
 * unconsumed, negative entries only for PTW faults) are modeled here.
 * The file deliberately includes no QEMU headers so that it can also be
 * compiled stand-alone by tests/unit and shared with other simulators.
 *
 * Behavioral baseline:
 *   - 56-bit physical address space, Sv39 39-bit virtual addresses.
 *   - Only bus addresses at/above DAYAN_MMU_DRAM_BASE (2 GiB) cross the
 *     translation engine; lower addresses (URAM aperture, control space,
 *     the APB page) pass through identity on every port.
 *   - Lookup order: FixMap -> per-port L1 -> shared L2 -> blocking PTW.
 *   - Faults do not abort an access; they redirect it to a fault-specific
 *     base address, exactly like the hardware master AR/AW substitution.
 */

#ifndef HW_RISCV_DAYAN_MMU_CORE_H
#define HW_RISCV_DAYAN_MMU_CORE_H

#include <stdbool.h>
#include <stdint.h>

#define DAYAN_MMU_PA_MASK        0x00ffffffffffffffull
#define DAYAN_MMU_VA_MASK        ((1ull << 39) - 1)
#define DAYAN_MMU_PAGE_MASK      0xfffull
#define DAYAN_MMU_DRAM_BASE      0x80000000ull
#define DAYAN_MMU_DEFAULT_DRAM_SIZE (4ull * 1024 * 1024 * 1024)

/* The APB4 register image occupies a 4 KiB page. */
#define DAYAN_MMU_APB_SIZE       0x1000

/* ClusterMMU port topology: 0..3 scalar, 4..7 tensor. */
#define DAYAN_MMU_NUM_PORTS      8
#define DAYAN_MMU_SCALAR_PORTS   4
#define DAYAN_MMU_L1_ENTRIES_SCALAR 16
#define DAYAN_MMU_L1_ENTRIES_TENSOR 32
#define DAYAN_MMU_L1_ENTRIES_MAX    DAYAN_MMU_L1_ENTRIES_TENSOR

typedef enum {
    DAYAN_MMU_READ = 0,
    DAYAN_MMU_WRITE = 1,
    DAYAN_MMU_EXECUTE = 2,
} DayanMmuAccess;

typedef enum {
    DAYAN_MMU_FAULT_NONE = 0,
    DAYAN_MMU_FAULT_PTE_INVALID = 1,
    DAYAN_MMU_FAULT_READ_PERM = 2,
    DAYAN_MMU_FAULT_WRITE_PERM = 3,
    DAYAN_MMU_FAULT_EXEC_PERM = 4,
    DAYAN_MMU_FAULT_ILLEGAL_PTE = 5,
    DAYAN_MMU_FAULT_PTW_BUS_ERROR = 9,
    DAYAN_MMU_FAULT_PTW_TIMEOUT = 10,
} DayanMmuFault;

typedef enum {
    DAYAN_MMU_PAGE_4K = 0,
    DAYAN_MMU_PAGE_2M = 1,
    DAYAN_MMU_PAGE_1G = 2,
} DayanMmuPageSize;

/* PTW read response codes, mirroring the AXI RRESP handling. */
typedef enum {
    DAYAN_MMU_PTE_OKAY = 0,
    DAYAN_MMU_PTE_BUS_ERROR = 1,
    DAYAN_MMU_PTE_TIMEOUT = 2,
} DayanMmuPteResponse;

typedef struct DayanMmuPteRead {
    DayanMmuPteResponse response;
    uint64_t data;
} DayanMmuPteRead;

/* Optional injected PTW transport; default is the bound DRAM image. */
typedef DayanMmuPteRead (*DayanMmuPteReaderFn)(void *opaque, uint64_t pa);

/*
 * APB4 register offsets (12-bit intra-page addresses).  The golden
 * vector suites (tests/unit/test-dayan-mmu.c and
 * dayan-emu/tests/mmu_test.cpp) guard this table against RTL drift.
 */
#define DAYAN_MMU_REG_PTW_BASE_LO          0x000
#define DAYAN_MMU_REG_PTW_BASE_HI          0x004
#define DAYAN_MMU_REG_FIX_MAP_BASE_LO      0x008
#define DAYAN_MMU_REG_FIX_MAP_BASE_HI      0x00c
#define DAYAN_MMU_REG_N_FIX_MAP_EN         0x010
#define DAYAN_MMU_REG_TLB_FLUSH            0x014
#define DAYAN_MMU_REG_FLUSH_ACK            0x018
#define DAYAN_MMU_REG_FLUSH_DONE           0x01c
#define DAYAN_MMU_REG_FLUSH_BUSY           0x020
#define DAYAN_MMU_REG_FLUSH_ERR_CODE       0x024
#define DAYAN_MMU_REG_GLOBAL_ASID          0x028
#define DAYAN_MMU_REG_ERROR_SLV_MASK_LO    0x048
#define DAYAN_MMU_REG_ERROR_SLV_MASK_HI    0x04c
#define DAYAN_MMU_REG_ERROR_SLV_OFF_MODE   0x050
#define DAYAN_MMU_REG_ERROR_SLV_CFG_ERR    0x070
#define DAYAN_MMU_REG_ERROR_SLV_CFG_CLR    0x074
#define DAYAN_MMU_REG_REDIR_PAGE_FAULT_LO  0x080
#define DAYAN_MMU_REG_REDIR_PAGE_FAULT_HI  0x084
#define DAYAN_MMU_REG_REDIR_READ_PERM_LO   0x088
#define DAYAN_MMU_REG_REDIR_READ_PERM_HI   0x08c
#define DAYAN_MMU_REG_REDIR_WRITE_PERM_LO  0x090
#define DAYAN_MMU_REG_REDIR_WRITE_PERM_HI  0x094
#define DAYAN_MMU_REG_REDIR_EXEC_PERM_LO   0x098
#define DAYAN_MMU_REG_REDIR_EXEC_PERM_HI   0x09c
#define DAYAN_MMU_REG_REDIR_PTW_ERR_LO     0x0a0
#define DAYAN_MMU_REG_REDIR_PTW_ERR_HI     0x0a4
#define DAYAN_MMU_REG_REDIR_DEFAULT_LO     0x0a8
#define DAYAN_MMU_REG_REDIR_DEFAULT_HI     0x0ac
#define DAYAN_MMU_REG_FIX_MAP_SIZE_LO      0x0b0
#define DAYAN_MMU_REG_FIX_MAP_SIZE_HI      0x0b4

/*
 * Classification used by the QEMU device to decide which accepted APB
 * writes are mirrored to the Dayan-emu side (see docs qemu-MMU guide
 * section 6.10.3): translation-semantic configuration goes out as a raw
 * register write, flush events are reported as their own message class
 * and pure status registers are never forwarded.
 */
typedef enum {
    DAYAN_MMU_FWD_NONE = 0,      /* status / undefined: do not forward  */
    DAYAN_MMU_FWD_REG_WRITE,     /* translation-semantic config write   */
    DAYAN_MMU_FWD_FLUSH,         /* TLB_FLUSH trigger                   */
} DayanMmuFwdClass;

typedef struct DayanMmuTlbEntry {
    bool valid;
    bool negative;
    uint8_t fault;                 /* DayanMmuFault                    */
    uint64_t vpn;                  /* 27-bit                           */
    uint64_t ppn;                  /* 44-bit                           */
    uint8_t asid;
    uint8_t page_size;             /* DayanMmuPageSize                 */
    bool global;
    bool r, w, x, u, a, d;
} DayanMmuTlbEntry;

typedef struct DayanMmuPdcEntry {
    bool valid;
    uint16_t vpn2, vpn1;
    uint8_t asid;
    bool global;
    uint64_t ppn;
} DayanMmuPdcEntry;

/*
 * Extended translate provenance: which PTE bus reads the page walk
 * issued, the leaf PTE observed, and whether the result came from the
 * fault redirect path.  Plain dayan_mmu_core_translate() discards
 * these; the Dayan-emu's translation result exposes them.
 */
typedef struct DayanMmuTranslateDetail {
    uint64_t walk_addresses[3];
    unsigned walk_depth;
    uint64_t pte_snapshot;
    bool global_mapping;
    bool redirected;
} DayanMmuTranslateDetail;

typedef struct DayanMmuCore {
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
    DayanMmuTlbEntry l1[DAYAN_MMU_NUM_PORTS][DAYAN_MMU_L1_ENTRIES_MAX];
    DayanMmuTlbEntry l2_4k[128];                 /* 128 sets x 1 way  */
    DayanMmuTlbEntry l2_2m[256 * 4];             /* 256 sets x 4 ways */
    DayanMmuTlbEntry l2_1g[128];                 /* 128 sets x 1 way  */
    DayanMmuPdcEntry pdc_l0[16];
    DayanMmuPdcEntry pdc_l1[16];
    uint16_t l1_lfsr;
    uint8_t repl_lfsr;

    /* Flush status bits (APB view). */
    bool flush_done;
    bool flush_busy;
    uint8_t flush_error_code;

    /* PTW transport: bound DRAM image, or an injected reader. */
    DayanMmuPteReaderFn reader;
    void *reader_opaque;
    const void *dram_host;
    uint64_t dram_base;
    uint64_t dram_size;

    /* Statistics (best effort, not architectural). */
    uint64_t stat_xlate;
    uint64_t stat_redirect;
    uint64_t stat_fault[16];
} DayanMmuCore;

#ifdef __cplusplus
extern "C" {
#endif

/* Reset all architectural and cache state to the power-on image
 * (FixMap full-range identity, empty caches). */
void dayan_mmu_core_init(DayanMmuCore *c);

/* Bind the shared DRAM image used by PTW reads.  PTW reads outside
 * [base, base + size - 8] return bus_error, matching the Dayan-emu
 * make_dram_reader(). */
void dayan_mmu_core_bind_dram(DayanMmuCore *c, const void *host,
                              uint64_t base, uint64_t size);
void dayan_mmu_core_set_reader(DayanMmuCore *c, DayanMmuPteReaderFn fn,
                               void *opaque);

/* One PTW bus read against the bound DRAM image.  Exposed so callers
 * that only need the DRAM path need not duplicate its bounds. */
DayanMmuPteRead dayan_mmu_core_dram_read(const DayanMmuCore *c, uint64_t pa);

/* APB4 register image.  write_reg returns false only for an unsupported
 * address; alignment-constrained writes are accepted but silently
 * ignored, matching the RTL. */
bool dayan_mmu_core_write_reg(DayanMmuCore *c, uint16_t off, uint32_t val);
uint32_t dayan_mmu_core_read_reg(const DayanMmuCore *c, uint16_t off);
DayanMmuFwdClass dayan_mmu_core_classify_write(uint16_t off);

/*
 * Translate one access.  The configuration image is snapshotted at call
 * entry (software equivalent of the slave AR/AW acceptance handshake).
 * On fault the returned physical address is the redirect PA, so callers
 * can keep treating the result as a plain address.  @fault and
 * @page_size may be NULL.
 */
uint64_t dayan_mmu_core_translate(DayanMmuCore *c, uint64_t va,
                                  DayanMmuAccess acc, unsigned port,
                                  DayanMmuFault *fault,
                                  DayanMmuPageSize *page_size);

/*
 * Translate with explicit provenance and an optional caller-pinned
 * config image.  @cfg == NULL snapshots @c at entry, like
 * dayan_mmu_core_translate().  A non-NULL @cfg is the two-phase
 * handshake form: config fields (ptw_base, FixMap, ASID, redirect
 * bases) are read from it after masking into a local copy, while TLB,
 * PDC and the PTW transport always come from @c.  @detail, if given,
 * receives the walk provenance of this access.
 */
uint64_t dayan_mmu_core_translate_full(DayanMmuCore *c,
                                       const DayanMmuCore *cfg,
                                       uint64_t va, DayanMmuAccess acc,
                                       unsigned port,
                                       DayanMmuFault *fault_out,
                                       DayanMmuPageSize *page_size_out,
                                       DayanMmuTranslateDetail *detail_out);

/* Flush semantics mirror the Dayan-emu: full or ASID-selective clear,
 * global entries survive ASID flushes, sticky DONE/ERR status. */
void dayan_mmu_core_flush(DayanMmuCore *c, bool asid_sel, uint8_t asid);
void dayan_mmu_core_ack_flush(DayanMmuCore *c);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* HW_RISCV_DAYAN_MMU_CORE_H */
