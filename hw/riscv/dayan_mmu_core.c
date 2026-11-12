/*
 * Dayan ClusterMMU functional model - pure C core
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Single behavioral implementation of the ClusterMMU, shared by the
 * QEMU dayan_mmu device, the Dayan-emu C++ shell (emu/mmu.cpp) and
 * the golden-vector suites on both sides.  The RTL boundary behaviors
 * (FixMap 39-bit arithmetic, PDC non-G-restoring hits, ERROR_SLV_MASK
 * stored but unconsumed, negative entries only for PTW faults) are
 * modeled here.  Only addresses at/above DAYAN_MMU_DRAM_BASE (2 GiB)
 * are translated; lower addresses pass through identity.  This file
 * must stay QEMU-header-free.
 */

#include "hw/riscv/dayan_mmu_core.h"

#include <stddef.h>
#include <string.h>

#define PPN_MASK   ((1ull << 44) - 1)
#define VPN_MASK   ((1ull << 27) - 1)
#define PAGE_BITS  12

#define L2_4K_SETS     128
#define L2_4K_WAYS     1
#define L2_2M_SETS     256
#define L2_2M_WAYS     4
#define L2_1G_SETS     128
#define L2_1G_WAYS     1
#define PDC_ENTRIES    16

static uint64_t mask_pa(uint64_t value)
{
    return value & DAYAN_MMU_PA_MASK;
}

static uint64_t vpn2_of(uint64_t vpn)
{
    return (vpn >> 18) & 0x1ff;
}

static uint64_t vpn1_of(uint64_t vpn)
{
    return (vpn >> 9) & 0x1ff;
}

static uint64_t vpn0_of(uint64_t vpn)
{
    return vpn & 0x1ff;
}

static uint64_t vpn_index(uint64_t vpn, unsigned level)
{
    switch (level) {
    case 0:
        return vpn2_of(vpn);
    case 1:
        return vpn1_of(vpn);
    default:
        return vpn0_of(vpn);
    }
}

static uint64_t page_size_bytes(uint8_t size)
{
    switch (size) {
    case DAYAN_MMU_PAGE_1G:
        return 1ull << 30;
    case DAYAN_MMU_PAGE_2M:
        return 1ull << 21;
    default:
        return 1ull << 12;
    }
}

static uint64_t page_low_mask(uint8_t size)
{
    return page_size_bytes(size) - 1;
}

static unsigned l1_entries(unsigned port)
{
    return port < DAYAN_MMU_SCALAR_PORTS ?
           DAYAN_MMU_L1_ENTRIES_SCALAR : DAYAN_MMU_L1_ENTRIES_TENSOR;
}

static bool page_match(const DayanMmuTlbEntry *entry, uint64_t vpn)
{
    switch (entry->page_size) {
    case DAYAN_MMU_PAGE_1G:
        return vpn2_of(entry->vpn) == vpn2_of(vpn);
    case DAYAN_MMU_PAGE_2M:
        return vpn2_of(entry->vpn) == vpn2_of(vpn) &&
               vpn1_of(entry->vpn) == vpn1_of(vpn);
    default:
        return (entry->vpn & VPN_MASK) == (vpn & VPN_MASK);
    }
}

static bool permission_ok(const DayanMmuTlbEntry *entry,
                          DayanMmuAccess access, DayanMmuFault *fault)
{
    if (!entry->valid) {
        if (fault) {
            *fault = DAYAN_MMU_FAULT_PTE_INVALID;
        }
        return false;
    }
    switch (access) {
    case DAYAN_MMU_READ:
        if (!entry->r) {
            if (fault) {
                *fault = DAYAN_MMU_FAULT_READ_PERM;
            }
            return false;
        }
        break;
    case DAYAN_MMU_WRITE:
        if (!entry->w) {
            if (fault) {
                *fault = DAYAN_MMU_FAULT_WRITE_PERM;
            }
            return false;
        }
        break;
    case DAYAN_MMU_EXECUTE:
        if (!entry->x) {
            if (fault) {
                *fault = DAYAN_MMU_FAULT_EXEC_PERM;
            }
            return false;
        }
        break;
    }
    if (fault) {
        *fault = DAYAN_MMU_FAULT_NONE;
    }
    return true;
}

/* ------------------------------------------------------------------ */
/* Construction / reset                                                */
/* ------------------------------------------------------------------ */

void dayan_mmu_core_init(DayanMmuCore *c)
{
    memset(c, 0, sizeof(*c));
    /* N_FIX_MAP_EN=0 with base 0 and size 0 gives the full-range
     * identity FixMap that keeps unconfigured software running. */
    c->l1_lfsr = 1;
    c->repl_lfsr = 1;
}

static void invalidate_all(DayanMmuCore *c);

void dayan_mmu_core_bind_dram(DayanMmuCore *c, const void *host,
                              uint64_t base, uint64_t size)
{
    c->dram_host = host;
    c->dram_base = base & DAYAN_MMU_PA_MASK;
    c->dram_size = size;
    /* The PTW transport changed; cached translations are stale. */
    invalidate_all(c);
}

void dayan_mmu_core_set_reader(DayanMmuCore *c, DayanMmuPteReaderFn fn,
                               void *opaque)
{
    c->reader = fn;
    c->reader_opaque = opaque;
    invalidate_all(c);
}

DayanMmuPteRead dayan_mmu_core_dram_read(const DayanMmuCore *c, uint64_t pa)
{
    DayanMmuPteRead result = { DAYAN_MMU_PTE_BUS_ERROR, 0 };
    uint64_t address = pa & DAYAN_MMU_PA_MASK;
    uint64_t offset;

    if (!c->dram_host || address < c->dram_base) {
        return result;
    }
    offset = address - c->dram_base;
    if (offset > c->dram_size || c->dram_size - offset < sizeof(uint64_t)) {
        return result;
    }
    memcpy(&result.data, (const uint8_t *)c->dram_host + offset,
           sizeof(result.data));
    result.response = DAYAN_MMU_PTE_OKAY;
    return result;
}

static DayanMmuPteRead pte_read(const DayanMmuCore *c, uint64_t pa)
{
    if (c->reader) {
        return c->reader(c->reader_opaque, pa);
    }
    return dayan_mmu_core_dram_read(c, pa);
}

/* ------------------------------------------------------------------ */
/* FixMap and redirect                                                 */
/* ------------------------------------------------------------------ */

static bool fixmap_hit(uint64_t va, const DayanMmuCore *c)
{
    uint64_t effective_va, page;

    if (c->n_fixmap_en) {
        return false;
    }
    /* Hardware computes FixMap on the low 39 VA bits regardless of the
     * upper canonical bits (spec 15.1). */
    effective_va = va & DAYAN_MMU_VA_MASK;
    page = effective_va >> PAGE_BITS;
    return c->fixmap_size_pages == 0 || page < c->fixmap_size_pages;
}

static uint64_t fixmap_pa(uint64_t va, const DayanMmuCore *c)
{
    return mask_pa(c->fixmap_base + (va & DAYAN_MMU_VA_MASK));
}

static uint64_t redirect_pa(uint64_t va, DayanMmuFault fault,
                            const DayanMmuCore *c)
{
    uint64_t base = c->redir[5];   /* default_base */

    switch (fault) {
    case DAYAN_MMU_FAULT_PTE_INVALID:
    case DAYAN_MMU_FAULT_ILLEGAL_PTE:
        base = c->redir[0];        /* page_fault */
        break;
    case DAYAN_MMU_FAULT_READ_PERM:
        base = c->redir[1];
        break;
    case DAYAN_MMU_FAULT_WRITE_PERM:
        base = c->redir[2];
        break;
    case DAYAN_MMU_FAULT_EXEC_PERM:
        base = c->redir[3];
        break;
    case DAYAN_MMU_FAULT_PTW_BUS_ERROR:
    case DAYAN_MMU_FAULT_PTW_TIMEOUT:
        base = c->redir[4];        /* ptw_error */
        break;
    default:
        break;
    }
    if (c->redir_offset_enable) {
        base += va & DAYAN_MMU_PAGE_MASK;
    }
    return mask_pa(base);
}

/* ------------------------------------------------------------------ */
/* L1 / L2 / PDC                                                       */
/* ------------------------------------------------------------------ */

static DayanMmuTlbEntry *l1_lookup(DayanMmuCore *c, unsigned port,
                                   uint64_t vpn, uint8_t asid)
{
    unsigned count = l1_entries(port);
    unsigned i;

    for (i = 0; i < count; i++) {
        DayanMmuTlbEntry *entry = &c->l1[port][i];

        if (entry->valid && page_match(entry, vpn) &&
            (entry->global || entry->asid == asid)) {
            return entry;
        }
    }
    return NULL;
}

static void l1_insert(DayanMmuCore *c, unsigned port,
                      const DayanMmuTlbEntry *entry)
{
    unsigned count = l1_entries(port);
    unsigned victim = count;
    unsigned i;

    if (!entry->valid) {
        return;
    }
    for (i = 0; i < count; i++) {
        DayanMmuTlbEntry *old = &c->l1[port][i];

        if (old->valid && old->vpn == entry->vpn &&
            (old->global || old->asid == entry->asid)) {
            old->valid = false;
        }
    }
    for (i = 0; i < count; i++) {
        if (!c->l1[port][i].valid) {
            victim = i;
            break;
        }
    }
    if (victim == count) {
        victim = c->l1_lfsr % count;
    }
    c->l1[port][victim] = *entry;
    c->l1_lfsr = (uint16_t)((c->l1_lfsr << 1) ^
        (((c->l1_lfsr >> 15) ^ (c->l1_lfsr >> 13) ^
          (c->l1_lfsr >> 12) ^ (c->l1_lfsr >> 10)) & 1));
    if (c->l1_lfsr == 0) {
        c->l1_lfsr = 1;
    }
}

static DayanMmuTlbEntry *l2_bank_lookup(DayanMmuTlbEntry *entries,
                                        unsigned sets, unsigned ways,
                                        unsigned set, uint64_t vpn,
                                        uint8_t asid)
{
    unsigned way;

    for (way = 0; way < ways; way++) {
        DayanMmuTlbEntry *entry = &entries[set * ways + way];

        if (entry->valid && page_match(entry, vpn) &&
            (entry->global || entry->asid == asid)) {
            return entry;
        }
    }
    return NULL;
}

/* Overlapping hits resolve 4K > 2M > 1G (bank order below). */
static DayanMmuTlbEntry *l2_lookup(DayanMmuCore *c, uint64_t vpn,
                                   uint8_t asid)
{
    DayanMmuTlbEntry *entry;

    entry = l2_bank_lookup(c->l2_4k, L2_4K_SETS, L2_4K_WAYS,
                           (unsigned)(vpn & 0x7f), vpn, asid);
    if (entry) {
        return entry;
    }
    entry = l2_bank_lookup(c->l2_2m, L2_2M_SETS, L2_2M_WAYS,
                           (unsigned)((vpn >> 9) & 0xff), vpn, asid);
    if (entry) {
        return entry;
    }
    return l2_bank_lookup(c->l2_1g, L2_1G_SETS, L2_1G_WAYS,
                          (unsigned)((vpn >> 18) & 0x7f), vpn, asid);
}

static void advance_repl_lfsr(DayanMmuCore *c)
{
    c->repl_lfsr = (uint8_t)((c->repl_lfsr << 1) ^
        (((c->repl_lfsr >> 7) ^ (c->repl_lfsr >> 5) ^
          (c->repl_lfsr >> 4) ^ (c->repl_lfsr >> 3)) & 1));
    if (c->repl_lfsr == 0) {
        c->repl_lfsr = 1;
    }
}

static void l2_insert(DayanMmuCore *c, const DayanMmuTlbEntry *entry)
{
    DayanMmuTlbEntry *entries;
    unsigned ways, set, victim, way;

    if (!entry->valid) {
        return;
    }
    switch (entry->page_size) {
    case DAYAN_MMU_PAGE_2M:
        entries = c->l2_2m;
        ways = L2_2M_WAYS;
        set = (unsigned)((entry->vpn >> 9) & 0xff);
        break;
    case DAYAN_MMU_PAGE_1G:
        entries = c->l2_1g;
        ways = L2_1G_WAYS;
        set = (unsigned)((entry->vpn >> 18) & 0x7f);
        break;
    default:
        entries = c->l2_4k;
        ways = L2_4K_WAYS;
        set = (unsigned)(entry->vpn & 0x7f);
        break;
    }
    victim = ways;
    for (way = 0; way < ways; way++) {
        if (!entries[set * ways + way].valid) {
            victim = way;
            break;
        }
    }
    if (victim == ways) {
        victim = c->repl_lfsr % ways;
    }
    entries[set * ways + victim] = *entry;
    advance_repl_lfsr(c);
}

static bool pdc_lookup(const DayanMmuPdcEntry *entries, unsigned count,
                       uint16_t v2, uint16_t v1, bool match_v1,
                       uint8_t asid, bool *global, uint64_t *ppn)
{
    unsigned i;

    for (i = 0; i < count; i++) {
        const DayanMmuPdcEntry *entry = &entries[i];

        if (entry->valid && entry->vpn2 == v2 &&
            (!match_v1 || entry->vpn1 == v1) &&
            (entry->global || entry->asid == asid)) {
            *global = entry->global;
            *ppn = entry->ppn;
            return true;
        }
    }
    return false;
}

/* RTL boundary (spec 15.2): a PDC hit restores only the skipped
 * non-leaf's PPN; its G bit is not re-derived from the entry, so the
 * cached_global used for insertion below is the node's own G bit. */
static void pdc_insert(DayanMmuPdcEntry *entries, unsigned count,
                       uint16_t v2, uint16_t v1,
                       uint8_t asid, bool global, uint64_t ppn,
                       DayanMmuCore *c)
{
    unsigned victim = count;
    unsigned i;

    for (i = 0; i < count; i++) {
        if (!entries[i].valid) {
            victim = i;
            break;
        }
    }
    if (victim == count) {
        victim = c->repl_lfsr % count;
    }
    entries[victim].valid = true;
    entries[victim].vpn2 = v2;
    entries[victim].vpn1 = v1;
    entries[victim].asid = asid;
    entries[victim].global = global;
    entries[victim].ppn = ppn & PPN_MASK;
    advance_repl_lfsr(c);
}

/* ------------------------------------------------------------------ */
/* Sv39 page table walk                                                */
/* ------------------------------------------------------------------ */

struct walk_result {
    DayanMmuFault fault;           /* NONE means leaf found            */
    uint8_t page_size;
    bool global_mapping;
    uint64_t pa;
    uint64_t pte_snapshot;
    uint64_t walk_addresses[3];    /* PTE bus reads, in issue order    */
    unsigned walk_depth;
};

/* @cfg supplies the config image for this access (ptw_base, ASID);
 * the PDC arrays and PTW transport are always the live @c state. */
static struct walk_result walk(DayanMmuCore *c, const DayanMmuCore *cfg,
                               uint64_t va, uint64_t vpn)
{
    struct walk_result result;
    const uint8_t asid = cfg->global_asid;
    uint64_t current_ppn = (cfg->ptw_base >> PAGE_BITS) & PPN_MASK;
    bool accumulated_global = false;
    const uint16_t v2 = (uint16_t)vpn2_of(vpn);
    const uint16_t v1 = (uint16_t)vpn1_of(vpn);
    unsigned level = 0;

    memset(&result, 0, sizeof(result));
    result.fault = DAYAN_MMU_FAULT_ILLEGAL_PTE;

    while (level < 3) {
        bool pdc_global = false;
        uint64_t pdc_ppn = 0;

        if (level == 0 &&
            pdc_lookup(c->pdc_l0, PDC_ENTRIES, v2, 0, false, asid,
                       &pdc_global, &pdc_ppn)) {
            current_ppn = pdc_ppn;
            level = 1;
            continue;
        }
        if (level == 1 &&
            pdc_lookup(c->pdc_l1, PDC_ENTRIES, v2, v1, true, asid,
                       &pdc_global, &pdc_ppn)) {
            current_ppn = pdc_ppn;
            level = 2;
            continue;
        }

        uint64_t pte_pa = mask_pa((current_ppn << PAGE_BITS) +
                                  (vpn_index(vpn, level) << 3));
        DayanMmuPteRead read = pte_read(c, pte_pa);
        uint64_t pte = read.data;
        bool valid, readable, writable, executable, global;
        uint64_t ppn;

        /* At most one read per level, so the fixed array cannot
         * overflow. */
        result.walk_addresses[result.walk_depth++] = pte_pa;

        if (read.response == DAYAN_MMU_PTE_TIMEOUT) {
            result.fault = DAYAN_MMU_FAULT_PTW_TIMEOUT;
            return result;
        }
        if (read.response == DAYAN_MMU_PTE_BUS_ERROR) {
            result.fault = DAYAN_MMU_FAULT_PTW_BUS_ERROR;
            result.pte_snapshot = read.data;
            return result;
        }

        valid = (pte >> 0) & 1;
        readable = (pte >> 1) & 1;
        writable = (pte >> 2) & 1;
        executable = (pte >> 3) & 1;
        global = (pte >> 5) & 1;
        ppn = (pte >> 10) & PPN_MASK;

        if (!valid) {
            result.fault = DAYAN_MMU_FAULT_PTE_INVALID;
            result.pte_snapshot = pte;
            return result;
        }
        if (writable && !readable) {
            result.fault = DAYAN_MMU_FAULT_ILLEGAL_PTE;
            result.pte_snapshot = pte;
            return result;
        }
        if (readable || executable) {
            if ((level == 0 && (ppn & ((1ull << 18) - 1))) ||
                (level == 1 && (ppn & ((1ull << 9) - 1)))) {
                result.fault = DAYAN_MMU_FAULT_ILLEGAL_PTE;
                result.pte_snapshot = pte;
                return result;
            }
            result.fault = DAYAN_MMU_FAULT_NONE;
            result.page_size = level == 0 ? DAYAN_MMU_PAGE_1G :
                               level == 1 ? DAYAN_MMU_PAGE_2M :
                                            DAYAN_MMU_PAGE_4K;
            result.global_mapping = accumulated_global || global;
            result.pte_snapshot = pte;
            {
                uint64_t low_bits = page_size_bytes(result.page_size) - 1;

                result.pa = mask_pa(((ppn << PAGE_BITS) & ~low_bits) |
                                    (va & low_bits));
            }
            return result;
        }
        if (level == 2) {
            result.fault = DAYAN_MMU_FAULT_ILLEGAL_PTE;
            result.pte_snapshot = pte;
            return result;
        }

        accumulated_global = accumulated_global || global;
        if (level == 0) {
            pdc_insert(c->pdc_l0, PDC_ENTRIES, v2, 0, asid,
                       global, ppn, c);
        } else {
            pdc_insert(c->pdc_l1, PDC_ENTRIES, v2, v1, asid,
                       global, ppn, c);
        }
        current_ppn = ppn;
        level++;
    }

    result.fault = DAYAN_MMU_FAULT_ILLEGAL_PTE;
    return result;
}

/* ------------------------------------------------------------------ */
/* Translate                                                           */
/* ------------------------------------------------------------------ */

uint64_t dayan_mmu_core_translate_full(DayanMmuCore *c,
                                       const DayanMmuCore *cfg_in,
                                       uint64_t va, DayanMmuAccess acc,
                                       unsigned port,
                                       DayanMmuFault *fault_out,
                                       DayanMmuPageSize *page_size_out,
                                       DayanMmuTranslateDetail *detail_out)
{
    DayanMmuCore snapshot;
    DayanMmuCore *cfg = &snapshot;
    DayanMmuFault fault = DAYAN_MMU_FAULT_NONE;
    DayanMmuPageSize size = DAYAN_MMU_PAGE_4K;
    DayanMmuTranslateDetail detail;
    uint64_t pa = 0;
    uint64_t vpn;
    DayanMmuTlbEntry *entry;
    unsigned i;

    memset(&detail, 0, sizeof(detail));

    /* Shared exit so every path reports the fault class, the page size,
     * the walk provenance and the (possibly redirect) PA exactly once.
     * size stays PAGE_4K on fault paths, matching the fault
     * substitution semantics. */
    #define MMU_RETURN()                              \
        do {                                          \
            if (fault != DAYAN_MMU_FAULT_NONE) {      \
                c->stat_fault[fault & 15]++;          \
                c->stat_redirect++;                   \
            }                                         \
            if (fault_out) {                          \
                *fault_out = fault;                   \
            }                                         \
            if (page_size_out) {                      \
                *page_size_out = size;                \
            }                                         \
            if (detail_out) {                         \
                *detail_out = detail;                 \
            }                                         \
            if (fault != DAYAN_MMU_FAULT_NONE) {      \
                return redirect_pa(va, fault, cfg);   \
            }                                         \
            return pa;                                \
        } while (0)

    /* Hardware topology: the ClusterMMU slave ports only observe bus
     * addresses at/above the DRAM base (2 GiB).  Everything below --
     * the URAM aperture, all control space and the MMU's own APB
     * page -- is routed around the translation engine and passes
     * through identity regardless of FixMap or page-table state. */
    if (va < DAYAN_MMU_DRAM_BASE) {
        pa = va;
        MMU_RETURN();
    }

    /* NULL cfg: snapshot the config image at entry (slave AR/AW
     * acceptance).  Non-NULL: the caller pinned the image at capture
     * time; mask into the local copy, never the caller's struct. */
    if (cfg_in) {
        snapshot = *cfg_in;
    } else {
        memcpy(&snapshot, c, sizeof(snapshot));
    }
    if (port >= DAYAN_MMU_NUM_PORTS) {
        port = 0;
    }
    snapshot.ptw_base = mask_pa(snapshot.ptw_base);
    snapshot.fixmap_base = mask_pa(snapshot.fixmap_base);
    snapshot.fixmap_size_pages = mask_pa(snapshot.fixmap_size_pages);
    for (i = 0; i < 6; i++) {
        snapshot.redir[i] = mask_pa(snapshot.redir[i]);
    }

    c->stat_xlate++;
    vpn = (va >> PAGE_BITS) & VPN_MASK;

    if (fixmap_hit(va, cfg)) {
        pa = fixmap_pa(va, cfg);
        size = DAYAN_MMU_PAGE_4K;
        MMU_RETURN();
    }

    entry = l1_lookup(c, port, vpn, cfg->global_asid);
    if (entry) {
        if (entry->negative) {
            fault = entry->fault;
        } else {
            permission_ok(entry, acc, &fault);
        }
        if (fault != DAYAN_MMU_FAULT_NONE) {
            detail.redirected = true;
            MMU_RETURN();
        }
        size = entry->page_size;
        pa = mask_pa((((entry->ppn & PPN_MASK) << PAGE_BITS) &
                      ~page_low_mask(size)) |
                     (va & page_low_mask(size)));
        detail.global_mapping = entry->global;
        MMU_RETURN();
    }

    entry = l2_lookup(c, vpn, cfg->global_asid);
    if (entry) {
        DayanMmuTlbEntry cached = *entry;

        if (cached.negative) {
            /* Negative hits report the cached fault directly; they are
             * not promoted into the port L1. */
            fault = cached.fault;
            detail.redirected = true;
            MMU_RETURN();
        }
        l1_insert(c, port, &cached);
        permission_ok(&cached, acc, &fault);
        if (fault != DAYAN_MMU_FAULT_NONE) {
            detail.redirected = true;
            MMU_RETURN();
        }
        size = cached.page_size;
        pa = mask_pa((((cached.ppn & PPN_MASK) << PAGE_BITS) &
                      ~page_low_mask(size)) |
                     (va & page_low_mask(size)));
        detail.global_mapping = cached.global;
        MMU_RETURN();
    }

    {
        struct walk_result walked = walk(c, cfg, va, vpn);

        for (i = 0; i < walked.walk_depth; i++) {
            detail.walk_addresses[i] = walked.walk_addresses[i];
        }
        detail.walk_depth = walked.walk_depth;
        detail.pte_snapshot = walked.pte_snapshot;

        if (walked.fault != DAYAN_MMU_FAULT_NONE) {
            /* Walk faults (and only they) enter the L2 negative cache;
             * permission checks happen after a successful fill. */
            DayanMmuTlbEntry negative;

            memset(&negative, 0, sizeof(negative));
            negative.valid = true;
            negative.negative = true;
            negative.fault = walked.fault;
            negative.vpn = vpn;
            negative.asid = cfg->global_asid;
            negative.page_size = DAYAN_MMU_PAGE_4K;
            l2_insert(c, &negative);

            fault = walked.fault;
            detail.redirected = true;
            MMU_RETURN();
        }

        {
            DayanMmuTlbEntry filled;

            memset(&filled, 0, sizeof(filled));
            filled.valid = true;
            filled.vpn = vpn;
            filled.asid = cfg->global_asid;
            filled.page_size = walked.page_size;
            filled.global = walked.global_mapping;
            filled.r = (walked.pte_snapshot >> 1) & 1;
            filled.w = (walked.pte_snapshot >> 2) & 1;
            filled.x = (walked.pte_snapshot >> 3) & 1;
            filled.u = (walked.pte_snapshot >> 4) & 1;
            filled.a = (walked.pte_snapshot >> 6) & 1;
            filled.d = (walked.pte_snapshot >> 7) & 1;
            filled.ppn = (walked.pte_snapshot >> 10) & PPN_MASK;

            l2_insert(c, &filled);
            l1_insert(c, port, &filled);

            permission_ok(&filled, acc, &fault);
            if (fault != DAYAN_MMU_FAULT_NONE) {
                detail.redirected = true;
                MMU_RETURN();
            }
            size = filled.page_size;
            pa = mask_pa((((filled.ppn & PPN_MASK) << PAGE_BITS) &
                          ~page_low_mask(size)) |
                         (va & page_low_mask(size)));
            detail.global_mapping = filled.global;
            MMU_RETURN();
        }
    }

    #undef MMU_RETURN
}

uint64_t dayan_mmu_core_translate(DayanMmuCore *c, uint64_t va,
                                  DayanMmuAccess acc, unsigned port,
                                  DayanMmuFault *fault_out,
                                  DayanMmuPageSize *page_size_out)
{
    return dayan_mmu_core_translate_full(c, NULL, va, acc, port,
                                         fault_out, page_size_out, NULL);
}

/* ------------------------------------------------------------------ */
/* Flush                                                               */
/* ------------------------------------------------------------------ */

static void invalidate_all(DayanMmuCore *c)
{
    memset(c->l1, 0, sizeof(c->l1));
    memset(c->l2_4k, 0, sizeof(c->l2_4k));
    memset(c->l2_2m, 0, sizeof(c->l2_2m));
    memset(c->l2_1g, 0, sizeof(c->l2_1g));
    memset(c->pdc_l0, 0, sizeof(c->pdc_l0));
    memset(c->pdc_l1, 0, sizeof(c->pdc_l1));
}

static void invalidate_asid(DayanMmuCore *c, uint8_t asid)
{
    unsigned port, i;

    for (port = 0; port < DAYAN_MMU_NUM_PORTS; port++) {
        for (i = 0; i < l1_entries(port); i++) {
            DayanMmuTlbEntry *entry = &c->l1[port][i];

            if (entry->valid && !entry->global && entry->asid == asid) {
                entry->valid = false;
            }
        }
    }
    for (i = 0; i < L2_4K_SETS * L2_4K_WAYS; i++) {
        DayanMmuTlbEntry *entry = &c->l2_4k[i];

        if (entry->valid && !entry->global && entry->asid == asid) {
            entry->valid = false;
        }
    }
    for (i = 0; i < L2_2M_SETS * L2_2M_WAYS; i++) {
        DayanMmuTlbEntry *entry = &c->l2_2m[i];

        if (entry->valid && !entry->global && entry->asid == asid) {
            entry->valid = false;
        }
    }
    for (i = 0; i < L2_1G_SETS * L2_1G_WAYS; i++) {
        DayanMmuTlbEntry *entry = &c->l2_1g[i];

        if (entry->valid && !entry->global && entry->asid == asid) {
            entry->valid = false;
        }
    }
    for (i = 0; i < PDC_ENTRIES; i++) {
        if (c->pdc_l0[i].valid && !c->pdc_l0[i].global &&
            c->pdc_l0[i].asid == asid) {
            c->pdc_l0[i].valid = false;
        }
        if (c->pdc_l1[i].valid && !c->pdc_l1[i].global &&
            c->pdc_l1[i].asid == asid) {
            c->pdc_l1[i].valid = false;
        }
    }
}

void dayan_mmu_core_flush(DayanMmuCore *c, bool asid_sel, uint8_t asid)
{
    /* The synchronous model never reports busy; keep the reject path
     * for parity with the Dayan-emu. */
    if (c->flush_busy) {
        c->flush_error_code = 1;
        return;
    }
    c->flush_busy = true;
    if (asid_sel) {
        invalidate_asid(c, asid);
    } else {
        invalidate_all(c);
    }
    c->flush_busy = false;
    c->flush_done = true;
    c->flush_error_code = 0;
}

void dayan_mmu_core_ack_flush(DayanMmuCore *c)
{
    c->flush_done = false;
}

/* ------------------------------------------------------------------ */
/* APB4 register image                                                 */
/* ------------------------------------------------------------------ */

static uint64_t low_word(uint64_t value)
{
    return value & 0xffffffffull;
}

static void set_low(uint64_t *field, uint32_t data)
{
    *field = (*field & ~(uint64_t)0xffffffff) | data;
}

static void set_high(uint64_t *field, uint32_t data)
{
    *field = (*field & 0xffffffffull) | ((uint64_t)(data & 0x00ffffff) << 32);
}

static void reapply_masks(DayanMmuCore *c)
{
    c->ptw_base = mask_pa(c->ptw_base);
    c->fixmap_base = mask_pa(c->fixmap_base);
    c->fixmap_size_pages = mask_pa(c->fixmap_size_pages);
    c->redir[0] = mask_pa(c->redir[0]);
    c->redir[1] = mask_pa(c->redir[1]);
    c->redir[2] = mask_pa(c->redir[2]);
    c->redir[3] = mask_pa(c->redir[3]);
    c->redir[4] = mask_pa(c->redir[4]);
    c->redir[5] = mask_pa(c->redir[5]);
    c->error_slv_mask = mask_pa(c->error_slv_mask);
}

DayanMmuFwdClass dayan_mmu_core_classify_write(uint16_t off)
{
    switch (off) {
    case DAYAN_MMU_REG_TLB_FLUSH:
        return DAYAN_MMU_FWD_FLUSH;
    case DAYAN_MMU_REG_PTW_BASE_LO:
    case DAYAN_MMU_REG_PTW_BASE_HI:
    case DAYAN_MMU_REG_FIX_MAP_BASE_LO:
    case DAYAN_MMU_REG_FIX_MAP_BASE_HI:
    case DAYAN_MMU_REG_N_FIX_MAP_EN:
    case DAYAN_MMU_REG_GLOBAL_ASID:
    case DAYAN_MMU_REG_ERROR_SLV_MASK_LO:
    case DAYAN_MMU_REG_ERROR_SLV_MASK_HI:
    case DAYAN_MMU_REG_ERROR_SLV_OFF_MODE:
    case DAYAN_MMU_REG_REDIR_PAGE_FAULT_LO:
    case DAYAN_MMU_REG_REDIR_PAGE_FAULT_HI:
    case DAYAN_MMU_REG_REDIR_READ_PERM_LO:
    case DAYAN_MMU_REG_REDIR_READ_PERM_HI:
    case DAYAN_MMU_REG_REDIR_WRITE_PERM_LO:
    case DAYAN_MMU_REG_REDIR_WRITE_PERM_HI:
    case DAYAN_MMU_REG_REDIR_EXEC_PERM_LO:
    case DAYAN_MMU_REG_REDIR_EXEC_PERM_HI:
    case DAYAN_MMU_REG_REDIR_PTW_ERR_LO:
    case DAYAN_MMU_REG_REDIR_PTW_ERR_HI:
    case DAYAN_MMU_REG_REDIR_DEFAULT_LO:
    case DAYAN_MMU_REG_REDIR_DEFAULT_HI:
    case DAYAN_MMU_REG_FIX_MAP_SIZE_LO:
    case DAYAN_MMU_REG_FIX_MAP_SIZE_HI:
        return DAYAN_MMU_FWD_REG_WRITE;
    default:
        /* FLUSH_ACK / ERROR_SLV_CFG_CLR / read-only / undefined. */
        return DAYAN_MMU_FWD_NONE;
    }
}

bool dayan_mmu_core_write_reg(DayanMmuCore *c, uint16_t off, uint32_t val)
{
    switch (off) {
    case DAYAN_MMU_REG_PTW_BASE_LO:
        set_low(&c->ptw_base, val);
        break;
    case DAYAN_MMU_REG_PTW_BASE_HI:
        set_high(&c->ptw_base, val);
        break;
    case DAYAN_MMU_REG_FIX_MAP_BASE_LO:
        /* Low halves bound to FixMap/redirect windows must be 4K
         * aligned; violations are silently ignored (RTL parity). */
        if ((val & 0xfff) == 0) {
            set_low(&c->fixmap_base, val);
        }
        break;
    case DAYAN_MMU_REG_FIX_MAP_BASE_HI:
        set_high(&c->fixmap_base, val);
        break;
    case DAYAN_MMU_REG_N_FIX_MAP_EN:
        c->n_fixmap_en = (val & 1) != 0;
        break;
    case DAYAN_MMU_REG_TLB_FLUSH:
        if (val & 1) {
            if (c->flush_busy) {
                c->flush_error_code = 1;
            } else {
                c->flush_busy = true;
                if (val & 2) {
                    invalidate_asid(c, (uint8_t)((val >> 2) & 0xff));
                } else {
                    invalidate_all(c);
                }
                c->flush_busy = false;
                c->flush_done = true;
                c->flush_error_code = 0;
            }
        }
        break;
    case DAYAN_MMU_REG_FLUSH_ACK:
        if (val & 1) {
            c->flush_done = false;
        }
        break;
    case DAYAN_MMU_REG_GLOBAL_ASID:
        c->global_asid = (uint8_t)(val & 0xff);
        break;
    case DAYAN_MMU_REG_ERROR_SLV_MASK_LO:
        /* Stored for architectural completeness; not consumed by the
         * data path (spec 15.3). */
        if ((val & 0xfff) == 0) {
            set_low(&c->error_slv_mask, val);
        }
        break;
    case DAYAN_MMU_REG_ERROR_SLV_MASK_HI:
        set_high(&c->error_slv_mask, val);
        break;
    case DAYAN_MMU_REG_ERROR_SLV_OFF_MODE:
        c->redir_offset_enable = (val & 1) != 0;
        break;
    case DAYAN_MMU_REG_ERROR_SLV_CFG_CLR:
        /* Standalone ERROR_SLV_CFG_ERR input is tied low. */
        break;
    case DAYAN_MMU_REG_REDIR_PAGE_FAULT_LO:
        if ((val & 0xfff) == 0) {
            set_low(&c->redir[0], val);
        }
        break;
    case DAYAN_MMU_REG_REDIR_PAGE_FAULT_HI:
        set_high(&c->redir[0], val);
        break;
    case DAYAN_MMU_REG_REDIR_READ_PERM_LO:
        if ((val & 0xfff) == 0) {
            set_low(&c->redir[1], val);
        }
        break;
    case DAYAN_MMU_REG_REDIR_READ_PERM_HI:
        set_high(&c->redir[1], val);
        break;
    case DAYAN_MMU_REG_REDIR_WRITE_PERM_LO:
        if ((val & 0xfff) == 0) {
            set_low(&c->redir[2], val);
        }
        break;
    case DAYAN_MMU_REG_REDIR_WRITE_PERM_HI:
        set_high(&c->redir[2], val);
        break;
    case DAYAN_MMU_REG_REDIR_EXEC_PERM_LO:
        if ((val & 0xfff) == 0) {
            set_low(&c->redir[3], val);
        }
        break;
    case DAYAN_MMU_REG_REDIR_EXEC_PERM_HI:
        set_high(&c->redir[3], val);
        break;
    case DAYAN_MMU_REG_REDIR_PTW_ERR_LO:
        if ((val & 0xfff) == 0) {
            set_low(&c->redir[4], val);
        }
        break;
    case DAYAN_MMU_REG_REDIR_PTW_ERR_HI:
        set_high(&c->redir[4], val);
        break;
    case DAYAN_MMU_REG_REDIR_DEFAULT_LO:
        if ((val & 0xfff) == 0) {
            set_low(&c->redir[5], val);
        }
        break;
    case DAYAN_MMU_REG_REDIR_DEFAULT_HI:
        set_high(&c->redir[5], val);
        break;
    case DAYAN_MMU_REG_FIX_MAP_SIZE_LO:
        set_low(&c->fixmap_size_pages, val);
        break;
    case DAYAN_MMU_REG_FIX_MAP_SIZE_HI:
        set_high(&c->fixmap_size_pages, val);
        break;
    default:
        /* Read-only and undefined addresses ignore writes. */
        return false;
    }
    reapply_masks(c);
    return true;
}

uint32_t dayan_mmu_core_read_reg(const DayanMmuCore *c, uint16_t off)
{
    switch (off) {
    case DAYAN_MMU_REG_PTW_BASE_LO:
        return (uint32_t)low_word(c->ptw_base);
    case DAYAN_MMU_REG_PTW_BASE_HI:
        return (uint32_t)(c->ptw_base >> 32) & 0x00ffffff;
    case DAYAN_MMU_REG_FIX_MAP_BASE_LO:
        return (uint32_t)low_word(c->fixmap_base);
    case DAYAN_MMU_REG_FIX_MAP_BASE_HI:
        return (uint32_t)(c->fixmap_base >> 32) & 0x00ffffff;
    case DAYAN_MMU_REG_N_FIX_MAP_EN:
        return c->n_fixmap_en ? 1 : 0;
    case DAYAN_MMU_REG_FLUSH_DONE:
        return c->flush_done ? 1 : 0;
    case DAYAN_MMU_REG_FLUSH_BUSY:
        return c->flush_busy ? 1 : 0;
    case DAYAN_MMU_REG_FLUSH_ERR_CODE:
        return c->flush_error_code;
    case DAYAN_MMU_REG_GLOBAL_ASID:
        return c->global_asid;
    case DAYAN_MMU_REG_ERROR_SLV_MASK_LO:
        return (uint32_t)low_word(c->error_slv_mask);
    case DAYAN_MMU_REG_ERROR_SLV_MASK_HI:
        return (uint32_t)(c->error_slv_mask >> 32) & 0x00ffffff;
    case DAYAN_MMU_REG_ERROR_SLV_OFF_MODE:
        return c->redir_offset_enable ? 1 : 0;
    case DAYAN_MMU_REG_ERROR_SLV_CFG_ERR:
        return 0;
    case DAYAN_MMU_REG_REDIR_PAGE_FAULT_LO:
        return (uint32_t)low_word(c->redir[0]);
    case DAYAN_MMU_REG_REDIR_PAGE_FAULT_HI:
        return (uint32_t)(c->redir[0] >> 32) & 0x00ffffff;
    case DAYAN_MMU_REG_REDIR_READ_PERM_LO:
        return (uint32_t)low_word(c->redir[1]);
    case DAYAN_MMU_REG_REDIR_READ_PERM_HI:
        return (uint32_t)(c->redir[1] >> 32) & 0x00ffffff;
    case DAYAN_MMU_REG_REDIR_WRITE_PERM_LO:
        return (uint32_t)low_word(c->redir[2]);
    case DAYAN_MMU_REG_REDIR_WRITE_PERM_HI:
        return (uint32_t)(c->redir[2] >> 32) & 0x00ffffff;
    case DAYAN_MMU_REG_REDIR_EXEC_PERM_LO:
        return (uint32_t)low_word(c->redir[3]);
    case DAYAN_MMU_REG_REDIR_EXEC_PERM_HI:
        return (uint32_t)(c->redir[3] >> 32) & 0x00ffffff;
    case DAYAN_MMU_REG_REDIR_PTW_ERR_LO:
        return (uint32_t)low_word(c->redir[4]);
    case DAYAN_MMU_REG_REDIR_PTW_ERR_HI:
        return (uint32_t)(c->redir[4] >> 32) & 0x00ffffff;
    case DAYAN_MMU_REG_REDIR_DEFAULT_LO:
        return (uint32_t)low_word(c->redir[5]);
    case DAYAN_MMU_REG_REDIR_DEFAULT_HI:
        return (uint32_t)(c->redir[5] >> 32) & 0x00ffffff;
    case DAYAN_MMU_REG_FIX_MAP_SIZE_LO:
        return (uint32_t)low_word(c->fixmap_size_pages);
    case DAYAN_MMU_REG_FIX_MAP_SIZE_HI:
        return (uint32_t)(c->fixmap_size_pages >> 32) & 0x00ffffff;
    default:
        return 0;
    }
}
