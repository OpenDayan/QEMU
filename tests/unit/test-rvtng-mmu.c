/*
 * ClusterMMU functional model unit tests
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Golden scenario suite for hw/riscv/rvtng_mmu_core.c.  The scenarios
 * and expected values mirror rvtng-cmodel/tests/mmu_test.cpp, whose
 * assertions are aligned against the ClusterMMU RTL; keeping both
 * suites scenario-identical is the drift guard for the C/C++ dual
 * implementation (guide decision D7).
 *
 * All configuration goes through the APB register image (the same
 * production entry point the guest uses); there is no direct-set API
 * in the C core by design.
 */

#include "qemu/osdep.h"

#include "hw/riscv/rvtng_mmu_core.h"

/* ------------------------------------------------------------------ */
/* Fake PTW transport                                                  */
/* ------------------------------------------------------------------ */

#define FAKE_MAX_PTES 64

typedef struct FakePageTable {
    struct {
        uint64_t addr;
        uint64_t pte;
    } entries[FAKE_MAX_PTES];
    unsigned count;
    RvtngMmuPteResponse response;
    size_t reads;
} FakePageTable;

static RvtngMmuPteRead fake_read(void *opaque, uint64_t addr)
{
    FakePageTable *t = opaque;
    RvtngMmuPteRead read = { t->response, 0 };
    unsigned i;

    t->reads++;
    if (t->response != RVTNG_MMU_PTE_OKAY) {
        return read;
    }
    for (i = 0; i < t->count; i++) {
        if (t->entries[i].addr == addr) {
            read.data = t->entries[i].pte;
            break;
        }
    }
    return read;
}

static void fake_put(FakePageTable *t, uint64_t addr, uint64_t pte)
{
    unsigned i;

    for (i = 0; i < t->count; i++) {
        if (t->entries[i].addr == addr) {
            t->entries[i].pte = pte;
            return;
        }
    }
    g_assert_cmpuint(t->count, <, FAKE_MAX_PTES);
    t->entries[t->count].addr = addr;
    t->entries[t->count].pte = pte;
    t->count++;
}

#define PTE_V (1ull << 0)
#define PTE_R (1ull << 1)
#define PTE_W (1ull << 2)
#define PTE_X (1ull << 3)
#define PTE_G (1ull << 5)

static const uint64_t kRoot = 0x80000000ull;
static const uint64_t kL1Table = 0x80001000ull;
static const uint64_t kL2Table = 0x80002000ull;
static const uint64_t kLeaf = 0x0000000040003000ull;

static uint64_t pte_for(uint64_t pa, uint64_t flags)
{
    return ((pa >> 12) << 10) | flags;
}

static uint64_t vpn_of(uint64_t va)
{
    return (va >> 12) & ((1ull << 27) - 1);
}

static uint64_t pte_addr(uint64_t table, uint64_t index)
{
    return table + index * sizeof(uint64_t);
}

/* Standard three-level Sv39 mapping of @va -> @pa with a 4 KiB leaf. */
static void install_4k(FakePageTable *t, uint64_t va, uint64_t pa,
                       uint64_t leaf_flags, uint64_t root_flags,
                       uint64_t l1_flags)
{
    uint64_t vpn = vpn_of(va);

    fake_put(t, pte_addr(kRoot, (vpn >> 18) & 0x1ff),
             pte_for(kL1Table, root_flags));
    fake_put(t, pte_addr(kL1Table, (vpn >> 9) & 0x1ff),
             pte_for(kL2Table, l1_flags));
    fake_put(t, pte_addr(kL2Table, vpn & 0x1ff), pte_for(pa, leaf_flags));
}

static void setup_svm(RvtngMmuCore *c, FakePageTable *t)
{
    rvtng_mmu_core_init(c);
    rvtng_mmu_core_set_reader(c, fake_read, t);
    /* Disable FixMap (active-low register) and point the PTW at the
     * fake root, mirroring make_mmu() in the CModel suite. */
    g_assert_true(rvtng_mmu_core_write_reg(c, RVTNG_MMU_REG_N_FIX_MAP_EN, 1));
    g_assert_true(rvtng_mmu_core_write_reg(
        c, RVTNG_MMU_REG_PTW_BASE_LO, (uint32_t)kRoot));
    g_assert_true(rvtng_mmu_core_write_reg(
        c, RVTNG_MMU_REG_PTW_BASE_HI, (uint32_t)(kRoot >> 32)));
}

/* Fault redirect table + offset mode; call after setup_svm() because
 * core_init wipes the register image. */
static void setup_redirects(RvtngMmuCore *c)
{
    g_assert_true(rvtng_mmu_core_write_reg(
        c, RVTNG_MMU_REG_REDIR_PAGE_FAULT_LO, 0x10000000));
    g_assert_true(rvtng_mmu_core_write_reg(
        c, RVTNG_MMU_REG_REDIR_READ_PERM_LO, 0x20000000));
    g_assert_true(rvtng_mmu_core_write_reg(
        c, RVTNG_MMU_REG_REDIR_WRITE_PERM_LO, 0x30000000));
    g_assert_true(rvtng_mmu_core_write_reg(
        c, RVTNG_MMU_REG_REDIR_EXEC_PERM_LO, 0x40000000));
    g_assert_true(rvtng_mmu_core_write_reg(
        c, RVTNG_MMU_REG_REDIR_PTW_ERR_LO, 0x50000000));
    g_assert_true(rvtng_mmu_core_write_reg(
        c, RVTNG_MMU_REG_ERROR_SLV_OFF_MODE, 1));
}

/* ------------------------------------------------------------------ */
/* FixMap                                                              */
/* ------------------------------------------------------------------ */

static void test_fixmap(void)
{
    RvtngMmuCore c;
    FakePageTable t;
    RvtngMmuFault fault;
    uint64_t high_va, pa;

    memset(&t, 0, sizeof(t));
    rvtng_mmu_core_init(&c);
    rvtng_mmu_core_set_reader(&c, fake_read, &t);

    /* Reset image: full-range identity FixMap. */
    pa = rvtng_mmu_core_translate(&c, 0x80001234, RVTNG_MMU_READ, 0,
                                  &fault, NULL);
    g_assert_cmpuint(fault, ==, RVTNG_MMU_FAULT_NONE);
    g_assert_cmpuint(pa, ==, 0x80001234);

    /* Below the 2 GiB DRAM base the engine is bypassed outright, so the
     * same offset passes through even though the reset FixMap would
     * identity-map it too. */
    pa = rvtng_mmu_core_translate(&c, 0x1234, RVTNG_MMU_READ, 0,
                                  &fault, NULL);
    g_assert_cmpuint(fault, ==, RVTNG_MMU_FAULT_NONE);
    g_assert_cmpuint(pa, ==, 0x1234);

    /* Windowed FixMap: base 0x100000, covering VA pages 0x80000..0x80001
     * (2 pages). */
    g_assert_true(rvtng_mmu_core_write_reg(&c, RVTNG_MMU_REG_FIX_MAP_BASE_LO,
                                           0x00100000));
    g_assert_true(rvtng_mmu_core_write_reg(&c, RVTNG_MMU_REG_FIX_MAP_SIZE_LO,
                                           0x80002));
    pa = rvtng_mmu_core_translate(&c, 0x80001000, RVTNG_MMU_WRITE, 0,
                                  &fault, NULL);
    g_assert_cmpuint(pa, ==, 0x80101000);
    /* First page outside the window: FixMap misses and the lookup falls
     * through to the (empty) page table -> pte_invalid. */
    pa = rvtng_mmu_core_translate(&c, 0x80002000, RVTNG_MMU_READ, 0,
                                  &fault, NULL);
    g_assert_cmpuint(fault, ==, RVTNG_MMU_FAULT_PTE_INVALID);

    /* Spec 15.1: FixMap arithmetic uses the low 39 VA bits, so a
     * non-canonical address still maps by its truncated form. */
    g_assert_true(rvtng_mmu_core_write_reg(&c, RVTNG_MMU_REG_FIX_MAP_BASE_LO,
                                           0x00002000));
    g_assert_true(rvtng_mmu_core_write_reg(&c, RVTNG_MMU_REG_FIX_MAP_SIZE_LO,
                                           0));
    high_va = (~RVTNG_MMU_VA_MASK & RVTNG_MMU_PA_MASK) | (1ull << 38);
    pa = rvtng_mmu_core_translate(&c, high_va, RVTNG_MMU_READ, 0,
                                  &fault, NULL);
    g_assert_cmpuint(pa, ==, 0x2000 + (1ull << 38));

    /* Misaligned FIX_MAP_BASE low half is silently ignored. */
    g_assert_true(rvtng_mmu_core_write_reg(&c, RVTNG_MMU_REG_FIX_MAP_BASE_LO,
                                           0x00000123));
    g_assert_cmpuint(rvtng_mmu_core_read_reg(&c, RVTNG_MMU_REG_FIX_MAP_BASE_LO),
                     ==, 0x00002000);
}

/* ------------------------------------------------------------------ */
/* Page sizes                                                          */
/* ------------------------------------------------------------------ */

static void test_page_sizes(void)
{
    const uint64_t va = 0x0000000092345678ull;
    RvtngMmuCore c;
    FakePageTable t;
    RvtngMmuFault fault;
    RvtngMmuPageSize size;
    uint64_t pa, vpn;

    /* 4 KiB leaf: three-level walk. */
    memset(&t, 0, sizeof(t));
    install_4k(&t, va, kLeaf, PTE_V | PTE_R | PTE_W, PTE_V, PTE_V);
    setup_svm(&c, &t);
    pa = rvtng_mmu_core_translate(&c, va + 0x321, RVTNG_MMU_WRITE, 0,
                                  &fault, &size);
    g_assert_cmpuint(fault, ==, RVTNG_MMU_FAULT_NONE);
    g_assert_cmpuint(size, ==, RVTNG_MMU_PAGE_4K);
    g_assert_cmpuint(pa, ==, kLeaf + ((va + 0x321) & 0xfff));
    /* Second access must hit the promoted L1 entry: no new PTW reads. */
    t.reads = 0;
    pa = rvtng_mmu_core_translate(&c, va + 0x322, RVTNG_MMU_READ, 0,
                                  &fault, NULL);
    g_assert_cmpuint(fault, ==, RVTNG_MMU_FAULT_NONE);
    g_assert_cmpuint(pa, ==, kLeaf + ((va + 0x322) & 0xfff));
    g_assert_cmpuint(t.reads, ==, 0);

    /* 2 MiB superpage. */
    memset(&t, 0, sizeof(t));
    vpn = vpn_of(va);
    fake_put(&t, pte_addr(kRoot, (vpn >> 18) & 0x1ff),
             pte_for(kL1Table, PTE_V));
    fake_put(&t, pte_addr(kL1Table, (vpn >> 9) & 0x1ff),
             pte_for(0x0000000040000000ull, PTE_V | PTE_R));
    setup_svm(&c, &t);
    pa = rvtng_mmu_core_translate(&c, va, RVTNG_MMU_READ, 0, &fault, &size);
    g_assert_cmpuint(fault, ==, RVTNG_MMU_FAULT_NONE);
    g_assert_cmpuint(size, ==, RVTNG_MMU_PAGE_2M);
    g_assert_cmpuint(pa, ==, 0x40000000ull + (va & 0x1fffff));

    /* 1 GiB superpage. */
    memset(&t, 0, sizeof(t));
    vpn = vpn_of(va);
    fake_put(&t, pte_addr(kRoot, (vpn >> 18) & 0x1ff),
             pte_for(0x0000000080000000ull, PTE_V | PTE_R | PTE_X));
    setup_svm(&c, &t);
    pa = rvtng_mmu_core_translate(&c, va, RVTNG_MMU_EXECUTE, 0,
                                  &fault, &size);
    g_assert_cmpuint(fault, ==, RVTNG_MMU_FAULT_NONE);
    g_assert_cmpuint(size, ==, RVTNG_MMU_PAGE_1G);
    g_assert_cmpuint(pa, ==, 0x80000000ull + (va & 0x3fffffffull));
}

/* ------------------------------------------------------------------ */
/* Faults, permissions and redirect                                    */
/* ------------------------------------------------------------------ */

static void test_faults_and_permissions(void)
{
    const uint64_t va = 0x0000000080456789ull;
    RvtngMmuCore c;
    FakePageTable t;
    RvtngMmuFault fault;
    uint64_t pa, vpn;

    memset(&t, 0, sizeof(t));
    install_4k(&t, va, kLeaf, PTE_V | PTE_R, PTE_V, PTE_V);
    setup_svm(&c, &t);
    setup_redirects(&c);

    pa = rvtng_mmu_core_translate(&c, va, RVTNG_MMU_READ, 0, &fault, NULL);
    g_assert_cmpuint(fault, ==, RVTNG_MMU_FAULT_NONE);
    g_assert_cmpuint(pa, ==, kLeaf + (va & 0xfff));

    pa = rvtng_mmu_core_translate(&c, va, RVTNG_MMU_WRITE, 0, &fault, NULL);
    g_assert_cmpuint(fault, ==, RVTNG_MMU_FAULT_WRITE_PERM);
    g_assert_cmpuint(pa, ==, 0x30000000ull + (va & 0xfff));

    pa = rvtng_mmu_core_translate(&c, va, RVTNG_MMU_EXECUTE, 0, &fault, NULL);
    g_assert_cmpuint(fault, ==, RVTNG_MMU_FAULT_EXEC_PERM);
    g_assert_cmpuint(pa, ==, 0x40000000ull + (va & 0xfff));

    /* V=0 leaf -> page-fault redirect class. */
    memset(&t, 0, sizeof(t));
    setup_svm(&c, &t);
    setup_redirects(&c);
    pa = rvtng_mmu_core_translate(&c, va, RVTNG_MMU_READ, 0, &fault, NULL);
    g_assert_cmpuint(fault, ==, RVTNG_MMU_FAULT_PTE_INVALID);
    g_assert_cmpuint(pa, ==, 0x10000000ull + (va & 0xfff));

    /* W=1,R=0 pointer is illegal. */
    memset(&t, 0, sizeof(t));
    vpn = vpn_of(va);
    fake_put(&t, pte_addr(kRoot, (vpn >> 18) & 0x1ff),
             pte_for(kL1Table, PTE_V | PTE_W));
    setup_svm(&c, &t);
    pa = rvtng_mmu_core_translate(&c, va, RVTNG_MMU_READ, 0, &fault, NULL);
    g_assert_cmpuint(fault, ==, RVTNG_MMU_FAULT_ILLEGAL_PTE);

    /* Misaligned 2 MiB superpage PPN is illegal. */
    memset(&t, 0, sizeof(t));
    vpn = vpn_of(va);
    fake_put(&t, pte_addr(kRoot, (vpn >> 18) & 0x1ff),
             pte_for(kL1Table, PTE_V));
    fake_put(&t, pte_addr(kL1Table, (vpn >> 9) & 0x1ff),
             pte_for(0x80002000ull, PTE_V | PTE_R));
    setup_svm(&c, &t);
    pa = rvtng_mmu_core_translate(&c, va, RVTNG_MMU_READ, 0, &fault, NULL);
    g_assert_cmpuint(fault, ==, RVTNG_MMU_FAULT_ILLEGAL_PTE);
}

/* ------------------------------------------------------------------ */
/* Negative cache and flush                                            */
/* ------------------------------------------------------------------ */

static void test_negative_cache_and_flush(void)
{
    const uint64_t va = 0x0000000080123456ull;
    RvtngMmuCore c;
    FakePageTable t;
    RvtngMmuFault fault;
    uint64_t pa;
    size_t reads_after_fault;

    memset(&t, 0, sizeof(t));
    t.response = RVTNG_MMU_PTE_BUS_ERROR;
    setup_svm(&c, &t);

    pa = rvtng_mmu_core_translate(&c, va, RVTNG_MMU_READ, 0, &fault, NULL);
    g_assert_cmpuint(fault, ==, RVTNG_MMU_FAULT_PTW_BUS_ERROR);
    reads_after_fault = t.reads;

    /* Repair the page table without flushing: the negative L2 entry
     * keeps returning the cached fault (spec: flush is software's
     * job).  The retry is served by L2 with no PTW reads. */
    t.response = RVTNG_MMU_PTE_OKAY;
    install_4k(&t, va, kLeaf, PTE_V | PTE_R, PTE_V, PTE_V);
    pa = rvtng_mmu_core_translate(&c, va, RVTNG_MMU_READ, 0, &fault, NULL);
    g_assert_cmpuint(fault, ==, RVTNG_MMU_FAULT_PTW_BUS_ERROR);
    g_assert_cmpuint(t.reads, ==, reads_after_fault);

    /* Flush clears the negative entry; the retry walks again. */
    rvtng_mmu_core_flush(&c, false, 0);
    g_assert_true(c.flush_done);
    pa = rvtng_mmu_core_translate(&c, va, RVTNG_MMU_READ, 0, &fault, NULL);
    g_assert_cmpuint(fault, ==, RVTNG_MMU_FAULT_NONE);
    g_assert_cmpuint(pa, ==, kLeaf + (va & 0xfff));
    rvtng_mmu_core_ack_flush(&c);
    g_assert_false(c.flush_done);

    /* Timeout transport. */
    memset(&t, 0, sizeof(t));
    t.response = RVTNG_MMU_PTE_TIMEOUT;
    setup_svm(&c, &t);
    pa = rvtng_mmu_core_translate(&c, va, RVTNG_MMU_READ, 0, &fault, NULL);
    g_assert_cmpuint(fault, ==, RVTNG_MMU_FAULT_PTW_TIMEOUT);
}

/* ------------------------------------------------------------------ */
/* APB image, FixMap application, flush handshake                      */
/* ------------------------------------------------------------------ */

static void test_apb_and_snapshot(void)
{
    RvtngMmuCore c;
    uint64_t pa;
    RvtngMmuFault fault;

    rvtng_mmu_core_init(&c);

    /* Misaligned FIX_MAP_BASE low half: accepted, silently ignored. */
    g_assert_true(rvtng_mmu_core_write_reg(&c, RVTNG_MMU_REG_FIX_MAP_BASE_LO,
                                           0x00000123));
    g_assert_cmpuint(rvtng_mmu_core_read_reg(&c, RVTNG_MMU_REG_FIX_MAP_BASE_LO),
                     ==, 0);
    g_assert_true(rvtng_mmu_core_write_reg(&c, RVTNG_MMU_REG_FIX_MAP_BASE_LO,
                                           0x00123000));
    g_assert_cmpuint(rvtng_mmu_core_read_reg(&c, RVTNG_MMU_REG_FIX_MAP_BASE_LO),
                     ==, 0x00123000);
    g_assert_true(rvtng_mmu_core_write_reg(&c, RVTNG_MMU_REG_FIX_MAP_BASE_HI,
                                           0x00000001));
    g_assert_cmpuint(rvtng_mmu_core_read_reg(&c, RVTNG_MMU_REG_FIX_MAP_BASE_HI),
                     ==, 1);
    g_assert_true(rvtng_mmu_core_write_reg(&c, RVTNG_MMU_REG_N_FIX_MAP_EN, 1));
    g_assert_cmpuint(rvtng_mmu_core_read_reg(&c, RVTNG_MMU_REG_N_FIX_MAP_EN),
                     ==, 1);
    g_assert_true(rvtng_mmu_core_write_reg(
        &c, RVTNG_MMU_REG_ERROR_SLV_OFF_MODE, 1));
    g_assert_cmpuint(
        rvtng_mmu_core_read_reg(&c, RVTNG_MMU_REG_ERROR_SLV_OFF_MODE), ==, 1);
    g_assert_true(rvtng_mmu_core_write_reg(&c, RVTNG_MMU_REG_ERROR_SLV_MASK_LO,
                                           0x00001000));
    g_assert_true(rvtng_mmu_core_write_reg(&c, RVTNG_MMU_REG_ERROR_SLV_MASK_HI,
                                           0x00000002));
    g_assert_cmpuint(
        rvtng_mmu_core_read_reg(&c, RVTNG_MMU_REG_ERROR_SLV_MASK_HI), ==, 2);

    /* Config changes take effect on subsequent translations (the
     * snapshot at translate() entry is the software stand-in for the
     * slave AR/AW acceptance handshake). */
    g_assert_true(rvtng_mmu_core_write_reg(&c, RVTNG_MMU_REG_N_FIX_MAP_EN, 0));
    g_assert_true(rvtng_mmu_core_write_reg(&c, RVTNG_MMU_REG_FIX_MAP_BASE_LO,
                                           0x00001000));
    g_assert_true(rvtng_mmu_core_write_reg(&c, RVTNG_MMU_REG_FIX_MAP_BASE_HI,
                                           0));
    pa = rvtng_mmu_core_translate(&c, 0x80002000, RVTNG_MMU_READ, 0, &fault, NULL);
    g_assert_cmpuint(fault, ==, RVTNG_MMU_FAULT_NONE);
    g_assert_cmpuint(pa, ==, 0x80003000);

    /* Flush handshake through the register image. */
    g_assert_true(rvtng_mmu_core_write_reg(&c, RVTNG_MMU_REG_TLB_FLUSH, 1));
    g_assert_cmpuint(rvtng_mmu_core_read_reg(&c, RVTNG_MMU_REG_FLUSH_DONE),
                     ==, 1);
    g_assert_cmpuint(rvtng_mmu_core_read_reg(&c, RVTNG_MMU_REG_FLUSH_BUSY),
                     ==, 0);
    g_assert_cmpuint(rvtng_mmu_core_read_reg(&c, RVTNG_MMU_REG_FLUSH_ERR_CODE),
                     ==, 0);
    g_assert_true(rvtng_mmu_core_write_reg(&c, RVTNG_MMU_REG_FLUSH_ACK, 1));
    g_assert_cmpuint(rvtng_mmu_core_read_reg(&c, RVTNG_MMU_REG_FLUSH_DONE),
                     ==, 0);

    /* Undefined address: write rejected, read zero. */
    g_assert_false(rvtng_mmu_core_write_reg(&c, 0xffff, 1));
    g_assert_cmpuint(rvtng_mmu_core_read_reg(&c, 0xffff), ==, 0);
    /* Read-only status registers reject writes too. */
    g_assert_false(rvtng_mmu_core_write_reg(&c, RVTNG_MMU_REG_FLUSH_DONE, 1));
}

/* ------------------------------------------------------------------ */
/* DRAM reader bounds                                                  */
/* ------------------------------------------------------------------ */

static void test_dram_reader_bounds(void)
{
    RvtngMmuCore c;
    uint8_t bytes[16] = { 0 };
    uint64_t value = 0x1122334455667788ull;
    RvtngMmuFault fault;
    uint64_t pa;

    rvtng_mmu_core_init(&c);
    memcpy(bytes, &value, sizeof(value));
    /* VA 0x80000000 roots its walk at root index 2, i.e. PTW page base
     * + 0x10; place the image on that slot. */
    rvtng_mmu_core_bind_dram(&c, bytes, 0x1010, sizeof(bytes));

    g_assert_true(rvtng_mmu_core_write_reg(&c, RVTNG_MMU_REG_N_FIX_MAP_EN, 1));

    /* The DRAM image acts as the page table; the root PTE read at
     * 0x1010 must observe the stored value (V=0 -> pte_invalid proves
     * the data path). */
    g_assert_true(rvtng_mmu_core_write_reg(
        &c, RVTNG_MMU_REG_PTW_BASE_LO, 0x00001010));
    pa = rvtng_mmu_core_translate(&c, 0x80000000, RVTNG_MMU_READ, 0, &fault, NULL);
    g_assert_cmpuint(fault, ==, RVTNG_MMU_FAULT_PTE_INVALID);
    /* Redirect bases are zero here: the fault PA is the bare offset. */
    g_assert_cmpuint(pa, ==, 0);

    /* Drop the negative entry cached for VA 0x80000000 before the
     * second probe. */
    rvtng_mmu_core_flush(&c, false, 0);
    rvtng_mmu_core_ack_flush(&c);

    /* Out-of-image PTW reads are bus errors: base 0xff0 truncates to
     * page 0, so the root slot read lands at 0x10. */
    g_assert_true(rvtng_mmu_core_write_reg(
        &c, RVTNG_MMU_REG_PTW_BASE_LO, 0x00000ff0));
    pa = rvtng_mmu_core_translate(&c, 0x80000000, RVTNG_MMU_READ, 0, &fault, NULL);
    g_assert_cmpuint(fault, ==, RVTNG_MMU_FAULT_PTW_BUS_ERROR);
    g_assert_cmpuint(pa, ==, 0);
}

/* ------------------------------------------------------------------ */
/* ASID flush                                                          */
/* ------------------------------------------------------------------ */

static void test_asid_flush(void)
{
    const uint64_t va = 0x0000000080567000ull;
    RvtngMmuCore c;
    FakePageTable t;
    RvtngMmuFault fault;
    size_t before_flush;

    memset(&t, 0, sizeof(t));
    install_4k(&t, va, kLeaf, PTE_V | PTE_R, PTE_V, PTE_V);
    setup_svm(&c, &t);

    g_assert_true(rvtng_mmu_core_write_reg(&c, RVTNG_MMU_REG_GLOBAL_ASID, 1));
    (void)rvtng_mmu_core_translate(&c, va, RVTNG_MMU_READ, 0, &fault, NULL);
    g_assert_cmpuint(fault, ==, RVTNG_MMU_FAULT_NONE);
    g_assert_true(rvtng_mmu_core_write_reg(&c, RVTNG_MMU_REG_GLOBAL_ASID, 2));
    (void)rvtng_mmu_core_translate(&c, va, RVTNG_MMU_READ, 0, &fault, NULL);
    g_assert_cmpuint(fault, ==, RVTNG_MMU_FAULT_NONE);
    before_flush = t.reads;

    /* ASID flush drops ASID-1 entries only; ASID-2 keeps its hit. */
    rvtng_mmu_core_flush(&c, true, 1);
    g_assert_true(rvtng_mmu_core_write_reg(&c, RVTNG_MMU_REG_GLOBAL_ASID, 2));
    (void)rvtng_mmu_core_translate(&c, va, RVTNG_MMU_READ, 0, &fault, NULL);
    g_assert_cmpuint(fault, ==, RVTNG_MMU_FAULT_NONE);
    g_assert_cmpuint(t.reads, ==, before_flush);

    g_assert_true(rvtng_mmu_core_write_reg(&c, RVTNG_MMU_REG_GLOBAL_ASID, 1));
    (void)rvtng_mmu_core_translate(&c, va, RVTNG_MMU_READ, 0, &fault, NULL);
    g_assert_cmpuint(fault, ==, RVTNG_MMU_FAULT_NONE);
    g_assert_cmpuint(t.reads, >, before_flush);
}

/* ------------------------------------------------------------------ */
/* PDC global inheritance (RTL boundary, spec 15.2)                    */
/* ------------------------------------------------------------------ */

static void test_global_pdc_rtl_behavior(void)
{
    const uint64_t va0 = 0x0000000080011000ull;
    const uint64_t va1 = 0x0000000080012000ull;
    RvtngMmuCore c;
    FakePageTable t;
    RvtngMmuFault fault;
    size_t reads_before;
    uint64_t vpn1, pa;

    memset(&t, 0, sizeof(t));
    install_4k(&t, va0, kLeaf, PTE_V | PTE_R, PTE_V | PTE_G, PTE_V);
    vpn1 = vpn_of(va1);
    fake_put(&t, pte_addr(kL2Table, vpn1 & 0x1ff),
             pte_for(kLeaf + 0x1000, PTE_V | PTE_R));
    setup_svm(&c, &t);

    g_assert_true(rvtng_mmu_core_write_reg(&c, RVTNG_MMU_REG_GLOBAL_ASID, 1));
    pa = rvtng_mmu_core_translate(&c, va0, RVTNG_MMU_READ, 0, &fault, NULL);
    g_assert_cmpuint(fault, ==, RVTNG_MMU_FAULT_NONE);
    g_assert_cmpuint(pa, ==, kLeaf);

    /* A second ASID walking a page that shares both non-leaf pointers
     * hits the PDC: the L0 read is skipped (2 reads instead of 3). */
    g_assert_true(rvtng_mmu_core_write_reg(&c, RVTNG_MMU_REG_GLOBAL_ASID, 2));
    reads_before = t.reads;
    pa = rvtng_mmu_core_translate(&c, va1, RVTNG_MMU_READ, 0, &fault, NULL);
    g_assert_cmpuint(fault, ==, RVTNG_MMU_FAULT_NONE);
    g_assert_cmpuint(pa, ==, kLeaf + 0x1000);
    g_assert_cmpuint(t.reads - reads_before, ==, 2);

    /* RTL boundary: the G bit of the PDC-restored non-leaf is not
     * inherited, so the entry built for ASID 2 is not global and a
     * third ASID must re-walk (through the still-global L0 PDC hit). */
    g_assert_true(rvtng_mmu_core_write_reg(&c, RVTNG_MMU_REG_GLOBAL_ASID, 3));
    reads_before = t.reads;
    (void)rvtng_mmu_core_translate(&c, va1, RVTNG_MMU_READ, 0, &fault, NULL);
    g_assert_cmpuint(fault, ==, RVTNG_MMU_FAULT_NONE);
    g_assert_cmpuint(t.reads, >, reads_before);
}

/* ------------------------------------------------------------------ */
/* Overlap priority and shared L2                                      */
/* ------------------------------------------------------------------ */

static void test_overlap_and_ports(void)
{
    const uint64_t va = 0x0000000081234567ull;
    RvtngMmuCore c;
    FakePageTable t;
    RvtngMmuFault fault;
    RvtngMmuPageSize size;
    uint64_t vpn, pa;

    /* 4 KiB leaf first. */
    memset(&t, 0, sizeof(t));
    install_4k(&t, va, kLeaf, PTE_V | PTE_R, PTE_V, PTE_V);
    setup_svm(&c, &t);
    pa = rvtng_mmu_core_translate(&c, va, RVTNG_MMU_READ, 0, &fault, &size);
    g_assert_cmpuint(size, ==, RVTNG_MMU_PAGE_4K);
    g_assert_cmpuint(pa, ==, kLeaf + (va & 0xfff));

    /* Rewrite the middle pointer into a 2 MiB leaf covering the same
     * VA, without flushing: the cached 4 KiB entry must win (overlap
     * priority 4K > 2M).  Port 1 has an empty L1, so the hit comes
     * from the shared L2 with no PTW reads. */
    vpn = vpn_of(va);
    fake_put(&t, pte_addr(kL1Table, (vpn >> 9) & 0x1ff),
             pte_for(0x40000000ull, PTE_V | PTE_R));
    t.reads = 0;
    pa = rvtng_mmu_core_translate(&c, va, RVTNG_MMU_READ, 1, &fault, &size);
    g_assert_cmpuint(size, ==, RVTNG_MMU_PAGE_4K);
    g_assert_cmpuint(pa, ==, kLeaf + (va & 0xfff));
    g_assert_cmpuint(t.reads, ==, 0);

    /* L2 is shared across ports: a fresh 2 MiB mapping walked by
     * port 0 is hit by port 1 without a walk. */
    rvtng_mmu_core_flush(&c, false, 0);
    rvtng_mmu_core_ack_flush(&c);
    memset(&t, 0, sizeof(t));
    vpn = vpn_of(va);
    fake_put(&t, pte_addr(kRoot, (vpn >> 18) & 0x1ff),
             pte_for(kL1Table, PTE_V));
    fake_put(&t, pte_addr(kL1Table, (vpn >> 9) & 0x1ff),
             pte_for(0x40000000ull, PTE_V | PTE_R));
    setup_svm(&c, &t);
    pa = rvtng_mmu_core_translate(&c, va, RVTNG_MMU_READ, 0, &fault, &size);
    g_assert_cmpuint(size, ==, RVTNG_MMU_PAGE_2M);
    g_assert_cmpuint(pa, ==, 0x40000000ull + (va & 0x1fffff));
    t.reads = 0;
    pa = rvtng_mmu_core_translate(&c, va, RVTNG_MMU_READ, 1, &fault, &size);
    g_assert_cmpuint(size, ==, RVTNG_MMU_PAGE_2M);
    g_assert_cmpuint(pa, ==, 0x40000000ull + (va & 0x1fffff));
    g_assert_cmpuint(t.reads, ==, 0);
}

/* ------------------------------------------------------------------ */
/* 2 GiB bus boundary                                                  */
/* ------------------------------------------------------------------ */

static void test_below_dram_identity(void)
{
    const uint64_t va = 0x1234;
    RvtngMmuCore c;
    FakePageTable t;
    RvtngMmuFault fault;
    RvtngMmuPageSize size;
    uint64_t pa;

    /* Both relocation mechanisms are armed against va: a page-table
     * leaf and a relocating full-range FixMap.  Either one alone would
     * move the access if it reached the engine; the 2 GiB bus boundary
     * routes it around both. */
    memset(&t, 0, sizeof(t));
    install_4k(&t, va, kLeaf, PTE_V | PTE_R, PTE_V, PTE_V);
    install_4k(&t, 0x80001234, kLeaf, PTE_V | PTE_R, PTE_V, PTE_V);
    setup_svm(&c, &t);
    g_assert_true(rvtng_mmu_core_write_reg(&c, RVTNG_MMU_REG_FIX_MAP_BASE_LO,
                                           0x90000000));

    pa = rvtng_mmu_core_translate(&c, va, RVTNG_MMU_READ, 0, &fault, &size);
    g_assert_cmpuint(fault, ==, RVTNG_MMU_FAULT_NONE);
    g_assert_cmpuint(size, ==, RVTNG_MMU_PAGE_4K);
    g_assert_cmpuint(pa, ==, va);
    g_assert_cmpuint(t.reads, ==, 0);

    /* Control: the same page offset above the base is translated, so
     * the identity result above is the boundary rule at work, not a
     * dead page table or FixMap. */
    g_assert_true(rvtng_mmu_core_write_reg(&c, RVTNG_MMU_REG_N_FIX_MAP_EN, 1));
    pa = rvtng_mmu_core_translate(&c, 0x80001234, RVTNG_MMU_READ, 0,
                                  &fault, NULL);
    g_assert_cmpuint(fault, ==, RVTNG_MMU_FAULT_NONE);
    g_assert_cmpuint(pa, ==, kLeaf + 0x234);
    g_assert_cmpuint(t.reads, ==, 3);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    g_test_add_func("/rvtng-mmu/fixmap", test_fixmap);
    g_test_add_func("/rvtng-mmu/below-dram-identity", test_below_dram_identity);
    g_test_add_func("/rvtng-mmu/page-sizes", test_page_sizes);
    g_test_add_func("/rvtng-mmu/faults-permissions",
                    test_faults_and_permissions);
    g_test_add_func("/rvtng-mmu/negative-cache-flush",
                    test_negative_cache_and_flush);
    g_test_add_func("/rvtng-mmu/apb-snapshot", test_apb_and_snapshot);
    g_test_add_func("/rvtng-mmu/dram-reader-bounds", test_dram_reader_bounds);
    g_test_add_func("/rvtng-mmu/asid-flush", test_asid_flush);
    g_test_add_func("/rvtng-mmu/global-pdc", test_global_pdc_rtl_behavior);
    g_test_add_func("/rvtng-mmu/overlap-ports", test_overlap_and_ports);

    return g_test_run();
}
