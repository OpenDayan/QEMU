/*
 * QTest testcase for the RVTNG-V0 ClusterMMU APB4 register image
 *
 * Copyright (c) 2026 Institute of Computing Technology,
 * Chinese Academy of Sciences
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Covers the APB-visible behaviour of the ClusterMMU front end:
 * reset defaults, register write/read semantics (including the
 * 4K-alignment silent-ignore rule), the TLB_FLUSH/ACK handshake and
 * the undefined-address behaviour.  The translation datapath itself
 * is covered by tests/unit/test-rvtng-mmu.c.
 */

#include "qemu/osdep.h"
#include "hw/riscv/rvtng_mmu_core.h"
#include "libqtest.h"

#define RVTNG_DRAM_SHM_NAME "/dram_shm_0"
#define RVTNG_DRAM_SIZE     (16ULL * 1024 * 1024 * 1024)

#define RVTNG_MMU_BASE 0x5000ull

static char *firmware_path;
static bool owns_dram_shm;

static void rvtng_mmu_test_cleanup(void)
{
    if (firmware_path) {
        unlink(firmware_path);
        g_clear_pointer(&firmware_path, g_free);
    }
    if (owns_dram_shm) {
        shm_unlink(RVTNG_DRAM_SHM_NAME);
        owns_dram_shm = false;
    }
}

static void rvtng_mmu_test_abrt(void *opaque)
{
    rvtng_mmu_test_cleanup();
}

static bool rvtng_mmu_test_prepare(void)
{
    char uram_name[32];
    int dram_fd;
    int firmware_fd;
    unsigned i;

    for (i = 0; i < 4; i++) {
        int uram_fd;

        snprintf(uram_name, sizeof(uram_name), "/uram_shm_%u", i);
        uram_fd = shm_open(uram_name, O_RDONLY, 0);
        if (uram_fd >= 0) {
            close(uram_fd);
            g_test_skip("an RVTNG URAM shared-memory object is in use");
            return false;
        }
        if (errno != ENOENT) {
            g_test_skip("cannot safely inspect RVTNG URAM shared memory");
            return false;
        }
    }

    dram_fd = shm_open(RVTNG_DRAM_SHM_NAME,
                       O_CREAT | O_EXCL | O_RDWR, 0600);
    if (dram_fd < 0 && errno == EEXIST) {
        g_test_skip("/dram_shm_0 is already in use");
        return false;
    }
    g_assert_cmpint(dram_fd, >=, 0);
    owns_dram_shm = true;
    g_assert_cmpint(ftruncate(dram_fd, RVTNG_DRAM_SIZE), ==, 0);
    close(dram_fd);

    firmware_fd = g_file_open_tmp("rvtng-mmu-qtest-XXXXXX.bin",
                                  &firmware_path, NULL);
    g_assert_cmpint(firmware_fd, >=, 0);
    g_assert_cmpint(ftruncate(firmware_fd, 4), ==, 0);
    close(firmware_fd);

    return true;
}

static QTestState *rvtng_mmu_qtest_init(const char *args)
{
    QTestState *qts;
    int saved_stdout;

    /* The platform's shared-memory diagnostics must not corrupt TAP output. */
    fflush(stdout);
    saved_stdout = dup(STDOUT_FILENO);
    g_assert_cmpint(saved_stdout, >=, 0);
    g_assert_cmpint(dup2(STDERR_FILENO, STDOUT_FILENO), >=, 0);
    qts = qtest_init(args);
    fflush(stdout);
    g_assert_cmpint(dup2(saved_stdout, STDOUT_FILENO), >=, 0);
    close(saved_stdout);

    return qts;
}

static uint64_t mmu_read(QTestState *qts, uint16_t off)
{
    return qtest_readl(qts, RVTNG_MMU_BASE + off);
}

static void mmu_write(QTestState *qts, uint16_t off, uint32_t val)
{
    qtest_writel(qts, RVTNG_MMU_BASE + off, val);
}

static void test_reset_defaults(QTestState *qts)
{
    static const uint16_t ro_offsets[] = {
        RVTNG_MMU_REG_TLB_FLUSH,
        RVTNG_MMU_REG_FLUSH_ACK,
        RVTNG_MMU_REG_ERROR_SLV_CFG_CLR,
    };
    unsigned i;

    g_assert_cmphex(mmu_read(qts, RVTNG_MMU_REG_PTW_BASE_LO), ==, 0);
    g_assert_cmphex(mmu_read(qts, RVTNG_MMU_REG_PTW_BASE_HI), ==, 0);
    g_assert_cmphex(mmu_read(qts, RVTNG_MMU_REG_FIX_MAP_BASE_LO), ==, 0);
    g_assert_cmphex(mmu_read(qts, RVTNG_MMU_REG_FIX_MAP_BASE_HI), ==, 0);
    g_assert_cmphex(mmu_read(qts, RVTNG_MMU_REG_N_FIX_MAP_EN), ==, 0);
    g_assert_cmphex(mmu_read(qts, RVTNG_MMU_REG_FLUSH_DONE), ==, 0);
    g_assert_cmphex(mmu_read(qts, RVTNG_MMU_REG_FLUSH_BUSY), ==, 0);
    g_assert_cmphex(mmu_read(qts, RVTNG_MMU_REG_FLUSH_ERR_CODE), ==, 0);
    g_assert_cmphex(mmu_read(qts, RVTNG_MMU_REG_GLOBAL_ASID), ==, 0);
    g_assert_cmphex(mmu_read(qts, RVTNG_MMU_REG_ERROR_SLV_MASK_LO), ==, 0);
    g_assert_cmphex(mmu_read(qts, RVTNG_MMU_REG_ERROR_SLV_MASK_HI), ==, 0);
    g_assert_cmphex(mmu_read(qts, RVTNG_MMU_REG_ERROR_SLV_OFF_MODE), ==, 0);
    g_assert_cmphex(mmu_read(qts, RVTNG_MMU_REG_ERROR_SLV_CFG_ERR), ==, 0);
    g_assert_cmphex(mmu_read(qts, RVTNG_MMU_REG_REDIR_PAGE_FAULT_LO), ==, 0);
    g_assert_cmphex(mmu_read(qts, RVTNG_MMU_REG_REDIR_PAGE_FAULT_HI), ==, 0);
    g_assert_cmphex(mmu_read(qts, RVTNG_MMU_REG_REDIR_READ_PERM_LO), ==, 0);
    g_assert_cmphex(mmu_read(qts, RVTNG_MMU_REG_REDIR_WRITE_PERM_LO), ==, 0);
    g_assert_cmphex(mmu_read(qts, RVTNG_MMU_REG_REDIR_EXEC_PERM_LO), ==, 0);
    g_assert_cmphex(mmu_read(qts, RVTNG_MMU_REG_REDIR_PTW_ERR_LO), ==, 0);
    g_assert_cmphex(mmu_read(qts, RVTNG_MMU_REG_REDIR_DEFAULT_LO), ==, 0);
    g_assert_cmphex(mmu_read(qts, RVTNG_MMU_REG_FIX_MAP_SIZE_LO), ==, 0);
    g_assert_cmphex(mmu_read(qts, RVTNG_MMU_REG_FIX_MAP_SIZE_HI), ==, 0);

    /* Every defined offset answers; a walk over the whole page must not
     * fault and undefined windows read as zero. */
    for (i = 0; i < RVTNG_MMU_APB_SIZE; i += 4) {
        mmu_read(qts, (uint16_t)i);
    }
    for (i = 0; i < ARRAY_SIZE(ro_offsets); i++) {
        mmu_write(qts, ro_offsets[i], UINT32_MAX);
    }
    g_assert_cmphex(mmu_read(qts, RVTNG_MMU_REG_FLUSH_DONE), ==, 0);
}

static void test_config_write_read(QTestState *qts)
{
    /* PTW base: full 56-bit assembly from the two halves. */
    mmu_write(qts, RVTNG_MMU_REG_PTW_BASE_LO, 0x80000000);
    mmu_write(qts, RVTNG_MMU_REG_PTW_BASE_HI, 0x00abcdef);
    g_assert_cmphex(mmu_read(qts, RVTNG_MMU_REG_PTW_BASE_LO), ==, 0x80000000);
    g_assert_cmphex(mmu_read(qts, RVTNG_MMU_REG_PTW_BASE_HI), ==, 0x00abcdef);
    mmu_write(qts, RVTNG_MMU_REG_PTW_BASE_LO, 0);
    mmu_write(qts, RVTNG_MMU_REG_PTW_BASE_HI, 0);

    /* N_FIX_MAP_EN keeps bit 0 only. */
    mmu_write(qts, RVTNG_MMU_REG_N_FIX_MAP_EN, 3);
    g_assert_cmphex(mmu_read(qts, RVTNG_MMU_REG_N_FIX_MAP_EN), ==, 1);
    mmu_write(qts, RVTNG_MMU_REG_N_FIX_MAP_EN, 0);

    /* GLOBAL_ASID keeps the low byte. */
    mmu_write(qts, RVTNG_MMU_REG_GLOBAL_ASID, 0x1a5);
    g_assert_cmphex(mmu_read(qts, RVTNG_MMU_REG_GLOBAL_ASID), ==, 0xa5);
    mmu_write(qts, RVTNG_MMU_REG_GLOBAL_ASID, 0);

    /* ERROR_SLV_MASK low half needs 4K alignment: misaligned ignored. */
    mmu_write(qts, RVTNG_MMU_REG_ERROR_SLV_MASK_LO, 0x123);
    g_assert_cmphex(mmu_read(qts, RVTNG_MMU_REG_ERROR_SLV_MASK_LO), ==, 0);
    mmu_write(qts, RVTNG_MMU_REG_ERROR_SLV_MASK_LO, 0x1000);
    g_assert_cmphex(mmu_read(qts, RVTNG_MMU_REG_ERROR_SLV_MASK_LO), ==, 0x1000);
    mmu_write(qts, RVTNG_MMU_REG_ERROR_SLV_MASK_LO, 0);

    /* Same silent-ignore rule for the redirect windows. */
    mmu_write(qts, RVTNG_MMU_REG_REDIR_PAGE_FAULT_LO, 0x7);
    g_assert_cmphex(mmu_read(qts, RVTNG_MMU_REG_REDIR_PAGE_FAULT_LO), ==, 0);
    mmu_write(qts, RVTNG_MMU_REG_REDIR_PAGE_FAULT_LO, 0x2000);
    g_assert_cmphex(mmu_read(qts, RVTNG_MMU_REG_REDIR_PAGE_FAULT_LO), ==,
                    0x2000);
    mmu_write(qts, RVTNG_MMU_REG_REDIR_PAGE_FAULT_LO, 0);

    /* FIX_MAP_SIZE has no alignment constraint. */
    mmu_write(qts, RVTNG_MMU_REG_FIX_MAP_SIZE_LO, 0x12345678);
    g_assert_cmphex(mmu_read(qts, RVTNG_MMU_REG_FIX_MAP_SIZE_LO), ==,
                    0x12345678);
    mmu_write(qts, RVTNG_MMU_REG_FIX_MAP_SIZE_LO, 0);

    /* ERROR_SLV_OFF_MODE keeps bit 0. */
    mmu_write(qts, RVTNG_MMU_REG_ERROR_SLV_OFF_MODE, 2);
    g_assert_cmphex(mmu_read(qts, RVTNG_MMU_REG_ERROR_SLV_OFF_MODE), ==, 0);
    mmu_write(qts, RVTNG_MMU_REG_ERROR_SLV_OFF_MODE, 1);
    g_assert_cmphex(mmu_read(qts, RVTNG_MMU_REG_ERROR_SLV_OFF_MODE), ==, 1);
    mmu_write(qts, RVTNG_MMU_REG_ERROR_SLV_OFF_MODE, 0);

    /* Undefined window: writes are dropped, reads stay zero. */
    mmu_write(qts, 0x0fc, UINT32_MAX);
    mmu_write(qts, 0x2f8, UINT32_MAX);
    g_assert_cmphex(mmu_read(qts, 0x0fc), ==, 0);
    g_assert_cmphex(mmu_read(qts, 0x2f8), ==, 0);
}

static void test_flush_handshake(QTestState *qts)
{
    /* Full flush trigger. */
    mmu_write(qts, RVTNG_MMU_REG_TLB_FLUSH, 1);
    g_assert_cmphex(mmu_read(qts, RVTNG_MMU_REG_FLUSH_DONE), ==, 1);
    g_assert_cmphex(mmu_read(qts, RVTNG_MMU_REG_FLUSH_BUSY), ==, 0);
    g_assert_cmphex(mmu_read(qts, RVTNG_MMU_REG_FLUSH_ERR_CODE), ==, 0);
    /* Status registers are read-only: poking them changes nothing. */
    mmu_write(qts, RVTNG_MMU_REG_FLUSH_DONE, 0);
    g_assert_cmphex(mmu_read(qts, RVTNG_MMU_REG_FLUSH_DONE), ==, 1);
    /* Ack clears DONE. */
    mmu_write(qts, RVTNG_MMU_REG_FLUSH_ACK, 1);
    g_assert_cmphex(mmu_read(qts, RVTNG_MMU_REG_FLUSH_DONE), ==, 0);

    /* ASID-selective flush: bit1 selects ASID mode, bits[9:2] the ASID. */
    mmu_write(qts, RVTNG_MMU_REG_TLB_FLUSH, (7u << 2) | 2u | 1u);
    g_assert_cmphex(mmu_read(qts, RVTNG_MMU_REG_FLUSH_DONE), ==, 1);
    g_assert_cmphex(mmu_read(qts, RVTNG_MMU_REG_FLUSH_ERR_CODE), ==, 0);
    mmu_write(qts, RVTNG_MMU_REG_FLUSH_ACK, 1);
    g_assert_cmphex(mmu_read(qts, RVTNG_MMU_REG_FLUSH_DONE), ==, 0);

    /* Writing zero to the trigger register does nothing. */
    mmu_write(qts, RVTNG_MMU_REG_TLB_FLUSH, 0);
    g_assert_cmphex(mmu_read(qts, RVTNG_MMU_REG_FLUSH_DONE), ==, 0);
    g_assert_cmphex(mmu_read(qts, RVTNG_MMU_REG_FLUSH_BUSY), ==, 0);
}

static void test_rvtng_mmu_registers(void)
{
    g_autofree char *args = NULL;
    g_autofree char *mtree = NULL;
    QTestState *qts;

    if (!rvtng_mmu_test_prepare()) {
        return;
    }

    /* forward-config must stay off: the suite must never talk to a
     * live Dayan-emu config-sync socket.  The dotted type name requires
     * the explicit key=value -global form (the short form splits at
     * the first dot). */
    args = g_strdup_printf("-machine rvtng-v0 -smp 4 -bios %s "
                           "-global driver=rvtng.v0.mmu,"
                           "property=forward-config,value=false",
                           firmware_path);
    qts = rvtng_mmu_qtest_init(args);

    mtree = qtest_hmp(qts, "info mtree");
    g_assert_nonnull(strstr(mtree, "rvtng.v0.mmu"));

    test_reset_defaults(qts);
    test_config_write_read(qts);
    test_flush_handshake(qts);

    /* System reset returns the register image to defaults. */
    qtest_system_reset(qts);
    g_assert_cmphex(mmu_read(qts, RVTNG_MMU_REG_PTW_BASE_LO), ==, 0);
    g_assert_cmphex(mmu_read(qts, RVTNG_MMU_REG_FIX_MAP_BASE_LO), ==, 0);
    g_assert_cmphex(mmu_read(qts, RVTNG_MMU_REG_N_FIX_MAP_EN), ==, 0);
    g_assert_cmphex(mmu_read(qts, RVTNG_MMU_REG_FLUSH_DONE), ==, 0);
    g_assert_cmphex(mmu_read(qts, RVTNG_MMU_REG_GLOBAL_ASID), ==, 0);

    qtest_quit(qts);
    rvtng_mmu_test_cleanup();
}

int main(int argc, char **argv)
{
    int ret;

    g_test_init(&argc, &argv, NULL);
    qtest_add_abrt_handler(rvtng_mmu_test_abrt, NULL);

    if (qtest_has_machine("rvtng-v0")) {
        qtest_add_func("/rvtng-mmu/registers", test_rvtng_mmu_registers);
    }

    ret = g_test_run();
    rvtng_mmu_test_cleanup();
    return ret;
}
