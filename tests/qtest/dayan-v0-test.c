/*
 * QTest testcase for the dayan-v0 cluster platform
 *
 * Copyright (c) 2026 Institute of Computing Technology,
 * Chinese Academy of Sciences
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "qemu/cutils.h"
#include "hw/riscv/dayan_fuse.h"
#include "libqtest.h"

#define DAYAN_DRAM_SHM_NAME "/dram_shm_0"
#define DAYAN_DRAM_SIZE     (16ULL * 1024 * 1024 * 1024)

#define DAYAN_MAILBOX_BASE      0x1000
#define DAYAN_CLINT_BASE        0x20000
#define DAYAN_ATOMIC_CACHE_BASE 0x400000
#define DAYAN_URAM_BASE         0x800000
#define DAYAN_URAM_SIZE         0x180000
#define DAYAN_URAM_STRIDE       0x200000

#define DAYAN_CLINT_IPI_BASE       0x100
#define DAYAN_CLINT_IPI_HART_SIZE  0x10
#define DAYAN_CLINT_IPI_STATUS     0x0
#define DAYAN_CLINT_IPI_ENABLE     0x4
#define DAYAN_CLINT_IPI_PENDING    0x8
#define DAYAN_CLINT_IPI_CLEAR      0xc
#define DAYAN_CLINT_MTIMECMP       0x4000
#define DAYAN_CLINT_MTIME          0xbff8

static char *firmware_path;
static bool owns_dram_shm;

static void dayan_v0_test_cleanup(void)
{
    if (firmware_path) {
        unlink(firmware_path);
        g_clear_pointer(&firmware_path, g_free);
    }
    if (owns_dram_shm) {
        shm_unlink(DAYAN_DRAM_SHM_NAME);
        owns_dram_shm = false;
    }
}

static void dayan_v0_test_abrt(void *opaque)
{
    dayan_v0_test_cleanup();
}

static bool dayan_v0_test_prepare(void)
{
    char uram_name[32];
    int dram_fd;
    int firmware_fd;

    for (unsigned i = 0; i < 4; i++) {
        int uram_fd;

        snprintf(uram_name, sizeof(uram_name), "/uram_shm_%u", i);
        uram_fd = shm_open(uram_name, O_RDONLY, 0);
        if (uram_fd >= 0) {
            close(uram_fd);
            g_test_skip("an Dayan URAM shared-memory object is in use");
            return false;
        }
        if (errno != ENOENT) {
            g_test_skip("cannot safely inspect Dayan URAM shared memory");
            return false;
        }
    }

    dram_fd = shm_open(DAYAN_DRAM_SHM_NAME,
                       O_CREAT | O_EXCL | O_RDWR, 0600);
    if (dram_fd < 0 && errno == EEXIST) {
        g_test_skip("/dram_shm_0 is already in use");
        return false;
    }
    g_assert_cmpint(dram_fd, >=, 0);
    owns_dram_shm = true;
    g_assert_cmpint(ftruncate(dram_fd, DAYAN_DRAM_SIZE), ==, 0);
    close(dram_fd);

    firmware_fd = g_file_open_tmp("dayan-v0-qtest-XXXXXX.bin",
                                  &firmware_path, NULL);
    g_assert_cmpint(firmware_fd, >=, 0);
    g_assert_cmpint(ftruncate(firmware_fd, 4), ==, 0);
    close(firmware_fd);

    return true;
}

static uint64_t dayan_clint_ipi_reg(unsigned hart, uint64_t reg)
{
    return DAYAN_CLINT_BASE + DAYAN_CLINT_IPI_BASE +
           hart * DAYAN_CLINT_IPI_HART_SIZE + reg;
}

static QTestState *dayan_v0_qtest_init(const char *args)
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

static void test_interrupt_handler(QTestState *qts)
{
    uint64_t base = DAYAN_INTC_BASE;
    uint64_t mailbox0 = base + DAYAN_INTC_MAILBOX0_OFF;
    uint64_t mailbox31 = mailbox0 + 31 * sizeof(uint64_t);

    g_assert_cmphex(qtest_readl(qts, base + DAYAN_INTC_STATUS_OFF), ==, 0);
    g_assert_cmphex(qtest_readl(qts, base + DAYAN_INTC_ENABLE_OFF), ==, 0);
    g_assert_cmphex(qtest_readq(qts, mailbox0), ==, 1);
    g_assert_cmphex(qtest_readq(qts, mailbox31), ==, 1);

    /* STATUS and ENABLE are read-only. */
    qtest_writel(qts, base + DAYAN_INTC_STATUS_OFF, UINT32_MAX);
    qtest_writel(qts, base + DAYAN_INTC_ENABLE_OFF, UINT32_MAX);
    g_assert_cmphex(qtest_readl(qts, base + DAYAN_INTC_STATUS_OFF), ==, 0);
    g_assert_cmphex(qtest_readl(qts, base + DAYAN_INTC_ENABLE_OFF), ==, 0);

    qtest_writel(qts, base + DAYAN_INTC_SET_OFF, BIT(5));
    g_assert_cmphex(qtest_readl(qts, base + DAYAN_INTC_STATUS_OFF), ==,
                    BIT(5));
    g_assert_cmphex(qtest_readl(qts, base + DAYAN_INTC_CLEAR_OFF), ==,
                    BIT(5));
    qtest_writel(qts, base + DAYAN_INTC_CLEAR_OFF, BIT(5));
    g_assert_cmphex(qtest_readl(qts, base + DAYAN_INTC_STATUS_OFF), ==, 0);

    qtest_writel(qts, base + DAYAN_INTC_ENABLE_SET_OFF, BIT(5));
    g_assert_cmphex(qtest_readl(qts, base + DAYAN_INTC_ENABLE_OFF), ==,
                    BIT(5));
    g_assert_cmphex(qtest_readl(qts, base + DAYAN_INTC_ENABLE_CLEAR_OFF), ==,
                    BIT(5));

    qtest_writel(qts, base + DAYAN_INTC_ENABLE_CLEAR_OFF, BIT(5));
    g_assert_cmphex(qtest_readl(qts, base + DAYAN_INTC_ENABLE_OFF), ==, 0);

    qtest_writeq(qts, mailbox0, UINT64_C(0x1122334455667788));
    qtest_writeb(qts, mailbox0 + 7, 0xaa);
    g_assert_cmphex(qtest_readq(qts, mailbox0), ==,
                    UINT64_C(0xaa22334455667788));
}

static void test_atomic_cache_control(QTestState *qts)
{
    const uint64_t base = DAYAN_ATOMIC_CACHE_CONTROL_BASE;
    const uint64_t backing = UINT64_C(0x80c00000);

    g_assert_cmphex(qtest_readq(
                        qts, base + DAYAN_ATOMIC_CACHE_STATUS_OFF),
                    ==, 0);
    g_assert_cmphex(qtest_readq(
                        qts, base + DAYAN_ATOMIC_CACHE_BACKING_OFF),
                    ==, 0);

    qtest_writeq(qts, base + DAYAN_ATOMIC_CACHE_BACKING_OFF, backing);
    g_assert_cmphex(qtest_readq(
                        qts, base + DAYAN_ATOMIC_CACHE_BACKING_OFF),
                    ==, backing);
    g_assert_cmphex(qtest_readq(
                        qts, base + DAYAN_ATOMIC_CACHE_STATUS_OFF),
                    ==, DAYAN_ATOMIC_CACHE_STATUS_DONE);

    qtest_writeq(qts, base + DAYAN_ATOMIC_CACHE_STATUS_OFF, 1);
    g_assert_cmphex(qtest_readq(
                        qts, base + DAYAN_ATOMIC_CACHE_STATUS_OFF),
                    ==, DAYAN_ATOMIC_CACHE_STATUS_DONE);
}

static void test_clint(QTestState *qts)
{
    uint64_t legacy0 = DAYAN_CLINT_BASE;
    uint64_t legacy1 = DAYAN_CLINT_BASE + sizeof(uint32_t);
    uint64_t status0 = dayan_clint_ipi_reg(0, DAYAN_CLINT_IPI_STATUS);
    uint64_t enable0 = dayan_clint_ipi_reg(0, DAYAN_CLINT_IPI_ENABLE);
    uint64_t pending0 = dayan_clint_ipi_reg(0, DAYAN_CLINT_IPI_PENDING);
    uint64_t clear0 = dayan_clint_ipi_reg(0, DAYAN_CLINT_IPI_CLEAR);
    uint64_t pending1 = dayan_clint_ipi_reg(1, DAYAN_CLINT_IPI_PENDING);

    g_assert_cmphex(qtest_readl(qts, legacy0), ==, 0);
    g_assert_cmphex(qtest_readl(qts, enable0), ==, 1);
    g_assert_cmphex(qtest_readl(qts, pending0), ==, 0);

    /* SET wins when one 64-bit write touches SET and CLEAR together. */
    qtest_writeq(qts, pending0, UINT64_C(0x0000000300000003));
    g_assert_cmphex(qtest_readl(qts, pending0), ==, 3);
    qtest_writel(qts, clear0, 3);

    qtest_writeb(qts, pending0 + 1, 0x02);
    g_assert_cmphex(qtest_readl(qts, pending0), ==, 0x200);
    g_assert_cmphex(qtest_readl(qts, status0), ==, 0);

    qtest_writeb(qts, enable0 + 1, 0x02);
    g_assert_cmphex(qtest_readl(qts, enable0), ==, 0x201);
    g_assert_cmphex(qtest_readl(qts, status0), ==, 0x200);
    g_assert_cmphex(qtest_readl(qts, legacy0), ==, 0x200);

    qtest_writeb(qts, clear0 + 1, 0x02);
    g_assert_cmphex(qtest_readl(qts, pending0), ==, 0);

    qtest_writeb(qts, pending0 + 2, 0x80);
    qtest_writel(qts, legacy0, 1);
    g_assert_cmphex(qtest_readl(qts, pending0), ==, 0x800001);
    g_assert_cmphex(qtest_readl(qts, legacy0), ==, 1);

    qtest_writel(qts, legacy0, 0);
    g_assert_cmphex(qtest_readl(qts, pending0), ==, 0);

    qtest_writel(qts, pending1, 1);
    g_assert_cmphex(qtest_readl(qts, legacy1), ==, 1);
    g_assert_cmphex(qtest_readl(qts, legacy0), ==, 0);

    g_assert_cmphex(qtest_readq(qts, DAYAN_CLINT_BASE +
                                DAYAN_CLINT_MTIMECMP), ==, UINT64_MAX);
    qtest_writeq(qts, DAYAN_CLINT_BASE + DAYAN_CLINT_MTIMECMP,
                 UINT64_C(0x123456789abcdef0));
    g_assert_cmphex(qtest_readq(qts, DAYAN_CLINT_BASE +
                                DAYAN_CLINT_MTIMECMP), ==,
                    UINT64_C(0x123456789abcdef0));
    g_assert_cmphex(qtest_readq(qts, DAYAN_CLINT_BASE + DAYAN_CLINT_MTIME),
                    ==, 0);
}

static void test_memory_windows(QTestState *qts)
{
    static const struct {
        uint64_t address;
        uint64_t value;
    } accesses[] = {
        { DAYAN_MAILBOX_BASE, UINT64_C(0x0102030405060708) },
        { DAYAN_ATOMIC_CACHE_BASE, UINT64_C(0x1111111122222222) },
        { DAYAN_URAM_BASE - 8, UINT64_C(0x3333333344444444) },
        { DAYAN_URAM_BASE, UINT64_C(0x5000000000000000) },
        { DAYAN_URAM_BASE + DAYAN_URAM_STRIDE,
          UINT64_C(0x6000000000000000) },
        { DAYAN_URAM_BASE + 2 * DAYAN_URAM_STRIDE,
          UINT64_C(0x7000000000000000) },
        { DAYAN_URAM_BASE + 3 * DAYAN_URAM_STRIDE,
          UINT64_C(0x8000000000000000) },
        { DAYAN_URAM_BASE + 3 * DAYAN_URAM_STRIDE + DAYAN_URAM_SIZE - 8,
          UINT64_C(0x99999999aaaaaaaa) },
    };

    for (unsigned i = 0; i < ARRAY_SIZE(accesses); i++) {
        qtest_writeq(qts, accesses[i].address, accesses[i].value);
        g_assert_cmphex(qtest_readq(qts, accesses[i].address), ==,
                        accesses[i].value);
    }
}

static void test_dayan_v0_cluster(void)
{
    g_autofree char *args = NULL;
    g_autofree char *mtree = NULL;
    QTestState *qts;

    if (!dayan_v0_test_prepare()) {
        return;
    }

    args = g_strdup_printf("-machine dayan-v0 -smp 4 -bios %s",
                           firmware_path);
    qts = dayan_v0_qtest_init(args);

    mtree = qtest_hmp(qts, "info mtree");
    g_assert_nonnull(strstr(mtree, "dayan.v0.mailbox"));
    g_assert_nonnull(strstr(mtree, "dayan.v0.interrupt-handler"));
    g_assert_nonnull(strstr(mtree, "dayan.v0.atomic-cache-control"));
    g_assert_nonnull(strstr(mtree, "dayan.v0.rom"));
    g_assert_nonnull(strstr(mtree, "dayan.v0.clint.ipi"));
    g_assert_nonnull(strstr(mtree, "dayan.v0.atomic-cache"));
    g_assert_nonnull(strstr(mtree, "dayan.v0.uram_3"));

    test_interrupt_handler(qts);
    test_atomic_cache_control(qts);
    test_clint(qts);
    test_memory_windows(qts);

    qtest_system_reset(qts);
    g_assert_cmphex(qtest_readl(qts, DAYAN_INTC_BASE +
                                DAYAN_INTC_STATUS_OFF), ==, 0);
    g_assert_cmphex(qtest_readl(qts, DAYAN_INTC_BASE +
                                DAYAN_INTC_ENABLE_OFF), ==, 0);
    g_assert_cmphex(qtest_readq(qts, DAYAN_INTC_BASE +
                                DAYAN_INTC_MAILBOX0_OFF), ==, 1);
    g_assert_cmphex(qtest_readq(
                        qts, DAYAN_ATOMIC_CACHE_CONTROL_BASE +
                                 DAYAN_ATOMIC_CACHE_STATUS_OFF),
                    ==, 0);
    g_assert_cmphex(qtest_readq(
                        qts, DAYAN_ATOMIC_CACHE_CONTROL_BASE +
                                 DAYAN_ATOMIC_CACHE_BACKING_OFF),
                    ==, 0);
    g_assert_cmphex(qtest_readl(qts, DAYAN_CLINT_BASE), ==, 0);
    g_assert_cmphex(qtest_readl(qts, dayan_clint_ipi_reg(
                                0, DAYAN_CLINT_IPI_ENABLE)), ==, 1);
    g_assert_cmphex(qtest_readq(qts, DAYAN_CLINT_BASE +
                                DAYAN_CLINT_MTIMECMP), ==, UINT64_MAX);

    qtest_quit(qts);
    dayan_v0_test_cleanup();
}

int main(int argc, char **argv)
{
    int ret;

    g_test_init(&argc, &argv, NULL);
    qtest_add_abrt_handler(dayan_v0_test_abrt, NULL);

    if (qtest_has_machine("dayan-v0")) {
        qtest_add_func("/dayan-v0/cluster", test_dayan_v0_cluster);
    }

    ret = g_test_run();
    dayan_v0_test_cleanup();
    return ret;
}
