/*
 * dayan-v0 ClusterMMU (scalar side) SysBus device
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Functional model of the ClusterMMU for the scalar (QEMU) ports 0..3.
 * The APB4 register image lives at GPA 0x0000_8000; the vCPU
 * load/store/fetch address path is hooked in target/riscv/cpu_helper.c
 * through dayan_cluster_mmu_xlate().
 *
 * Concurrency model: MTTCG lets several vCPU threads translate
 * concurrently with MMIO register writes, all under one QemuMutex.
 * Everything QEMU-global (tlb_flush) happens outside the lock.
 *
 * Config sync with the Dayan-emu process: accepted translation-semantic
 * register writes are mirrored to a dedicated push-only Unix socket
 * (/tmp/card0_dayan_mmu) as {0x20, addr, value}; flush triggers go out as
 * {0x21, mode|asid<<8}.  The Dayan-emu owns the listening end and applies
 * every message to all of its per-core mmu_t instances.  Writes made
 * while disconnected are kept in a shadow image, and (re)connects start
 * with a full replay plus flush, so the Dayan-emu always converges to the
 * QEMU-side config.  See the qemu-MMU implementation guide section 6.10.
 */

#include "qemu/osdep.h"
#include <poll.h>
#include "qemu/error-report.h"
#include "qemu/thread.h"
#include "qapi/error.h"
#include "hw/qdev-properties.h"
#include "hw/qdev-properties-system.h"
#include "hw/sysbus.h"
#include "system/address-spaces.h"
#include "exec/cputlb.h"
#include "migration/vmstate.h"
#include "hw/riscv/dayan_mmu.h"
#include "hw/riscv/dayan-intf.h"
#include "trace.h"

/*
 * Redirect-shim window (dayan_mmu_redirect_tlb_pa()): one sparse region
 * per redirect class, laid out so that the offset within a class region
 * is the full ClusterMMU VA.  Faulting vCPU accesses are pointed here
 * through a marker-attributed TLB entry; the shim handler recovers the
 * VA from its own address and re-runs the ClusterMMU translation for
 * every access, so no offset-preserving page mapping is ever cached for
 * a redirected page.  The window lives at PA 2^48, far above the 16 GiB
 * DRAM declaration, and is 6 x 512 GiB (one full Sv39 VA space per
 * class).  It is still reachable by ordinary PA translation, so every
 * handler rejects accesses without the marker attribute with
 * MEMTX_DECODE_ERROR (guest-visible as a bus access fault), which makes
 * the window behave exactly like unassigned address space for anything
 * that did not come through a redirected TLB entry.
 */
#define DAYAN_MMU_REDIRECT_SLOTS        6
#define DAYAN_MMU_REDIRECT_VA_SPACE     (1ull << 39)
#define DAYAN_MMU_REDIRECT_WINDOW_BASE  0x0001000000000000ull
#define DAYAN_MMU_REDIRECT_WINDOW_SIZE \
    ((hwaddr)DAYAN_MMU_REDIRECT_SLOTS * DAYAN_MMU_REDIRECT_VA_SPACE)

/* Config-sync socket: the connect thread retries indefinitely so that
 * a Dayan-emu backend which appears late (normal non--S startup) is still
 * synced; running without any Dayan-emu costs one failing connect() per
 * retry interval and never stalls the guest. */
#define DAYAN_MMU_FWD_SOCKET_PATH DAYAN_MMU_SOCKET_PATH
#define DAYAN_MMU_FWD_RETRIES     100
#define DAYAN_MMU_FWD_RETRY_US    100000
#define DAYAN_MMU_FWD_RETRY_IDLE_US 1000000

static const VMStateDescription vmstate_dayan_mmu;

static void dayan_mmu_reset_hold(Object *obj, ResetType type);

/* ------------------------------------------------------------------ */
/* Config forwarding to the Dayan-emu                                     */
/* ------------------------------------------------------------------ */

/*
 * The forwarding state is process-global because it backs one shared
 * config-sync socket; dayan-v0 instantiates exactly one ClusterMMU.
 * fwd_lock serializes socket pushes between the MMIO writers and the
 * connect thread, and orders shadow-image updates against replay, so a
 * replay can neither miss nor overtake a concurrently accepted write.
 * It must stay a leaf lock: never taken while holding the device lock.
 */
static int fwd_fd = -1;
static bool fwd_thread_started;
static bool fwd_warn_connect;
static bool fwd_warn_lost;
static QemuMutex fwd_lock;
static uint32_t fwd_shadow[DAYAN_MMU_APB_SIZE / 4];

static void *dayan_mmu_fwd_thread(void *opaque)
{
    unsigned failures = 0;

    for (;;) {
        int sleep_us = DAYAN_MMU_FWD_RETRY_IDLE_US;

        if (qatomic_read(&fwd_fd) < 0) {
            uint16_t off;
            int fd = dayan_socket_client_init(DAYAN_MMU_FWD_SOCKET_PATH);

            if (fd >= 0) {
                bool ok = true;

                qemu_mutex_lock(&fwd_lock);
                if (qatomic_read(&fwd_fd) < 0) {
                    qatomic_set(&fwd_fd, fd);
                    /*
                     * The peer may have missed every write made before
                     * it started listening: replay the whole shadow
                     * image, then flush so no Dayan-emu translation
                     * cached under the stale config survives.
                     */
                    for (off = 0; ok && off < DAYAN_MMU_APB_SIZE;
                         off += 4) {
                        if (dayan_mmu_core_classify_write(off) !=
                            DAYAN_MMU_FWD_REG_WRITE) {
                            continue;
                        }
                        ok = dayan_socket_push(fd, 0x20) == 0 &&
                             dayan_socket_push(fd, off) == 0 &&
                             dayan_socket_push(fd, fwd_shadow[off >> 2]) == 0;
                        trace_dayan_mmu_fwd(0x20, off, fwd_shadow[off >> 2]);
                    }
                    if (ok) {
                        ok = dayan_socket_push(fd, 0x21) == 0 &&
                             dayan_socket_push(fd, 0) == 0;
                        trace_dayan_mmu_fwd(0x21, 0, 0);
                    }
                    if (!ok) {
                        qatomic_set(&fwd_fd, -1);
                        close(fd);
                        if (!fwd_warn_lost) {
                            fwd_warn_lost = true;
                            warn_report("dayan-mmu: Dayan-emu config-sync "
                                        "peer lost during config replay");
                        }
                    }
                } else {
                    close(fd);
                }
                qemu_mutex_unlock(&fwd_lock);
                failures = 0;
            } else {
                if (++failures == DAYAN_MMU_FWD_RETRIES &&
                    !fwd_warn_connect) {
                    fwd_warn_connect = true;
                    warn_report("dayan-mmu: no Dayan-emu config-sync peer at "
                                "%s yet; register writes are buffered "
                                "until it connects",
                                DAYAN_MMU_FWD_SOCKET_PATH);
                }
                sleep_us = DAYAN_MMU_FWD_RETRY_US;
            }
        }
        g_usleep(sleep_us);
    }
    return NULL;
}

static void dayan_mmu_fwd_ensure_started(void)
{
    QemuThread thread;

    if (fwd_thread_started) {
        return;
    }
    qemu_mutex_init(&fwd_lock);
    fwd_thread_started = true;
    qemu_thread_create(&thread, "dayan-mmu-fwd",
                       dayan_mmu_fwd_thread, NULL,
                       QEMU_THREAD_DETACHED);
}

bool dayan_emu_resource_clear(uint32_t mask)
{
    uint64_t status = 1;
    bool ok = false;
    int fd;

    if (!mask) {
        return true;
    }
    dayan_mmu_fwd_ensure_started();
    qemu_mutex_lock(&fwd_lock);
    fd = qatomic_read(&fwd_fd);
    if (fd >= 0) {
        struct pollfd reply = { .fd = fd, .events = POLLIN };

        ok = dayan_socket_push(fd, 0x22) == 0 &&
             dayan_socket_push(fd, mask) == 0 &&
             poll(&reply, 1, 5000) == 1 &&
             dayan_socket_pop(fd, &status) == 0;
        if (!ok) {
            qatomic_set(&fwd_fd, -1);
            close(fd);
        }
    }
    qemu_mutex_unlock(&fwd_lock);
    return ok && status == 0;
}

static void dayan_mmu_fwd_send(DayanMmuState *s, uint16_t off,
                               uint32_t val, bool flush_trigger)
{
    DayanMmuFwdClass cls;
    int fd;
    bool ok;

    if (!s->forward_config) {
        return;
    }
    cls = flush_trigger ? DAYAN_MMU_FWD_FLUSH
                        : dayan_mmu_core_classify_write(off);
    if (cls == DAYAN_MMU_FWD_NONE) {
        /* Status registers and undefined addresses have no Dayan-emu
         * counterpart; FLUSH_DONE/ERR stay QEMU-authoritative. */
        return;
    }

    dayan_mmu_fwd_ensure_started();
    qemu_mutex_lock(&fwd_lock);
    if (cls == DAYAN_MMU_FWD_REG_WRITE) {
        /*
         * Recorded before the connection check so a replay running in
         * parallel can never miss this write.  The shadow keeps the
         * register's actual value, not the raw write: writes the model
         * accepted but silently ignored (misaligned halves, reserved
         * bits) must not overwrite the effective image, or a replay
         * would diverge from this side.
         */
        fwd_shadow[off >> 2] = dayan_mmu_core_read_reg(&s->core, off);
    }
    fd = qatomic_read(&fwd_fd);
    if (fd < 0) {
        /* Not connected yet: the shadow image keeps the event and the
         * connect thread replays it once the peer shows up.  Flushing
         * is unnecessary here, the replay ends with a full flush. */
        qemu_mutex_unlock(&fwd_lock);
        return;
    }

    if (cls == DAYAN_MMU_FWD_FLUSH) {
        uint32_t mode = (val >> 1) & 1;
        uint32_t asid = (val >> 2) & 0xff;

        ok = dayan_socket_push(fd, 0x21) == 0 &&
             dayan_socket_push(fd, mode | (asid << 8)) == 0;
        trace_dayan_mmu_fwd(0x21, mode | (asid << 8), 0);
    } else {
        ok = dayan_socket_push(fd, 0x20) == 0 &&
             dayan_socket_push(fd, off) == 0 &&
             dayan_socket_push(fd, val) == 0;
        trace_dayan_mmu_fwd(0x20, off, val);
    }
    if (!ok) {
        qatomic_set(&fwd_fd, -1);
        close(fd);
        if (!fwd_warn_lost) {
            fwd_warn_lost = true;
            warn_report("dayan-mmu: Dayan-emu config-sync peer lost; "
                        "register writes are buffered until it "
                        "reconnects");
        }
    }
    qemu_mutex_unlock(&fwd_lock);
}

/*
 * Rebuild the shadow image from the live config and drop the socket so
 * the connect thread replays the new image (ending in a flush) into
 * the Dayan-emu.  Runs from reset and post-load, both under the BQL like
 * every register writer, so the config reads need no extra locking.
 */
static void dayan_mmu_fwd_resync(DayanMmuState *s)
{
    uint16_t off;
    int fd;

    if (!fwd_thread_started) {
        return;
    }
    qemu_mutex_lock(&fwd_lock);
    memset(fwd_shadow, 0, sizeof(fwd_shadow));
    for (off = 0; off < DAYAN_MMU_APB_SIZE; off += 4) {
        if (dayan_mmu_core_classify_write(off) == DAYAN_MMU_FWD_REG_WRITE) {
            fwd_shadow[off >> 2] = dayan_mmu_core_read_reg(&s->core, off);
        }
    }
    fd = qatomic_read(&fwd_fd);
    if (fd >= 0) {
        qatomic_set(&fwd_fd, -1);
        close(fd);
    }
    qemu_mutex_unlock(&fwd_lock);
}

/* ------------------------------------------------------------------ */
/* Translation entry point used by target/riscv                        */
/* ------------------------------------------------------------------ */

static const char *dayan_mmu_acc_name(DayanMmuAccess acc)
{
    switch (acc) {
    case DAYAN_MMU_WRITE:
        return "write";
    case DAYAN_MMU_EXECUTE:
        return "exec";
    default:
        return "read";
    }
}

uint64_t dayan_cluster_mmu_xlate(DeviceState *dev, uint64_t gpa,
                                 DayanMmuAccess acc, unsigned port,
                                 DayanMmuFault *fault_out)
{
    DayanMmuState *s = DAYAN_MMU(dev);
    DayanMmuFault fault = DAYAN_MMU_FAULT_NONE;
    DayanMmuPageSize page_size = DAYAN_MMU_PAGE_4K;
    uint64_t pa;

    qemu_mutex_lock(&s->lock);
    pa = dayan_mmu_core_translate(&s->core, gpa, acc, port,
                                  &fault, &page_size);
    qemu_mutex_unlock(&s->lock);

    if (fault_out) {
        *fault_out = fault;
    }
    if (fault != DAYAN_MMU_FAULT_NONE) {
        trace_dayan_mmu_redirect(gpa, pa, fault);
    }
    trace_dayan_mmu_xlate(gpa, pa, port, dayan_mmu_acc_name(acc), fault);
    return pa;
}

/* ------------------------------------------------------------------ */
/* Redirect shim (exact-address representation for fault redirects)    */
/* ------------------------------------------------------------------ */

/* Same class grouping as redirect_pa() in the core, in core.redir[]
 * order: page_fault/read/write/exec/ptw_error/default. */
static unsigned dayan_mmu_redirect_slot(DayanMmuFault fault)
{
    switch (fault) {
    case DAYAN_MMU_FAULT_PTE_INVALID:
    case DAYAN_MMU_FAULT_ILLEGAL_PTE:
        return 0;
    case DAYAN_MMU_FAULT_READ_PERM:
        return 1;
    case DAYAN_MMU_FAULT_WRITE_PERM:
        return 2;
    case DAYAN_MMU_FAULT_EXEC_PERM:
        return 3;
    case DAYAN_MMU_FAULT_PTW_BUS_ERROR:
    case DAYAN_MMU_FAULT_PTW_TIMEOUT:
        return 4;
    default:
        return 5;
    }
}

/*
 * The PA one redirected access must land on, recomputed per access from
 * the live config by re-running the ClusterMMU translation: with
 * ERROR_SLV_OFF_MODE=0 the hardware substitutes the whole master
 * address, so the faulting access's page offset is dropped rather than
 * re-added.  Re-translating (instead of pinning the redirect at fill
 * time) also keeps stale entries self-healing: if the config was fixed
 * without a flush, the access passes through to the real address.
 */
static uint64_t dayan_mmu_redirect_target(DayanMmuState *s, uint64_t va,
                                          DayanMmuAccess acc, unsigned port)
{
    DayanMmuFault fault = DAYAN_MMU_FAULT_NONE;
    DayanMmuPageSize page_size = DAYAN_MMU_PAGE_4K;
    uint64_t pa;

    qemu_mutex_lock(&s->lock);
    pa = dayan_mmu_core_translate(&s->core, va, acc, port,
                                  &fault, &page_size);
    qemu_mutex_unlock(&s->lock);
    trace_dayan_mmu_redirect(va, pa, fault);
    return pa;
}

static bool dayan_mmu_redirect_pa_in_window(uint64_t pa)
{
    return pa - DAYAN_MMU_REDIRECT_WINDOW_BASE < DAYAN_MMU_REDIRECT_WINDOW_SIZE;
}

/*
 * Attributes carried by a redirected TLB entry: the marker separates
 * entries we installed from ordinary PA translation onto the window
 * (which must behave like unassigned memory); port and access type
 * select the translation the handler re-runs.  The VA itself comes from
 * the shim address, which is laid out as class-region + full VA.
 */
MemTxAttrs dayan_mmu_redirect_tlb_attrs(unsigned port, DayanMmuAccess acc,
                                        uint64_t va)
{
    MemTxAttrs attrs = MEMTXATTRS_UNSPECIFIED;

    attrs._reserved1 = (DAYAN_MMU_REDIRECT_ATTR_MARK << 4) |
                       ((port & 3) << 2) | (acc & 3);
    return attrs;
}

static bool dayan_mmu_redirect_parse(MemTxAttrs attrs, hwaddr addr,
                                     uint64_t *va, DayanMmuAccess *acc,
                                     unsigned *port)
{
    if ((attrs._reserved1 >> 4) != DAYAN_MMU_REDIRECT_ATTR_MARK) {
        return false;
    }
    *acc = attrs._reserved1 & 3;
    *port = (attrs._reserved1 >> 2) & 3;
    *va = addr & (DAYAN_MMU_REDIRECT_VA_SPACE - 1);
    return true;
}

static MemTxResult dayan_mmu_redirect_read(void *opaque, hwaddr addr,
                                           uint64_t *data, unsigned size,
                                           MemTxAttrs attrs)
{
    DayanMmuState *s = DAYAN_MMU(opaque);
    DayanMmuAccess acc;
    unsigned port;
    uint64_t va, pa;

    if (!dayan_mmu_redirect_parse(attrs, addr, &va, &acc, &port)) {
        *data = 0;
        return MEMTX_DECODE_ERROR;
    }
    pa = dayan_mmu_redirect_target(s, va, acc, port);
    if (!dayan_mmu_redirect_pa_in_window(pa)) {
        address_space_read(&address_space_memory, pa, MEMTXATTRS_UNSPECIFIED,
                           data, size);
    } else {
        memset(data, 0, size);
    }
    return MEMTX_OK;
}

static MemTxResult dayan_mmu_redirect_write(void *opaque, hwaddr addr,
                                            uint64_t data, unsigned size,
                                            MemTxAttrs attrs)
{
    DayanMmuState *s = DAYAN_MMU(opaque);
    DayanMmuAccess acc;
    unsigned port;
    uint64_t va, pa;

    if (!dayan_mmu_redirect_parse(attrs, addr, &va, &acc, &port)) {
        return MEMTX_DECODE_ERROR;
    }
    pa = dayan_mmu_redirect_target(s, va, DAYAN_MMU_WRITE, port);
    if (!dayan_mmu_redirect_pa_in_window(pa)) {
        address_space_write(&address_space_memory, pa, MEMTXATTRS_UNSPECIFIED,
                            &data, size);
    }
    return MEMTX_OK;
}

static const MemoryRegionOps dayan_mmu_redirect_ops = {
    .read_with_attrs = dayan_mmu_redirect_read,
    .write_with_attrs = dayan_mmu_redirect_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 8,
        .unaligned = true,
    },
    .impl = {
        .min_access_size = 1,
        .max_access_size = 8,
        .unaligned = true,
    },
};

hwaddr dayan_mmu_redirect_tlb_pa(DeviceState *dev, DayanMmuFault fault,
                                 uint64_t va)
{
    return DAYAN_MMU_REDIRECT_WINDOW_BASE +
           (hwaddr)dayan_mmu_redirect_slot(fault) * DAYAN_MMU_REDIRECT_VA_SPACE +
           (va & (DAYAN_MMU_REDIRECT_VA_SPACE - 1));
}

/* ------------------------------------------------------------------ */
/* QEMU TLB coordination                                               */
/* ------------------------------------------------------------------ */

static void dayan_mmu_flush_qemu_tlbs(void)
{
    /* tlb_flush_all_cpus_synced() is async-safe, so it can be issued
     * from vCPU/MMIO context under MTTCG.  When no CPU is current
     * (qtest or monitor driven access, or a reset with vCPUs stopped)
     * nothing is in flight: each vCPU clears its own TLB on reset. */
    if (current_cpu) {
        tlb_flush_all_cpus_synced(current_cpu);
    }
}

/* ------------------------------------------------------------------ */
/* MMIO (APB4 register image)                                          */
/* ------------------------------------------------------------------ */

static uint64_t dayan_mmu_read(void *opaque, hwaddr offset, unsigned size)
{
    DayanMmuState *s = DAYAN_MMU(opaque);
    uint32_t value;

    if (offset >= DAYAN_MMU_APB_SIZE) {
        return 0;
    }
    qemu_mutex_lock(&s->lock);
    value = dayan_mmu_core_read_reg(&s->core, offset);
    qemu_mutex_unlock(&s->lock);
    return value;
}

static void dayan_mmu_write(void *opaque, hwaddr offset, uint64_t value,
                            unsigned size)
{
    DayanMmuState *s = DAYAN_MMU(opaque);
    bool accepted, flush_trigger;

    if (offset >= DAYAN_MMU_APB_SIZE) {
        return;
    }
    qemu_mutex_lock(&s->lock);
    accepted = dayan_mmu_core_write_reg(&s->core, offset, value);
    flush_trigger = accepted && offset == DAYAN_MMU_REG_TLB_FLUSH &&
                    (value & 1);
    if (flush_trigger) {
        trace_dayan_mmu_flush((value >> 1) & 1, (value >> 2) & 0xff);
    }
    qemu_mutex_unlock(&s->lock);

    if (!accepted) {
        /* Undefined address: ignored locally and not mirrored. */
        return;
    }
    trace_dayan_mmu_reg_write(offset, value);

    /*
     * Hardware keeps TLB contents across configuration writes and
     * requires software to flush explicitly (spec 16.5); the default
     * here follows that, eager-tlb-flush=on is a debug aid.
     */
    if (flush_trigger || s->eager_tlb_flush) {
        dayan_mmu_flush_qemu_tlbs();
    }
    /* Socket writes must not hold the model lock (deadlock-free), and
     * the MMIO path holds the BQL so pushes are serialized. */
    dayan_mmu_fwd_send(s, offset, value, flush_trigger);
}

static const MemoryRegionOps dayan_mmu_ops = {
    .read = dayan_mmu_read,
    .write = dayan_mmu_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
        .unaligned = false,
    },
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

/* ------------------------------------------------------------------ */
/* QOM plumbing                                                        */
/* ------------------------------------------------------------------ */

static void dayan_mmu_realize(DeviceState *dev, Error **errp)
{
    DayanMmuState *s = DAYAN_MMU(dev);
    MemoryRegionSection occupied;

    qemu_mutex_init(&s->lock);
    dayan_mmu_core_init(&s->core);
    if (s->dram_mr) {
        dayan_mmu_core_bind_dram(&s->core,
                                 memory_region_get_ram_ptr(s->dram_mr),
                                 s->dram_base, s->dram_size);
    }
    if (s->forward_config) {
        /*
         * Connect (and replay the config image) as early as possible,
         * not on the first forwarded write: the Dayan-emu closes its
         * tensor instruction path until the initial replay lands, so
         * both sides should already be talking before the guest starts.
         */
        dayan_mmu_fwd_ensure_started();
    }

    memory_region_init_io(&s->iomem, OBJECT(s), &dayan_mmu_ops, s,
                          TYPE_DAYAN_MMU, DAYAN_MMU_APB_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);

    memory_region_init_io(&s->redirect_iomem, OBJECT(s),
                          &dayan_mmu_redirect_ops, s,
                          "dayan.v0.mmu-redirect",
                          DAYAN_MMU_REDIRECT_WINDOW_SIZE);
    occupied = memory_region_find(get_system_memory(),
                                  DAYAN_MMU_REDIRECT_WINDOW_BASE,
                                  DAYAN_MMU_REDIRECT_WINDOW_SIZE);
    if (occupied.mr) {
        memory_region_unref(occupied.mr);
        error_setg(errp, "dayan-mmu: redirect shim window %#llx..%#llx "
                   "collides with an existing memory region",
                   (unsigned long long)DAYAN_MMU_REDIRECT_WINDOW_BASE,
                   (unsigned long long)(DAYAN_MMU_REDIRECT_WINDOW_BASE +
                                        DAYAN_MMU_REDIRECT_WINDOW_SIZE - 1));
        return;
    }
    /* Internal implementation detail of the vCPU TLB (see
     * dayan_mmu_redirect_tlb_pa()); never a guest-programmable aperture,
     * so the device maps it itself. */
    memory_region_add_subregion_overlap(get_system_memory(),
                                        DAYAN_MMU_REDIRECT_WINDOW_BASE,
                                        &s->redirect_iomem, 0);
}

static void dayan_mmu_reset_hold(Object *obj, ResetType type)
{
    DayanMmuState *s = DAYAN_MMU(obj);

    dayan_mmu_core_init(&s->core);
    if (s->dram_mr) {
        dayan_mmu_core_bind_dram(&s->core,
                                 memory_region_get_ram_ptr(s->dram_mr),
                                 s->dram_base, s->dram_size);
    }
    /* vCPUs clear their own TLBs in the same reset sweep.  The Dayan-emu,
     * however, survives a guest reset: resync the socket so its config
     * image and translation caches return to the reset image too. */
    dayan_mmu_fwd_resync(s);
}

static const Property dayan_mmu_properties[] = {
    DEFINE_PROP_UINT64("dram-base", DayanMmuState, dram_base,
                       DAYAN_MMU_DRAM_BASE),
    /* Must match the real shared-memory size, not the (larger) region
     * declaration, or PTW reads beyond the shm SIGBUS. */
    DEFINE_PROP_UINT64("dram-size", DayanMmuState, dram_size,
                       DAYAN_MMU_DEFAULT_DRAM_SIZE),
    DEFINE_PROP_BOOL("eager-tlb-flush", DayanMmuState, eager_tlb_flush,
                     false),
    DEFINE_PROP_BOOL("forward-config", DayanMmuState, forward_config,
                     true),
    DEFINE_PROP_LINK("dram-mr", DayanMmuState, dram_mr,
                     TYPE_MEMORY_REGION, MemoryRegion *),
};

static void dayan_mmu_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = dayan_mmu_realize;
    dc->vmsd = &vmstate_dayan_mmu;
    dc->desc = "dayan-v0 ClusterMMU";
    device_class_set_props(dc, dayan_mmu_properties);
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
    rc->phases.hold = dayan_mmu_reset_hold;
}

static const TypeInfo dayan_mmu_info = {
    .name = TYPE_DAYAN_MMU,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(DayanMmuState),
    .class_init = dayan_mmu_class_init,
};

static void dayan_mmu_register_types(void)
{
    type_register_static(&dayan_mmu_info);
}
type_init(dayan_mmu_register_types)

/* ------------------------------------------------------------------ */
/* Migration                                                           */
/* ------------------------------------------------------------------ */

/*
 * The translation caches are not migrated: post_load performs a full
 * model flush and the (not yet implemented) machine-level migration
 * will flush vCPU TLBs through the CPU reset path.  Config and flush
 * status bits are the architectural state and travel with the stream.
 */
static int dayan_mmu_post_load(void *opaque, int version_id)
{
    DayanMmuState *s = DAYAN_MMU(opaque);

    qemu_mutex_lock(&s->lock);
    dayan_mmu_core_flush(&s->core, false, 0);
    qemu_mutex_unlock(&s->lock);
    /* The Dayan-emu never saw the migrated config; resync replays the
     * restored image into it. */
    dayan_mmu_fwd_resync(s);
    return 0;
}

static const VMStateDescription vmstate_dayan_mmu = {
    .name = TYPE_DAYAN_MMU,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = dayan_mmu_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT64(core.ptw_base, DayanMmuState),
        VMSTATE_UINT64(core.fixmap_base, DayanMmuState),
        VMSTATE_UINT64(core.fixmap_size_pages, DayanMmuState),
        VMSTATE_BOOL(core.n_fixmap_en, DayanMmuState),
        VMSTATE_UINT8(core.global_asid, DayanMmuState),
        VMSTATE_UINT64_ARRAY(core.redir, DayanMmuState, 6),
        VMSTATE_BOOL(core.redir_offset_enable, DayanMmuState),
        VMSTATE_UINT64(core.error_slv_mask, DayanMmuState),
        VMSTATE_BOOL(core.flush_done, DayanMmuState),
        VMSTATE_UINT8(core.flush_error_code, DayanMmuState),
        VMSTATE_END_OF_LIST()
    },
};
