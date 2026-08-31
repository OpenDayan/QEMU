/*
 * RVTNG-V0 ClusterMMU (scalar side) SysBus device
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Functional model of the ClusterMMU for the scalar (QEMU) ports 0..3.
 * The APB4 register image lives at GPA 0x0000_8000; the vCPU
 * load/store/fetch address path is hooked in target/riscv/cpu_helper.c
 * through rvtng_cluster_mmu_xlate().
 *
 * Concurrency model: MTTCG lets several vCPU threads translate
 * concurrently with MMIO register writes, all under one QemuMutex.
 * Everything QEMU-global (tlb_flush) happens outside the lock.
 *
 * Config sync with the CModel process: accepted translation-semantic
 * register writes are mirrored to a dedicated push-only Unix socket
 * (/tmp/card0_rvt_mmu) as {0x20, addr, value}; flush triggers go out as
 * {0x21, mode|asid<<8}.  The CModel owns the listening end and applies
 * every message to all of its per-core mmu_t instances.  Writes made
 * while disconnected are kept in a shadow image, and (re)connects start
 * with a full replay plus flush, so the CModel always converges to the
 * QEMU-side config.  See the qemu-MMU implementation guide section 6.10.
 */

#include "qemu/osdep.h"
#include "qemu/error-report.h"
#include "qemu/thread.h"
#include "qapi/error.h"
#include "hw/qdev-properties.h"
#include "hw/qdev-properties-system.h"
#include "hw/sysbus.h"
#include "system/address-spaces.h"
#include "exec/cputlb.h"
#include "migration/vmstate.h"
#include "hw/riscv/rvtng_mmu.h"
#include "hw/riscv/rvtng-intf.h"
#include "trace.h"

/* SoC aperture constants used by translate-scope=dram-uram (kept local
 * to avoid a include cycle with hw/riscv/rvtng_v0.h). */
#define RVTNG_MMU_URAM_BASE       0x00800000ull
#define RVTNG_MMU_URAM_STRIDE     0x00200000ull
#define RVTNG_MMU_URAM_CORES      8
#define RVTNG_MMU_DRAM_BUS_BASE   0x80000000ull

/*
 * Redirect-shim window (rvtng_mmu_redirect_tlb_pa()): one sparse region
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
#define RVTNG_MMU_REDIRECT_SLOTS        6
#define RVTNG_MMU_REDIRECT_VA_SPACE     (1ull << 39)
#define RVTNG_MMU_REDIRECT_WINDOW_BASE  0x0001000000000000ull
#define RVTNG_MMU_REDIRECT_WINDOW_SIZE \
    ((hwaddr)RVTNG_MMU_REDIRECT_SLOTS * RVTNG_MMU_REDIRECT_VA_SPACE)

/* Config-sync socket: the connect thread retries indefinitely so that
 * a CModel backend which appears late (normal non--S startup) is still
 * synced; running without any CModel costs one failing connect() per
 * retry interval and never stalls the guest. */
#define RVTNG_MMU_FWD_SOCKET_PATH RVT_MMU_SOCKET_PATH
#define RVTNG_MMU_FWD_RETRIES     100
#define RVTNG_MMU_FWD_RETRY_US    100000
#define RVTNG_MMU_FWD_RETRY_IDLE_US 1000000

static const VMStateDescription vmstate_rvtng_mmu;

static void rvtng_mmu_reset_hold(Object *obj, ResetType type);

/* ------------------------------------------------------------------ */
/* Config forwarding to the CModel                                     */
/* ------------------------------------------------------------------ */

/*
 * The forwarding state is process-global because it backs one shared
 * config-sync socket; rvtng-v0 instantiates exactly one ClusterMMU.
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
static uint32_t fwd_shadow[RVTNG_MMU_APB_SIZE / 4];

static void *rvtng_mmu_fwd_thread(void *opaque)
{
    unsigned failures = 0;

    for (;;) {
        int sleep_us = RVTNG_MMU_FWD_RETRY_IDLE_US;

        if (qatomic_read(&fwd_fd) < 0) {
            uint16_t off;
            int fd = rvt_socket_client_init(RVTNG_MMU_FWD_SOCKET_PATH);

            if (fd >= 0) {
                bool ok = true;

                qemu_mutex_lock(&fwd_lock);
                if (qatomic_read(&fwd_fd) < 0) {
                    qatomic_set(&fwd_fd, fd);
                    /*
                     * The peer may have missed every write made before
                     * it started listening: replay the whole shadow
                     * image, then flush so no CModel translation
                     * cached under the stale config survives.
                     */
                    for (off = 0; ok && off < RVTNG_MMU_APB_SIZE;
                         off += 4) {
                        if (rvtng_mmu_core_classify_write(off) !=
                            RVTNG_MMU_FWD_REG_WRITE) {
                            continue;
                        }
                        ok = rvt_socket_push(fd, 0x20) == 0 &&
                             rvt_socket_push(fd, off) == 0 &&
                             rvt_socket_push(fd, fwd_shadow[off >> 2]) == 0;
                        trace_rvtng_mmu_fwd(0x20, off, fwd_shadow[off >> 2]);
                    }
                    if (ok) {
                        ok = rvt_socket_push(fd, 0x21) == 0 &&
                             rvt_socket_push(fd, 0) == 0;
                        trace_rvtng_mmu_fwd(0x21, 0, 0);
                    }
                    if (!ok) {
                        qatomic_set(&fwd_fd, -1);
                        close(fd);
                        if (!fwd_warn_lost) {
                            fwd_warn_lost = true;
                            warn_report("rvtng-mmu: CModel config-sync "
                                        "peer lost during config replay");
                        }
                    }
                } else {
                    close(fd);
                }
                qemu_mutex_unlock(&fwd_lock);
                failures = 0;
            } else {
                if (++failures == RVTNG_MMU_FWD_RETRIES &&
                    !fwd_warn_connect) {
                    fwd_warn_connect = true;
                    warn_report("rvtng-mmu: no CModel config-sync peer at "
                                "%s yet; register writes are buffered "
                                "until it connects",
                                RVTNG_MMU_FWD_SOCKET_PATH);
                }
                sleep_us = RVTNG_MMU_FWD_RETRY_US;
            }
        }
        g_usleep(sleep_us);
    }
    return NULL;
}

static void rvtng_mmu_fwd_ensure_started(void)
{
    QemuThread thread;

    if (fwd_thread_started) {
        return;
    }
    qemu_mutex_init(&fwd_lock);
    fwd_thread_started = true;
    qemu_thread_create(&thread, "rvtng-mmu-fwd",
                       rvtng_mmu_fwd_thread, NULL,
                       QEMU_THREAD_DETACHED);
}

static void rvtng_mmu_fwd_send(RvtngMmuState *s, uint16_t off,
                               uint32_t val, bool flush_trigger)
{
    RvtngMmuFwdClass cls;
    int fd;
    bool ok;

    if (!s->forward_config) {
        return;
    }
    cls = flush_trigger ? RVTNG_MMU_FWD_FLUSH
                        : rvtng_mmu_core_classify_write(off);
    if (cls == RVTNG_MMU_FWD_NONE) {
        /* Status registers and undefined addresses have no CModel
         * counterpart; FLUSH_DONE/ERR stay QEMU-authoritative. */
        return;
    }

    rvtng_mmu_fwd_ensure_started();
    qemu_mutex_lock(&fwd_lock);
    if (cls == RVTNG_MMU_FWD_REG_WRITE) {
        /*
         * Recorded before the connection check so a replay running in
         * parallel can never miss this write.  The shadow keeps the
         * register's actual value, not the raw write: writes the model
         * accepted but silently ignored (misaligned halves, reserved
         * bits) must not overwrite the effective image, or a replay
         * would diverge from this side.
         */
        fwd_shadow[off >> 2] = rvtng_mmu_core_read_reg(&s->core, off);
    }
    fd = qatomic_read(&fwd_fd);
    if (fd < 0) {
        /* Not connected yet: the shadow image keeps the event and the
         * connect thread replays it once the peer shows up.  Flushing
         * is unnecessary here, the replay ends with a full flush. */
        qemu_mutex_unlock(&fwd_lock);
        return;
    }

    if (cls == RVTNG_MMU_FWD_FLUSH) {
        uint32_t mode = (val >> 1) & 1;
        uint32_t asid = (val >> 2) & 0xff;

        ok = rvt_socket_push(fd, 0x21) == 0 &&
             rvt_socket_push(fd, mode | (asid << 8)) == 0;
        trace_rvtng_mmu_fwd(0x21, mode | (asid << 8), 0);
    } else {
        ok = rvt_socket_push(fd, 0x20) == 0 &&
             rvt_socket_push(fd, off) == 0 &&
             rvt_socket_push(fd, val) == 0;
        trace_rvtng_mmu_fwd(0x20, off, val);
    }
    if (!ok) {
        qatomic_set(&fwd_fd, -1);
        close(fd);
        if (!fwd_warn_lost) {
            fwd_warn_lost = true;
            warn_report("rvtng-mmu: CModel config-sync peer lost; "
                        "register writes are buffered until it "
                        "reconnects");
        }
    }
    qemu_mutex_unlock(&fwd_lock);
}

/*
 * Rebuild the shadow image from the live config and drop the socket so
 * the connect thread replays the new image (ending in a flush) into
 * the CModel.  Runs from reset and post-load, both under the BQL like
 * every register writer, so the config reads need no extra locking.
 */
static void rvtng_mmu_fwd_resync(RvtngMmuState *s)
{
    uint16_t off;
    int fd;

    if (!fwd_thread_started) {
        return;
    }
    qemu_mutex_lock(&fwd_lock);
    memset(fwd_shadow, 0, sizeof(fwd_shadow));
    for (off = 0; off < RVTNG_MMU_APB_SIZE; off += 4) {
        if (rvtng_mmu_core_classify_write(off) == RVTNG_MMU_FWD_REG_WRITE) {
            fwd_shadow[off >> 2] = rvtng_mmu_core_read_reg(&s->core, off);
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

static const char *rvtng_mmu_acc_name(RvtngMmuAccess acc)
{
    switch (acc) {
    case RVTNG_MMU_WRITE:
        return "write";
    case RVTNG_MMU_EXECUTE:
        return "exec";
    default:
        return "read";
    }
}

static bool rvtng_mmu_scope_covers(RvtngMmuState *s, uint64_t gpa)
{
    if (s->scope_all) {
        return true;
    }
    /* dram-uram: only the data apertures are translated, low control
     * space stays pass-through (bring-up mode). */
    if (gpa >= RVTNG_MMU_URAM_BASE &&
        gpa < RVTNG_MMU_URAM_BASE +
              RVTNG_MMU_URAM_CORES * RVTNG_MMU_URAM_STRIDE) {
        return true;
    }
    return gpa >= RVTNG_MMU_DRAM_BUS_BASE;
}

uint64_t rvtng_cluster_mmu_xlate(DeviceState *dev, uint64_t gpa,
                                 RvtngMmuAccess acc, unsigned port,
                                 RvtngMmuFault *fault_out)
{
    RvtngMmuState *s = RVTNG_MMU(dev);
    RvtngMmuFault fault = RVTNG_MMU_FAULT_NONE;
    RvtngMmuPageSize page_size = RVTNG_MMU_PAGE_4K;
    uint64_t pa;

    if (!rvtng_mmu_scope_covers(s, gpa)) {
        return gpa;
    }

    qemu_mutex_lock(&s->lock);
    pa = rvtng_mmu_core_translate(&s->core, gpa, acc, port,
                                  &fault, &page_size);
    qemu_mutex_unlock(&s->lock);

    if (fault_out) {
        *fault_out = fault;
    }
    if (fault != RVTNG_MMU_FAULT_NONE) {
        trace_rvtng_mmu_redirect(gpa, pa, fault);
    }
    trace_rvtng_mmu_xlate(gpa, pa, port, rvtng_mmu_acc_name(acc), fault);
    return pa;
}

/* ------------------------------------------------------------------ */
/* Redirect shim (exact-address representation for fault redirects)    */
/* ------------------------------------------------------------------ */

/* Same class grouping as redirect_pa() in the core, in core.redir[]
 * order: page_fault/read/write/exec/ptw_error/default. */
static unsigned rvtng_mmu_redirect_slot(RvtngMmuFault fault)
{
    switch (fault) {
    case RVTNG_MMU_FAULT_PTE_INVALID:
    case RVTNG_MMU_FAULT_ILLEGAL_PTE:
        return 0;
    case RVTNG_MMU_FAULT_READ_PERM:
        return 1;
    case RVTNG_MMU_FAULT_WRITE_PERM:
        return 2;
    case RVTNG_MMU_FAULT_EXEC_PERM:
        return 3;
    case RVTNG_MMU_FAULT_PTW_BUS_ERROR:
    case RVTNG_MMU_FAULT_PTW_TIMEOUT:
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
static uint64_t rvtng_mmu_redirect_target(RvtngMmuState *s, uint64_t va,
                                          RvtngMmuAccess acc, unsigned port)
{
    RvtngMmuFault fault = RVTNG_MMU_FAULT_NONE;
    RvtngMmuPageSize page_size = RVTNG_MMU_PAGE_4K;
    uint64_t pa;

    qemu_mutex_lock(&s->lock);
    pa = rvtng_mmu_core_translate(&s->core, va, acc, port,
                                  &fault, &page_size);
    qemu_mutex_unlock(&s->lock);
    trace_rvtng_mmu_redirect(va, pa, fault);
    return pa;
}

static bool rvtng_mmu_redirect_pa_in_window(uint64_t pa)
{
    return pa - RVTNG_MMU_REDIRECT_WINDOW_BASE < RVTNG_MMU_REDIRECT_WINDOW_SIZE;
}

/*
 * Attributes carried by a redirected TLB entry: the marker separates
 * entries we installed from ordinary PA translation onto the window
 * (which must behave like unassigned memory); port and access type
 * select the translation the handler re-runs.  The VA itself comes from
 * the shim address, which is laid out as class-region + full VA.
 */
MemTxAttrs rvtng_mmu_redirect_tlb_attrs(unsigned port, RvtngMmuAccess acc,
                                        uint64_t va)
{
    MemTxAttrs attrs = MEMTXATTRS_UNSPECIFIED;

    attrs._reserved1 = (RVTNG_MMU_REDIRECT_ATTR_MARK << 4) |
                       ((port & 3) << 2) | (acc & 3);
    return attrs;
}

static bool rvtng_mmu_redirect_parse(MemTxAttrs attrs, hwaddr addr,
                                     uint64_t *va, RvtngMmuAccess *acc,
                                     unsigned *port)
{
    if ((attrs._reserved1 >> 4) != RVTNG_MMU_REDIRECT_ATTR_MARK) {
        return false;
    }
    *acc = attrs._reserved1 & 3;
    *port = (attrs._reserved1 >> 2) & 3;
    *va = addr & (RVTNG_MMU_REDIRECT_VA_SPACE - 1);
    return true;
}

static MemTxResult rvtng_mmu_redirect_read(void *opaque, hwaddr addr,
                                           uint64_t *data, unsigned size,
                                           MemTxAttrs attrs)
{
    RvtngMmuState *s = RVTNG_MMU(opaque);
    RvtngMmuAccess acc;
    unsigned port;
    uint64_t va, pa;

    if (!rvtng_mmu_redirect_parse(attrs, addr, &va, &acc, &port)) {
        *data = 0;
        return MEMTX_DECODE_ERROR;
    }
    pa = rvtng_mmu_redirect_target(s, va, acc, port);
    if (!rvtng_mmu_redirect_pa_in_window(pa)) {
        address_space_read(&address_space_memory, pa, MEMTXATTRS_UNSPECIFIED,
                           data, size);
    } else {
        memset(data, 0, size);
    }
    return MEMTX_OK;
}

static MemTxResult rvtng_mmu_redirect_write(void *opaque, hwaddr addr,
                                            uint64_t data, unsigned size,
                                            MemTxAttrs attrs)
{
    RvtngMmuState *s = RVTNG_MMU(opaque);
    RvtngMmuAccess acc;
    unsigned port;
    uint64_t va, pa;

    if (!rvtng_mmu_redirect_parse(attrs, addr, &va, &acc, &port)) {
        return MEMTX_DECODE_ERROR;
    }
    pa = rvtng_mmu_redirect_target(s, va, RVTNG_MMU_WRITE, port);
    if (!rvtng_mmu_redirect_pa_in_window(pa)) {
        address_space_write(&address_space_memory, pa, MEMTXATTRS_UNSPECIFIED,
                            &data, size);
    }
    return MEMTX_OK;
}

static const MemoryRegionOps rvtng_mmu_redirect_ops = {
    .read_with_attrs = rvtng_mmu_redirect_read,
    .write_with_attrs = rvtng_mmu_redirect_write,
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

hwaddr rvtng_mmu_redirect_tlb_pa(DeviceState *dev, RvtngMmuFault fault,
                                 uint64_t va)
{
    return RVTNG_MMU_REDIRECT_WINDOW_BASE +
           (hwaddr)rvtng_mmu_redirect_slot(fault) * RVTNG_MMU_REDIRECT_VA_SPACE +
           (va & (RVTNG_MMU_REDIRECT_VA_SPACE - 1));
}

/* ------------------------------------------------------------------ */
/* QEMU TLB coordination                                               */
/* ------------------------------------------------------------------ */

static void rvtng_mmu_flush_qemu_tlbs(void)
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

static uint64_t rvtng_mmu_read(void *opaque, hwaddr offset, unsigned size)
{
    RvtngMmuState *s = RVTNG_MMU(opaque);
    uint32_t value;

    if (offset >= RVTNG_MMU_APB_SIZE) {
        return 0;
    }
    qemu_mutex_lock(&s->lock);
    value = rvtng_mmu_core_read_reg(&s->core, offset);
    qemu_mutex_unlock(&s->lock);
    return value;
}

static void rvtng_mmu_write(void *opaque, hwaddr offset, uint64_t value,
                            unsigned size)
{
    RvtngMmuState *s = RVTNG_MMU(opaque);
    bool accepted, flush_trigger;

    if (offset >= RVTNG_MMU_APB_SIZE) {
        return;
    }
    qemu_mutex_lock(&s->lock);
    accepted = rvtng_mmu_core_write_reg(&s->core, offset, value);
    flush_trigger = accepted && offset == RVTNG_MMU_REG_TLB_FLUSH &&
                    (value & 1);
    if (flush_trigger) {
        trace_rvtng_mmu_flush((value >> 1) & 1, (value >> 2) & 0xff);
    }
    qemu_mutex_unlock(&s->lock);

    if (!accepted) {
        /* Undefined address: ignored locally and not mirrored. */
        return;
    }
    trace_rvtng_mmu_reg_write(offset, value);

    /*
     * Hardware keeps TLB contents across configuration writes and
     * requires software to flush explicitly (spec 16.5); the default
     * here follows that, eager-tlb-flush=on is a debug aid.
     */
    if (flush_trigger || s->eager_tlb_flush) {
        rvtng_mmu_flush_qemu_tlbs();
    }
    /* Socket writes must not hold the model lock (deadlock-free), and
     * the MMIO path holds the BQL so pushes are serialized. */
    rvtng_mmu_fwd_send(s, offset, value, flush_trigger);
}

static const MemoryRegionOps rvtng_mmu_ops = {
    .read = rvtng_mmu_read,
    .write = rvtng_mmu_write,
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

static void rvtng_mmu_realize_scope(RvtngMmuState *s);

static void rvtng_mmu_realize(DeviceState *dev, Error **errp)
{
    RvtngMmuState *s = RVTNG_MMU(dev);
    MemoryRegionSection occupied;

    qemu_mutex_init(&s->lock);
    rvtng_mmu_realize_scope(s);
    rvtng_mmu_core_init(&s->core);
    if (s->dram_mr) {
        rvtng_mmu_core_bind_dram(&s->core,
                                 memory_region_get_ram_ptr(s->dram_mr),
                                 s->dram_base, s->dram_size);
    }
    if (s->forward_config) {
        /*
         * Connect (and replay the config image) as early as possible,
         * not on the first forwarded write: the CModel closes its
         * tensor instruction path until the initial replay lands, so
         * both sides should already be talking before the guest starts.
         */
        rvtng_mmu_fwd_ensure_started();
    }

    memory_region_init_io(&s->iomem, OBJECT(s), &rvtng_mmu_ops, s,
                          TYPE_RVTNG_MMU, RVTNG_MMU_APB_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);

    memory_region_init_io(&s->redirect_iomem, OBJECT(s),
                          &rvtng_mmu_redirect_ops, s,
                          "rvtng.v0.mmu-redirect",
                          RVTNG_MMU_REDIRECT_WINDOW_SIZE);
    occupied = memory_region_find(get_system_memory(),
                                  RVTNG_MMU_REDIRECT_WINDOW_BASE,
                                  RVTNG_MMU_REDIRECT_WINDOW_SIZE);
    if (occupied.mr) {
        memory_region_unref(occupied.mr);
        error_setg(errp, "rvtng-mmu: redirect shim window %#llx..%#llx "
                   "collides with an existing memory region",
                   (unsigned long long)RVTNG_MMU_REDIRECT_WINDOW_BASE,
                   (unsigned long long)(RVTNG_MMU_REDIRECT_WINDOW_BASE +
                                        RVTNG_MMU_REDIRECT_WINDOW_SIZE - 1));
        return;
    }
    /* Internal implementation detail of the vCPU TLB (see
     * rvtng_mmu_redirect_tlb_pa()); never a guest-programmable aperture,
     * so the device maps it itself. */
    memory_region_add_subregion_overlap(get_system_memory(),
                                        RVTNG_MMU_REDIRECT_WINDOW_BASE,
                                        &s->redirect_iomem, 0);
}

static void rvtng_mmu_reset_hold(Object *obj, ResetType type)
{
    RvtngMmuState *s = RVTNG_MMU(obj);

    rvtng_mmu_core_init(&s->core);
    if (s->dram_mr) {
        rvtng_mmu_core_bind_dram(&s->core,
                                 memory_region_get_ram_ptr(s->dram_mr),
                                 s->dram_base, s->dram_size);
    }
    /* vCPUs clear their own TLBs in the same reset sweep.  The CModel,
     * however, survives a guest reset: resync the socket so its config
     * image and translation caches return to the reset image too. */
    rvtng_mmu_fwd_resync(s);
}

static const Property rvtng_mmu_properties[] = {
    DEFINE_PROP_UINT64("dram-base", RvtngMmuState, dram_base,
                       RVTNG_MMU_DRAM_BASE),
    /* Must match the real shared-memory size, not the (larger) region
     * declaration, or PTW reads beyond the shm SIGBUS. */
    DEFINE_PROP_UINT64("dram-size", RvtngMmuState, dram_size,
                       RVTNG_MMU_DEFAULT_DRAM_SIZE),
    DEFINE_PROP_BOOL("eager-tlb-flush", RvtngMmuState, eager_tlb_flush,
                     false),
    DEFINE_PROP_BOOL("forward-config", RvtngMmuState, forward_config,
                     true),
    DEFINE_PROP_STRING("translate-scope", RvtngMmuState, translate_scope),
    DEFINE_PROP_LINK("dram-mr", RvtngMmuState, dram_mr,
                     TYPE_MEMORY_REGION, MemoryRegion *),
};

static void rvtng_mmu_realize_scope(RvtngMmuState *s)
{
    /* translate-scope is validated at realize time; see
     * rvtng_mmu_scope_covers() for the runtime meaning. */
    s->scope_all = !s->translate_scope ||
                   strcmp(s->translate_scope, "all") == 0;
    if (!s->scope_all && strcmp(s->translate_scope, "dram-uram") != 0) {
        warn_report("rvtng-mmu: unknown translate-scope '%s', using 'all'",
                    s->translate_scope);
        s->scope_all = true;
    }
}

static void rvtng_mmu_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = rvtng_mmu_realize;
    dc->vmsd = &vmstate_rvtng_mmu;
    dc->desc = "RVTNG-V0 ClusterMMU";
    device_class_set_props(dc, rvtng_mmu_properties);
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
    rc->phases.hold = rvtng_mmu_reset_hold;
}

static const TypeInfo rvtng_mmu_info = {
    .name = TYPE_RVTNG_MMU,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(RvtngMmuState),
    .class_init = rvtng_mmu_class_init,
};

static void rvtng_mmu_register_types(void)
{
    type_register_static(&rvtng_mmu_info);
}
type_init(rvtng_mmu_register_types)

/* ------------------------------------------------------------------ */
/* Migration                                                           */
/* ------------------------------------------------------------------ */

/*
 * The translation caches are not migrated: post_load performs a full
 * model flush and the (not yet implemented) machine-level migration
 * will flush vCPU TLBs through the CPU reset path.  Config and flush
 * status bits are the architectural state and travel with the stream.
 */
static int rvtng_mmu_post_load(void *opaque, int version_id)
{
    RvtngMmuState *s = RVTNG_MMU(opaque);

    qemu_mutex_lock(&s->lock);
    rvtng_mmu_core_flush(&s->core, false, 0);
    qemu_mutex_unlock(&s->lock);
    /* The CModel never saw the migrated config; resync replays the
     * restored image into it. */
    rvtng_mmu_fwd_resync(s);
    return 0;
}

static const VMStateDescription vmstate_rvtng_mmu = {
    .name = TYPE_RVTNG_MMU,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = rvtng_mmu_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT64(core.ptw_base, RvtngMmuState),
        VMSTATE_UINT64(core.fixmap_base, RvtngMmuState),
        VMSTATE_UINT64(core.fixmap_size_pages, RvtngMmuState),
        VMSTATE_BOOL(core.n_fixmap_en, RvtngMmuState),
        VMSTATE_UINT8(core.global_asid, RvtngMmuState),
        VMSTATE_UINT64_ARRAY(core.redir, RvtngMmuState, 6),
        VMSTATE_BOOL(core.redir_offset_enable, RvtngMmuState),
        VMSTATE_UINT64(core.error_slv_mask, RvtngMmuState),
        VMSTATE_BOOL(core.flush_done, RvtngMmuState),
        VMSTATE_UINT8(core.flush_error_code, RvtngMmuState),
        VMSTATE_END_OF_LIST()
    },
};
