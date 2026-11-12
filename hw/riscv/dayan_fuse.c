#include "qemu/osdep.h"
#include "qemu/main-loop.h"
#include "system/memory.h"
#include "hw/riscv/dayan-intf.h"
#include "hw/riscv/dayan_fuse.h"

#define FUSE_USE_VERSION 35

#include <fuse.h>
#include <fuse_lowlevel.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <errno.h>
#include <poll.h>

#define FIOC_NAME "xdma0_"
#define USER_NAME "user"
#define H2C_NAME "h2c_0"
#define C2H_NAME "c2h_0"
#define EVENTS_NAME_PREFIX "events_"

static pthread_mutex_t event_lock = PTHREAD_MUTEX_INITIALIZER;
static uint32_t event_pending_count;
static struct fuse_pollhandle *event_poll_handle;

static struct fuse_pollhandle *fioc_take_event_poll_handle(void)
{
    struct fuse_pollhandle *ph;

    pthread_mutex_lock(&event_lock);
    ph = event_poll_handle;
    event_poll_handle = NULL;
    pthread_mutex_unlock(&event_lock);
    return ph;
}

static void fioc_destroy_poll_handle(struct fuse_pollhandle *ph)
{
    if (ph != NULL) {
        fuse_pollhandle_destroy(ph);
    }
}

/*
 * Accumulate one event in the aggregated host event counter.
 * The source is decoded by the host via xdma0_user STATUS/MAILBOX.
 */
void dayan_fuse_emit_event(int source_id, uint32_t value)
{
    struct fuse_pollhandle *ph;

    if (source_id < 0 || source_id >= DAYAN_INTC_MAILBOX_COUNT || value == 0) {
        return;
    }

    pthread_mutex_lock(&event_lock);
    if (UINT32_MAX - event_pending_count < value) {
        event_pending_count = UINT32_MAX;
    } else {
        event_pending_count += value;
    }
    ph = event_poll_handle;
    event_poll_handle = NULL;
    pthread_mutex_unlock(&event_lock);

    if (ph != NULL) {
        fuse_notify_poll(ph);
        fuse_pollhandle_destroy(ph);
    }
}

void dayan_fuse_reset_events(void)
{
    pthread_mutex_lock(&event_lock);
    event_pending_count = 0;
    pthread_mutex_unlock(&event_lock);
}

enum
{
    FIOC_NONE,
    FIOC_ROOT,
    FIOC_USER,
    FIOC_H2C,
    FIOC_C2H,
    /* Event files: only FIOC_EVENT_0 (aggregated vector) */
    FIOC_EVENT_0 = 10,
};

static int fioc_file_type(const char *path)
{
    if (strcmp(path, "/") == 0)
        return FIOC_ROOT;
    if (strcmp(path, "/" FIOC_NAME USER_NAME) == 0)
        return FIOC_USER;
    if (strcmp(path, "/" FIOC_NAME H2C_NAME) == 0)
        return FIOC_H2C;
    if (strcmp(path, "/" FIOC_NAME C2H_NAME) == 0)
        return FIOC_C2H;
    if (strcmp(path, "/" FIOC_NAME EVENTS_NAME_PREFIX "0") == 0)
        return FIOC_EVENT_0;
    return FIOC_NONE;
}

static int fioc_getattr(const char *path, struct stat *stbuf,
                        struct fuse_file_info *fi)
{
    (void)fi;
    stbuf->st_uid = getuid();
    stbuf->st_gid = getgid();
    stbuf->st_atime = stbuf->st_mtime = time(NULL);

    int ft = fioc_file_type(path);
    switch (ft)
    {
    case FIOC_ROOT:
        stbuf->st_mode = S_IFDIR | 0755;
        stbuf->st_nlink = 2;
        break;
    case FIOC_USER:
        stbuf->st_mode = S_IFREG | 0666;
        stbuf->st_nlink = 1;
        stbuf->st_size = DAYAN_USER_SIZE;
        stbuf->st_rdev = 10;
        stbuf->st_dev = 10;
        break;
    case FIOC_H2C:
    case FIOC_C2H:
        stbuf->st_mode = S_IFREG | 0666;
        stbuf->st_nlink = 1;
        stbuf->st_size = 4096;
        stbuf->st_rdev = 10;
        stbuf->st_dev = 10;
        break;
    case FIOC_EVENT_0:
        stbuf->st_mode = S_IFREG | 0666;
        stbuf->st_nlink = 1;
        stbuf->st_size = 4;
        stbuf->st_rdev = 10;
        stbuf->st_dev = 10;
        break;
    default:
        return -ENOENT;
    }

    return 0;
}

static int fioc_open(const char *path, struct fuse_file_info *fi)
{
    int ft = fioc_file_type(path);
    if (ft != FIOC_NONE)
    {
        fi->direct_io = true;
        fi->nonseekable = false;
        fi->fh = ft;
        return 0;
    }
    return -ENOENT;
}

extern AddressSpace *fuse_as;

static int fioc_read(const char *path, char *buf, size_t size,
                     off_t offset, struct fuse_file_info *fi)
{
    int ft = (int)fi->fh;
    if (ft == FIOC_EVENT_0) {
        uint32_t count;

        if (size < sizeof(count)) {
            return -EINVAL;
        }

        pthread_mutex_lock(&event_lock);
        count = event_pending_count;
        event_pending_count = 0;
        pthread_mutex_unlock(&event_lock);
        if (count == 0) {
            return -EAGAIN;
        }
        memcpy(buf, &count, sizeof(count));
        return sizeof(count);
    }

    hwaddr addr = (hwaddr)offset;
    if (ft == FIOC_USER) {
        if (offset < 0 || (uint64_t)offset + (uint64_t)size > DAYAN_USER_SIZE) {
            return -EINVAL;
        }
        addr = DAYAN_USER_BASE + (hwaddr)offset;
    }

    /* address_space_read may dispatch to device MMIO (e.g. the Dayan INTC),
     * whose handlers mutate shared IRQ state and raise CPU interrupts. Like any
     * QEMU device access from a non-vCPU thread, this must hold the BQL, or it
     * races with the vCPU's interrupt handling and deadlocks on the BQL. */
    bql_lock();
    MemTxResult res = address_space_read(fuse_as, addr,
        MEMTXATTRS_UNSPECIFIED, buf, size);
    bql_unlock();
    if(res) {
        fprintf(stderr,
                "fioc read error: res=%d path=%s offset=0x%llx addr=0x%llx size=%zu\n",
                res, path, (unsigned long long)offset,
                (unsigned long long)addr, size);
        return -EIO;
    }
    return size;
}

static int fioc_release(const char *path, struct fuse_file_info *fi)
{
    int ft = (int)fi->fh;

    if (ft == FIOC_EVENT_0) {
        fioc_destroy_poll_handle(fioc_take_event_poll_handle());
    }
    return 0;
}

static int fioc_poll(const char *path, struct fuse_file_info *fi,
                     struct fuse_pollhandle *ph, unsigned *reventsp)
{
    struct fuse_pollhandle *old_ph = NULL;
    int ft = (int)fi->fh;

    if (ft != FIOC_EVENT_0) {
        fioc_destroy_poll_handle(ph);
        *reventsp = POLLIN | POLLOUT;
        return 0;
    }

    pthread_mutex_lock(&event_lock);
    if (event_pending_count != 0) {
        old_ph = ph;
        *reventsp = POLLIN;
    } else {
        if (ph != NULL && event_poll_handle != ph) {
            old_ph = event_poll_handle;
            event_poll_handle = ph;
        }
        *reventsp = 0;
    }
    pthread_mutex_unlock(&event_lock);
    fioc_destroy_poll_handle(old_ph);
    return 0;
}

static int fioc_write(const char *path, const char *buf, size_t size,
    off_t offset, struct fuse_file_info *fi)
{
    int ft = (int)fi->fh;
    hwaddr addr = (hwaddr)offset;
    if (ft == FIOC_USER) {
        if (offset < 0 || (uint64_t)offset + (uint64_t)size > DAYAN_USER_SIZE) {
            return -EINVAL;
        }
        addr = DAYAN_USER_BASE + (hwaddr)offset;
    }

    // printf("QEMU Get fioc_write: 0x%lx, data[0]: 0x%lx\n", offset, *(long*)buf);
    /* Hold the BQL: a write to the INTC MMIO window runs dayan_intc_write(),
     * which updates shared interrupt state and raises IRQs to the vCPU. Doing
     * that lock-free from the FUSE thread races the vCPU's cpu_handle_interrupt
     * path and intermittently deadlocks the machine on the BQL. */
    bql_lock();
    MemTxResult res = address_space_write(fuse_as, addr,
        MEMTXATTRS_UNSPECIFIED, buf, size);
    bql_unlock();
    // printf("QEMU Get fioc_write: 0x%lx, res: %d, data[0]: 0x%lx\n", offset, res, *(long*)buf);
    if(res) {
        fprintf(stderr,
                "fioc write error: res=%d path=%s offset=0x%llx addr=0x%llx size=%zu\n",
                res, path, (unsigned long long)offset,
                (unsigned long long)addr, size);
        return -EIO;
    }
	return size;
}

static int fioc_readdir(const char *path, void *buf, fuse_fill_dir_t filler,
			off_t offset, struct fuse_file_info *fi,
			enum fuse_readdir_flags flags)
{
	(void) fi;
	(void) offset;
	(void) flags;

	if (fioc_file_type(path) != FIOC_ROOT)
		return -ENOENT;

	filler(buf, ".", NULL, 0, 0);
	filler(buf, "..", NULL, 0, 0);
	filler(buf, FIOC_NAME USER_NAME, NULL, 0, 0);
	filler(buf, FIOC_NAME C2H_NAME, NULL, 0, 0);
	filler(buf, FIOC_NAME H2C_NAME, NULL, 0, 0);
	filler(buf, FIOC_NAME EVENTS_NAME_PREFIX "0", NULL, 0, 0);

	return 0;
}

static const struct fuse_operations fioc_oper = {
    .getattr = fioc_getattr,
    .readdir = fioc_readdir,
    .open = fioc_open,
    .release = fioc_release,
    .read = fioc_read,
    .write = fioc_write,
    .poll = fioc_poll,
};
static char *fuse_argv[] = {
    (char*)"./nemu-fuse",
    (char*)"-s",
    (char*)"-f",
    (char*)"/tmp/virtual_npu/",
    NULL};

static struct fuse *fioc_fuse;
static char *fioc_mountpoint;

int fuse_prepare_mount(void);
int fuse_prepare_mount(void)
{
    struct fuse_args args = FUSE_ARGS_INIT(4, fuse_argv);
    struct fuse_cmdline_opts opts;
    const char *mountpoint = getenv("NPU_DEVICE_PREFIX");
    int rc;

    if (fioc_fuse != NULL) {
        return 0;
    }

    if (mountpoint != NULL && mountpoint[0] != '\0') {
        fuse_argv[3] = (char *)mountpoint;
    }
    memset(&opts, 0, sizeof(opts));
    rc = fuse_parse_cmdline(&args, &opts);
    if (rc != 0 || opts.mountpoint == NULL) {
        fprintf(stderr, "QEMU: fuse_parse_cmdline failed rc=%d\n", rc);
        fuse_opt_free_args(&args);
        free(opts.mountpoint);
        return -1;
    }

    fioc_fuse = fuse_new(&args, &fioc_oper, sizeof(fioc_oper), NULL);
    fuse_opt_free_args(&args);
    if (fioc_fuse == NULL) {
        fprintf(stderr, "QEMU: fuse_new failed\n");
        free(opts.mountpoint);
        return -1;
    }

    errno = 0;
    rc = fuse_mount(fioc_fuse, opts.mountpoint);
    if (rc != 0) {
        int saved_errno = errno;
        fprintf(stderr, "QEMU: fuse_mount failed rc=%d errno=%d (%s)\n",
                rc, saved_errno, strerror(saved_errno));
        fuse_destroy(fioc_fuse);
        fioc_fuse = NULL;
        free(opts.mountpoint);
        return -1;
    }

    fioc_mountpoint = opts.mountpoint;
    return 0;
}

int fuse_loop_main(void);
int fuse_loop_main(void)
{
    if (fioc_fuse == NULL && fuse_prepare_mount() != 0) {
        exit(EXIT_FAILURE);
    }

    sigset_t mask;
    sigemptyset(&mask);
    sigaddset(&mask, SIGVTALRM);
    sigaddset(&mask, SIGBUS);
    sigaddset(&mask, SIGTRAP);
    pthread_sigmask(SIG_SETMASK, &mask, NULL);
    printf("QEMU: FUSE Loop Main Thread.\n");

    errno = 0;
    int rc = fuse_loop(fioc_fuse);
    if (rc != 0) {
        int saved_errno = errno;
        fprintf(stderr, "QEMU: fuse_loop failed rc=%d errno=%d (%s)\n",
                rc, saved_errno, strerror(saved_errno));
        exit(EXIT_FAILURE);
    }

    fuse_unmount(fioc_fuse);
    fioc_destroy_poll_handle(fioc_take_event_poll_handle());
    fuse_destroy(fioc_fuse);
    fioc_fuse = NULL;
    free(fioc_mountpoint);
    fioc_mountpoint = NULL;
    return rc;
}

int fuse_init(void);
int fuse_init(void)
{
    if (fuse_prepare_mount() != 0) {
        exit(EXIT_FAILURE);
    }
    return fuse_loop_main();
}
