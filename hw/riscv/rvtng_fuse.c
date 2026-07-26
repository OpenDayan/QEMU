#include "qemu/osdep.h"
#include "qemu/main-loop.h"
#include "system/memory.h"
#include "hw/riscv/rvtng-intf.h"
#include "hw/riscv/rvtng_fuse.h"

#define STACK_SIZE (1ul * 1024 * 1024) // FUSE thread stack size
#define FUSE_USE_VERSION 35

#include <fuse.h>
#include <fuse_lowlevel.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <errno.h>
#include <fcntl.h>

#define FIOC_NAME "xdma0_"
#define USER_NAME "user"
#define H2C_NAME "h2c_0"
#define C2H_NAME "c2h_0"
#define EVENTS_NAME_PREFIX "events_"
#define AGG_EVENT_ID 0

#define NPU_EVENT_PIPE_MAX 32
static int event_pipe_read[NPU_EVENT_PIPE_MAX];
static int event_pipe_write[NPU_EVENT_PIPE_MAX];
static int event_pipe_initialized;

/*
 * Emit one event token to the aggregated host event pipe.
 * The source is decoded by the host via xdma0_user STATUS/MAILBOX.
 */
void rvtng_fuse_emit_event(int source_id, uint32_t value)
{
    if (!event_pipe_initialized) {
        fprintf(stderr,
                "QEMU FUSE EVT: skip emit initialized=%d source=%d "
                "value=0x%x\n",
                event_pipe_initialized, source_id, value);
        return;
    }
    if (event_pipe_write[AGG_EVENT_ID] < 0) {
        fprintf(stderr, "QEMU FUSE EVT: invalid write pipe agg=%d\n", AGG_EVENT_ID);
        return;
    }
    ssize_t n = write(event_pipe_write[AGG_EVENT_ID], &value, sizeof(value));
    fprintf(stderr,
            "QEMU FUSE EVT: emit source=%d agg=%d value=0x%x "
            "write_ret=%zd errno=%d\n",
            source_id, AGG_EVENT_ID, value, n, (n < 0) ? errno : 0);
    (void)n;
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
        stbuf->st_size = RVTNG_INTC_SIZE;
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
    default:
        if (ft >= FIOC_EVENT_0 && ft < FIOC_EVENT_0 + NPU_EVENT_PIPE_MAX) {
            stbuf->st_mode = S_IFREG | 0666;
            stbuf->st_nlink = 1;
            stbuf->st_size = 4;
            stbuf->st_rdev = 10;
            stbuf->st_dev = 10;
        } else {
            return -ENOENT;
        }
        break;
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
    if (ft >= FIOC_EVENT_0 && ft < FIOC_EVENT_0 + NPU_EVENT_PIPE_MAX) {
        int core_id = AGG_EVENT_ID;
        if (event_pipe_read[core_id] < 0 || size < 4)
            return 0;
        ssize_t n = read(event_pipe_read[core_id], buf, size >= 4 ? 4 : size);
        if (n > 0) {
            uint32_t val = *(uint32_t *)buf;
            fprintf(stderr,
                    "QEMU FUSE EVT: host read agg=%d value=0x%x ret=%zd\n",
                    core_id, val, n);
            return (int)n;
        }
        if (n < 0 && errno == EAGAIN) {
            fprintf(stderr,
                    "QEMU FUSE EVT: host read agg=%d ret=EAGAIN\n",
                    core_id);
            return -EAGAIN;  /* Non-blocking read: no pending event token. */
        }
        fprintf(stderr,
                "QEMU FUSE EVT: host read agg=%d ret=%zd errno=%d\n",
                core_id, n, (n < 0) ? errno : 0);
        return 0;
    }

    hwaddr addr = (hwaddr)offset;
    if (ft == FIOC_USER) {
        if (offset < 0 || (uint64_t)offset + (uint64_t)size > RVTNG_INTC_SIZE) {
            return -EINVAL;
        }
        addr = RVTNG_INTC_BASE + (hwaddr)offset;
    }

    /* address_space_read may dispatch to device MMIO (e.g. the rvtng INTC),
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

static int fioc_write(const char *path, const char *buf, size_t size,
    off_t offset, struct fuse_file_info *fi)
{
    int ft = (int)fi->fh;
    hwaddr addr = (hwaddr)offset;
    if (ft == FIOC_USER) {
        if (offset < 0 || (uint64_t)offset + (uint64_t)size > RVTNG_INTC_SIZE) {
            return -EINVAL;
        }
        addr = RVTNG_INTC_BASE + (hwaddr)offset;
    }

    // printf("QEMU Get fioc_write: 0x%lx, data[0]: 0x%lx\n", offset, *(long*)buf);
    /* Hold the BQL: a write to the INTC MMIO window runs rvtng_intc_write(),
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
	.getattr	= fioc_getattr,
	.readdir	= fioc_readdir,
	.open		= fioc_open,
	.read		= fioc_read,
	.write		= fioc_write,
};
static char *fuse_argv[] = {
    (char*)"./nemu-fuse",
    (char*)"-s",
    (char*)"-f",
    (char*)"/tmp/virtual_npu/",
    NULL};

static struct fuse *fioc_fuse;
static char *fioc_mountpoint;

static void fuse_event_pipes_init(void)
{
    if (event_pipe_initialized) {
        return;
    }

    for (int i = 0; i < NPU_EVENT_PIPE_MAX; i++) {
        event_pipe_read[i] = -1;
        event_pipe_write[i] = -1;
    }
    {
        int fd[2];
        if (pipe(fd) == 0) {
            event_pipe_read[AGG_EVENT_ID] = fd[0];
            event_pipe_write[AGG_EVENT_ID] = fd[1];
            /* Keep event reads non-blocking so FUSE thread never stalls on empty pipe. */
            (void)fcntl(fd[0], F_SETFL, fcntl(fd[0], F_GETFL) | O_NONBLOCK);
        }
    }
    event_pipe_initialized = 1;
}

int fuse_prepare_mount(void);
int fuse_prepare_mount(void)
{
    struct fuse_args args = FUSE_ARGS_INIT(4, fuse_argv);
    struct fuse_cmdline_opts opts;
    int rc;

    if (fioc_fuse != NULL) {
        return 0;
    }

    fuse_event_pipes_init();

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
