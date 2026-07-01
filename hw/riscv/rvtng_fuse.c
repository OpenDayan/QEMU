#include "qemu/osdep.h"
#include "system/memory.h"
#include "hw/riscv/rvtng-intf.h"
#include "hw/riscv/rvtng_fuse.h"

#define STACK_SIZE (1ul * 1024 * 1024) // FUSE thread stack size
#define FUSE_USE_VERSION 35

#include <fuse.h>
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

/* Emit one event token to the aggregated host event pipe.
 * Core source is decoded by host via xdma0_user INTC STATUS/MAILBOX.
 */
void rvtng_fuse_emit_event(int core_id, uint32_t value)
{
    if (!event_pipe_initialized) {
        fprintf(stderr,
                "QEMU FUSE EVT: skip emit initialized=%d core=%d value=0x%x\n",
                event_pipe_initialized, core_id, value);
        return;
    }
    if (event_pipe_write[AGG_EVENT_ID] < 0) {
        fprintf(stderr, "QEMU FUSE EVT: invalid write pipe agg=%d\n", AGG_EVENT_ID);
        return;
    }
    ssize_t n = write(event_pipe_write[AGG_EVENT_ID], &value, sizeof(value));
    fprintf(stderr,
            "QEMU FUSE EVT: emit core=%d agg=%d value=0x%x write_ret=%zd errno=%d\n",
            core_id, AGG_EVENT_ID, value, n, (n < 0) ? errno : 0);
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
    (void)path;
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

    MemTxResult res = address_space_read(fuse_as, addr,
        MEMTXATTRS_UNSPECIFIED, buf, size);
    if(res) {
        printf("fioc read error: %d.\n", res);
    }
    return size;
}

static int fioc_write(const char *path, const char *buf, size_t size,
    off_t offset, struct fuse_file_info *fi)
{
    (void)path;
    int ft = (int)fi->fh;
    hwaddr addr = (hwaddr)offset;
    if (ft == FIOC_USER) {
        if (offset < 0 || (uint64_t)offset + (uint64_t)size > RVTNG_INTC_SIZE) {
            return -EINVAL;
        }
        addr = RVTNG_INTC_BASE + (hwaddr)offset;
    }

    // printf("QEMU Get fioc_write: 0x%lx, data[0]: 0x%lx\n", offset, *(long*)buf);
    MemTxResult res = address_space_write(fuse_as, addr,
        MEMTXATTRS_UNSPECIFIED, buf, size);
    // printf("QEMU Get fioc_write: 0x%lx, res: %d, data[0]: 0x%lx\n", offset, res, *(long*)buf);
    if(res) {
        printf("fioc write error: %d.\n", res);
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
static char *fuse_args[] = {
    (char*)"./nemu-fuse",
    (char*)"-s",
    (char*)"-f",
    (char*)"/tmp/virtual_npu/",
    NULL};

/* FUSE service thread */
static void* child_func(void *arg)
{
    printf("QEMU: Child Thread.\n");
    sigset_t mask;
    sigemptyset(&mask);
    sigaddset(&mask, SIGVTALRM);
    sigaddset(&mask, SIGBUS);
    sigaddset(&mask, SIGTRAP);
    pthread_sigmask(SIG_SETMASK, &mask, NULL);
    errno = 0;
    int ret = fuse_main(4, fuse_args, &fioc_oper, NULL);
    if (ret) {
        fprintf(stderr, "QEMU FUSE: fuse_main returned %d errno=%d (%s)\n",
                ret, errno, strerror(errno));
    }
    return NULL;
}

int fuse_init(void);
int fuse_init(void)
{
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

    pthread_t fuse_id;
    int ret = pthread_create(&fuse_id, NULL, child_func, NULL);
    if (ret == -1)
    {
        perror("pthread_create for FUSE thread fail!");
        exit(EXIT_FAILURE);
    }
    sigset_t mask;
    sigemptyset(&mask);
    sigaddset(&mask, SIGHUP);
    sigaddset(&mask, SIGINT);
    sigaddset(&mask, SIGTERM);
    sigaddset(&mask, SIGPIPE);
    sigaddset(&mask, SIGILL);
    sigaddset(&mask, SIGABRT);
    sigaddset(&mask, SIGFPE);
    sigaddset(&mask, SIGSEGV);
    pthread_sigmask(SIG_SETMASK, &mask, NULL);
    printf("QEMU: Host Thread.\n");
    return 0;
}
