#include "qemu/osdep.h"
#include "system/memory.h"

#define STACK_SIZE (1ul * 1024 * 1024) // FUSE Own 1MB Stack
#define FUSE_USE_VERSION 35

#include <fuse.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <errno.h>

#define FIOC_NAME "xdma0_"
#define H2C_NAME "h2c_0"
#define C2H_NAME "c2h_0"

enum
{
    FIOC_NONE,
    FIOC_ROOT,
    FIOC_H2C,
    FIOC_C2H,
};

static int fioc_file_type(const char *path)
{
    if (strcmp(path, "/") == 0)
        return FIOC_ROOT;
    if (strcmp(path, "/" FIOC_NAME H2C_NAME) == 0)
        return FIOC_H2C;
    if (strcmp(path, "/" FIOC_NAME C2H_NAME) == 0)
        return FIOC_C2H;
    return FIOC_NONE;
}

static int fioc_getattr(const char *path, struct stat *stbuf,
                        struct fuse_file_info *fi)
{
    (void)fi;
    stbuf->st_uid = getuid();
    stbuf->st_gid = getgid();
    stbuf->st_atime = stbuf->st_mtime = time(NULL);

    switch (fioc_file_type(path))
    {
    case FIOC_ROOT:
        stbuf->st_mode = S_IFDIR | 0755;
        stbuf->st_nlink = 2;
        break;
    case FIOC_H2C:
    case FIOC_C2H:
        stbuf->st_mode = S_IFREG | 0666;
        stbuf->st_nlink = 1;
        stbuf->st_size = 4096;
        stbuf->st_rdev = 10;
        stbuf->st_dev = 10;
        break;
    case FIOC_NONE:
        return -ENOENT;
    }

    return 0;
}

static int fioc_open(const char *path, struct fuse_file_info *fi)
{
    if (fioc_file_type(path) != FIOC_NONE)
    {
        fi->direct_io = true;
        fi->nonseekable = false;
        fi->fh = fioc_file_type(path);
        return 0;
    }
    return -ENOENT;
}

extern AddressSpace *fuse_as;

static int fioc_read(const char *path, char *buf, size_t size,
                     off_t offset, struct fuse_file_info *fi)
{
    // printf("QEMU Get fioc_read: 0x%lx, data[0]: 0x%lx\n", offset, *(long*)buf);
    MemTxResult res = address_space_read(fuse_as, offset,
        MEMTXATTRS_UNSPECIFIED, buf, size);
    // printf("QEMU Get fioc_read: 0x%lx, res: %d, data[0]: 0x%lx\n", offset, res, *(long*)buf);
    if(res) {
        printf("fioc read error: %d.\n", res);
    }
    return size;
}

static int fioc_write(const char *path, const char *buf, size_t size,
    off_t offset, struct fuse_file_info *fi)
{
    // printf("QEMU Get fioc_write: 0x%lx, data[0]: 0x%lx\n", offset, *(long*)buf);
    MemTxResult res = address_space_write(fuse_as, offset,
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
	filler(buf, FIOC_NAME C2H_NAME, NULL, 0, 0);
	filler(buf, FIOC_NAME H2C_NAME, NULL, 0, 0);

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

// FUSE 子进程入口函数
static void* child_func(void *arg)
{
    printf("QEMU: Child Thread.\n");
    sigset_t mask;
    sigemptyset(&mask);
    sigaddset(&mask, SIGVTALRM);
    sigaddset(&mask, SIGBUS);
    sigaddset(&mask, SIGTRAP);
    pthread_sigmask(SIG_SETMASK, &mask, NULL);
    fuse_main(4, fuse_args, &fioc_oper, NULL);
    return NULL;
}
int fuse_init(void);
int fuse_init(void)
{
    // 创建 FUSE 线程
    pthread_t fuse_id;
    int ret = pthread_create(&fuse_id, NULL, child_func, NULL); //创建并执行线程id
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
