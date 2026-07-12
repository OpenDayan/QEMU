/*
 * QEMU System Emulator
 *
 * Copyright (c) 2003-2020 Fabrice Bellard
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "qemu/osdep.h"
#include "qemu-main.h"
#include "qemu/main-loop.h"
#include "system/replay.h"
#include "system/system.h"
#include "system/memory.h"
#include "hw/riscv/rvtng-intf.h"

#ifdef CONFIG_SDL
/*
 * SDL insists on wrapping the main() function with its own implementation on
 * some platforms; it does so via a macro that renames our main function, so
 * <SDL.h> must be #included here even with no SDL code called from this file.
 */
#include <SDL.h>
#endif

#ifdef CONFIG_DARWIN
#include <CoreFoundation/CoreFoundation.h>
#endif

extern int fuse_loop_main(void);

static void *qemu_default_main(void *opaque)
{
    int status;

    replay_mutex_lock();
    bql_lock();
    status = qemu_main_loop();
    qemu_cleanup(status);
    bql_unlock();
    replay_mutex_unlock();

    exit(status);
}

static void *fuse_loop_thread_main(void *opaque)
{
    (void)opaque;
    fuse_loop_main();
    return NULL;
}

int (*qemu_main)(void);

#ifdef CONFIG_DARWIN
static int os_darwin_cfrunloop_main(void)
{
    CFRunLoopRun();
    g_assert_not_reached();
}
int (*qemu_main)(void) = os_darwin_cfrunloop_main;
#endif

// Global Intf for inter-call
extern RVTPackQ_t rvtPackQ[RVTNG_MAX_CALC_CORES];
extern RVTCommitQ_t rvtCommitQ[RVTNG_MAX_CALC_CORES];
extern int fuse_init(void);
extern int fuse_prepare_mount(void);
const int rvtSimBackendEnable = 1;

static int true_main(int argc, char **argv)
{
    ATTRIBUTE_UNUSED int ret = system("umount /tmp/virtual_npu");
    bool use_fuse_loop = qemu_main == NULL;
    int instReqFd, instRspFd, msgFd;
    rvt_inst_socket_pair_t instSockets;
    rvt_msg_socket_t msgSocket;
    pthread_t instServe, msgServe;
    int thread_error;

    if (use_fuse_loop && fuse_prepare_mount() != 0) {
        return 1;
    }

    qemu_init(argc, argv);
    // Initialize queues for all cores
    for(int i = 0; i < RVTNG_MAX_CALC_CORES; i++) {
        RVTPackQ_init(&rvtPackQ[i], 32);
        RVTCommitQ_init(&rvtCommitQ[i], 32);
    }
    frontend_init(&msgFd, &instReqFd, &instRspFd);
    instSockets.fdReq = instReqFd;
    instSockets.fdRsp = instRspFd;
    msgSocket.fd = msgFd;
    thread_error = pthread_create(&instServe, NULL,
                                  frontend_serve_inst_loop, &instSockets);
    if (thread_error != 0) {
        fprintf(stderr, "QEMU: failed to create instruction IPC thread: %s\n",
                strerror(thread_error));
        return 1;
    }
    thread_error = pthread_create(&msgServe, NULL,
                                  frontend_serve_msg_loop, &msgSocket);
    if (thread_error != 0) {
        fprintf(stderr, "QEMU: failed to create message IPC thread: %s\n",
                strerror(thread_error));
        pthread_cancel(instServe);
        pthread_join(instServe, NULL);
        return 1;
    }
    pthread_detach(instServe);
    pthread_detach(msgServe);

    /*
     * qemu_init acquires the BQL and replay mutex lock. BQL is acquired when
     * initializing cpus, to block associated threads until initialization is
     * complete. Replay_mutex lock is acquired on initialization, because it
     * must be held when configuring icount_mode.
     *
     * On MacOS, qemu main event loop runs in a background thread, as main
     * thread must be reserved for UI. Thus, we need to transfer lock ownership,
     * and the simplest way to do that is to release them, and reacquire them
     * from qemu_default_main.
     */
    bql_unlock();
    replay_mutex_unlock();

    if (qemu_main) {
        QemuThread main_loop_thread;
        qemu_thread_create(&main_loop_thread, "qemu_main",
                           qemu_default_main, NULL, QEMU_THREAD_DETACHED);
        return qemu_main();
    } else {
        pthread_t fuseLoop;
        pthread_create(&fuseLoop, NULL, fuse_loop_thread_main, NULL);
        pthread_detach(fuseLoop);
        qemu_default_main(NULL);
        g_assert_not_reached();
    }
}

// Verilator Interface
extern AddressSpace *fuse_as;
int frontend_dram_read_service(char *buf, size_t size, off_t addr)
{
    MemTxResult res = address_space_read(fuse_as, addr,
        MEMTXATTRS_UNSPECIFIED, buf, size);
    // printf("QEMU Get frontend_dram_read: 0x%lx, res: %d, data[0]: 0x%lx\n", addr, res, *(long*)buf);
    if(res) {
        printf("frontend_dram_read error: %d.\n", res);
    }
    return size;
}

int frontend_dram_write_service(char *buf, size_t size, off_t addr)
{
    MemTxResult res = address_space_write(fuse_as, addr,
        MEMTXATTRS_UNSPECIFIED, buf, size);
    // printf("QEMU Get frontend_dram_write: 0x%lx, res: %d, data[0]: 0x%lx\n", addr, res, *(long*)buf);
    if(res) {
        printf("frontend_dram_write error: %d.\n", res);
    }
	return size;
}

// Backend Tensor Instructions API
int frontend_instruction_fetch_service(int coreIdx, rvt_pack_t* pkg) {
    // Validate coreIdx range
    if(coreIdx < 0 || coreIdx >= RVTNG_MAX_CALC_CORES) {
        return 0;
    }

    // Get instruction from the corresponding core's queue
    int ret = RVTPackQ_try_take(&rvtPackQ[coreIdx], pkg);
    // if(ret) {
        // printf("QEMU: inst fetched: %lx.\n", pkg->inst);
    // }
    return ret;
}

int frontend_instruction_commit_service(int coreIdx, rvt_commit_t* pkg) {
    // Validate coreIdx range
    if(coreIdx < 0 || coreIdx >= RVTNG_MAX_CALC_CORES) {
        return 0;
    }

    // Put result into the corresponding core's queue
    RVTCommitQ_put(&rvtCommitQ[coreIdx], *pkg);
    return 1;
}

#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdlib.h>
#include <fcntl.h>

static int redirect_qemu_log(void)
{
    const char *log_path = getenv("RVTNG_QEMU_LOG");

    if (log_path == NULL || log_path[0] == '\0') {
        return 0;
    }

    int log_fd = open(log_path, O_WRONLY | O_CREAT | O_TRUNC, 0644);
    if (log_fd == -1) {
        fprintf(stderr, "QEMU: cannot open RVTNG_QEMU_LOG '%s': %s\n",
                log_path, strerror(errno));
        return -1;
    }
    if (dup2(log_fd, STDOUT_FILENO) == -1) {
        int saved_errno = errno;
        if (log_fd > STDERR_FILENO) {
            close(log_fd);
        }
        fprintf(stderr, "QEMU: cannot redirect stdout to '%s': %s\n",
                log_path, strerror(saved_errno));
        return -1;
    }
    if (dup2(log_fd, STDERR_FILENO) == -1) {
        int saved_errno = errno;
        if (log_fd > STDERR_FILENO) {
            close(log_fd);
        }
        fprintf(stderr, "QEMU: cannot redirect stderr to '%s': %s\n",
                log_path, strerror(saved_errno));
        return -1;
    }
    if (log_fd > STDERR_FILENO) {
        close(log_fd);
    }

    return 0;
}

int main(int argc, char **argv)
{
    umask(0);
    if (redirect_qemu_log() != 0) {
        return EXIT_FAILURE;
    }

    return true_main(argc, argv);
}
