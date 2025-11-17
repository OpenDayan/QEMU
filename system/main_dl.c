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
extern RVTPackQ_t rvtPackQ;
extern RVTCommitQ_t rvtCommitQ;
const int rvtSimBackendEnable = 1;

static int true_main(int argc, char **argv)
{
    ATTRIBUTE_UNUSED int ret = system("umount /tmp/virtual_npu");
    long instFd, msgFd;
    pthread_t instServe, msgServe;
    qemu_init(argc, argv);
    RVTPackQ_init(&rvtPackQ, 32);
    RVTCommitQ_init(&rvtCommitQ, 32);
    frontend_init((int*)&msgFd, (int*)&instFd);
    pthread_create(&instServe, NULL, frontend_serve_inst_loop, (void*) instFd);
    pthread_create(&msgServe, NULL, frontend_serve_msg_loop, (void*) msgFd);

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
    printf("QEMU Get frontend_dram_read: 0x%lx, res: %d, data[0]: 0x%lx\n", addr, res, *(long*)buf);
    if(res) {
        printf("frontend_dram_read error: %d.\n", res);
    }
    return size;
}

int frontend_dram_write_service(char *buf, size_t size, off_t addr)
{
    MemTxResult res = address_space_write(fuse_as, addr,
        MEMTXATTRS_UNSPECIFIED, buf, size);
    printf("QEMU Get frontend_dram_write: 0x%lx, res: %d, data[0]: 0x%lx\n", addr, res, *(long*)buf);
    if(res) {
        printf("frontend_dram_write error: %d.\n", res);
    }
	return size;
}

// Backend Tensor Instructions API
int frontend_instruction_fetch_service(int coreIdx, rvt_pack_t* pkg) {
    int ret = RVTPackQ_try_take(&rvtPackQ, pkg);
    if(ret) {
        printf("QEMU: inst fetched: %lx.\n", pkg->inst);
    }
    return ret;
}

int frontend_instruction_commit_service(int coreIdx, rvt_commit_t* pkg) {
    RVTCommitQ_put(&rvtCommitQ, *pkg);
    return 1;
}

#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdlib.h>
#include <fcntl.h>

int main(int argc, char **argv) {
    pid_t pid = fork();
    
    if (pid == -1) {
        perror("fork failed");
        return 1;
    }
    
    if (pid > 0) {
        return 0;
    }
    if (setsid() == -1) {
        perror("setsid failed");
        exit(EXIT_FAILURE);
    }
    pid = fork();
    if (pid == -1) {
        perror("second fork failed");
        exit(EXIT_FAILURE);
    }
    if (pid > 0) {
        exit(EXIT_SUCCESS);
    }
    umask(0);
    int log_fd = open(".emu_log", O_WRONLY | O_CREAT | O_TRUNC, 0644);
    if (log_fd == -1) {
        exit(EXIT_FAILURE);
    }
    if (dup2(log_fd, STDOUT_FILENO) == -1) {
        close(log_fd);
        exit(EXIT_FAILURE);
    }
    close(log_fd);
    close(STDIN_FILENO);
    close(STDERR_FILENO);
    return true_main(argc, argv);
}

