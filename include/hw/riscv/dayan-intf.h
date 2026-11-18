/***************************************************************************************
 * Copyright (c) 2025 Institute of Computing Technology, Chinese Academy of
 *Sciences
 *
 * EMU is licensed under Mulan PSL v2.
 * You can use this software according to the terms and conditions of the Mulan
 *PSL v2. You may obtain a copy of Mulan PSL v2 at:
 *          http://license.coscl.org.cn/MulanPSL2
 *
 * THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY
 *KIND, EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO
 *NON-INFRINGEMENT, MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE.
 *
 * See the Mulan PSL v2 for more details.
 ***************************************************************************************/
#ifndef DAYAN_INTF_H
#define DAYAN_INTF_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include "ipc_channel.h"
#include "blocking-queue.h"

typedef struct {
	uint64_t inst;
    uint64_t rd;
    uint64_t rs1;
    uint64_t rs2;
    uint64_t timestamp;
} dayan_pack_t;

typedef struct {
	uint64_t rd;
	uint64_t data;
    uint64_t timestamp;
} dayan_commit_t;

typedef struct {
    int fdReq;
    int fdRsp;
} dayan_inst_socket_pair_t;

typedef struct {
    int fd;
} dayan_msg_socket_t;

#define DAYAN_MSG_SOCKET_PATH "/tmp/card0_dayan_msg"
#define DAYAN_INST_REQ_SOCKET_PATH "/tmp/card0_dayan_inst_req"
#define DAYAN_INST_RSP_SOCKET_PATH "/tmp/card0_dayan_inst_rsp"
/*
 * Push-only MMU config-sync channel (QEMU -> Dayan-emu): carries accepted
 * ClusterMMU register writes (opcode 0x20) and flush triggers (0x21).
 * It exists so configuration mirroring never interleaves with the
 * position-framed msg socket, where Dayan-emu reads 0x10/0x11 replies
 * inline and a spurious opcode word would desynchronize the stream.
 */
#define DAYAN_MMU_SOCKET_PATH "/tmp/card0_dayan_mmu"

DEFINE_BLOCKING_QUEUE(dayan_pack_t, DayanPackQ)
DEFINE_BLOCKING_QUEUE(dayan_commit_t, DayanCommitQ)

// Maximum number of NPU cores supported
// This should match the maximum value supported by Dayan-emu (default: 1, max: 8)
#define DAYAN_MAX_CALC_CORES 8

// Frontend Service API
int frontend_dram_read_service(char *buf, size_t size, off_t addr);
int frontend_dram_write_service(char *buf, size_t size, off_t addr);
int frontend_instruction_fetch_service(int coreIdx, dayan_pack_t* pkg);
int frontend_instruction_commit_service(int coreIdx, dayan_commit_t* pkg);
int frontend_interrupt_update_service(int coreIdx, int level);

static inline void dayan_socket_fatal(const char *operation, const char *stage)
{
    int error = errno;

    fprintf(stderr,
            "Dayan IPC fatal: %s failed at %s: errno=%d (%s)\n",
            operation, stage, error, strerror(error));
    fflush(stderr);
    exit(EXIT_FAILURE);
}

static inline void dayan_socket_push_or_fatal(int sockfd, uint64_t data,
                                            const char *stage)
{
    if (dayan_socket_push(sockfd, data) != 0) {
        dayan_socket_fatal("send", stage);
    }
}

static inline void dayan_socket_pop_or_fatal(int sockfd, uint64_t *data,
                                           const char *stage)
{
    if (dayan_socket_pop(sockfd, data) != 0) {
        dayan_socket_fatal("receive", stage);
    }
}

static inline void backend_init(int* fdMsg, int* fdInstReq, int* fdInstRsp) {
    int retry = 0;
    while((*fdMsg = dayan_socket_client_init(DAYAN_MSG_SOCKET_PATH)) == -1) {
        if(retry >= 20) {
            exit(-1);
        }
        printf("Backend Init msg client Fail, retrying %d...\n", ++retry);
        usleep(500000);
    }
    while((*fdInstReq = dayan_socket_client_init(DAYAN_INST_REQ_SOCKET_PATH)) == -1) {
        if(retry >= 20) {
            exit(-1);
        }
        printf("Backend Init inst req client Fail, retrying %d...\n", ++retry);
        usleep(500000);
    }
    while((*fdInstRsp = dayan_socket_server_init(DAYAN_INST_RSP_SOCKET_PATH)) == -1) {
        if(retry >= 20) {
            exit(-1);
        }
        printf("Backend Init inst rsp server Fail, retrying %d...\n", ++retry);
        usleep(500000);
    }
}

static inline void frontend_init(int* fdMsg, int* fdInstReq, int* fdInstRsp) {
    int retry = 0;
    while((*fdMsg = dayan_socket_server_init(DAYAN_MSG_SOCKET_PATH)) == -1) {
        if(retry >= 20) {
            exit(-1);
        }
        printf("Frontend Init msg server Fail, retrying %d...\n", ++retry);
        usleep(500000);
    }
    while((*fdInstReq = dayan_socket_server_init(DAYAN_INST_REQ_SOCKET_PATH)) == -1) {
        if(retry >= 20) {
            exit(-1);
        }
        printf("Frontend Init inst req server Fail, retrying %d...\n", ++retry);
        usleep(500000);
    }
    while((*fdInstRsp = dayan_socket_client_init(DAYAN_INST_RSP_SOCKET_PATH)) == -1) {
        if(retry >= 20) {
            exit(-1);
        }
        printf("Frontend Init inst rsp client Fail, retrying %d...\n", ++retry);
        usleep(500000);
    }
}

static inline void* frontend_serve_inst_loop(void* arg) {
    dayan_inst_socket_pair_t* sockets = (dayan_inst_socket_pair_t*)arg;
    int fdInstReq = sockets->fdReq;
    int fdInstRsp = sockets->fdRsp;
    uint64_t msg;
    while(1) {
        // Waiting status;
        dayan_socket_pop_or_fatal(fdInstReq, &msg,
                                "frontend instruction request core index");
        dayan_pack_t instPkg = {0};
        int n = frontend_instruction_fetch_service(msg, &instPkg);
        dayan_socket_push_or_fatal(fdInstRsp, n,
                                 "frontend instruction response valid flag");
        if(n != 0) {
            dayan_socket_push_or_fatal(fdInstRsp, instPkg.inst,
                                     "frontend instruction response inst");
            dayan_socket_push_or_fatal(fdInstRsp, instPkg.rd,
                                     "frontend instruction response rd");
            dayan_socket_push_or_fatal(fdInstRsp, instPkg.rs1,
                                     "frontend instruction response rs1");
            dayan_socket_push_or_fatal(fdInstRsp, instPkg.rs2,
                                     "frontend instruction response rs2");
            dayan_socket_push_or_fatal(
                fdInstRsp, instPkg.timestamp,
                "frontend instruction response timestamp");
        }
    }
    return NULL;
}

// backend端：创建共享内存区域
static inline void* backend_get_shm_ptr(const char* shm_name, size_t sz) {
    // 创建共享内存对象
    int shm_fd = shm_open(shm_name, O_CREAT | O_RDWR, 0666);
    if (shm_fd == -1) {
        perror("shm_open failed");
        return NULL;
    }
    
    // 设置共享内存大小
    if (ftruncate(shm_fd, sz) == -1) {
        perror("ftruncate failed");
        close(shm_fd);
        return NULL;
    }
    
    // 内存映射
    void* ptr = mmap(NULL, sz, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (ptr == MAP_FAILED) {
        perror("mmap failed");
        close(shm_fd);
        return NULL;
    }
    
    close(shm_fd); // 文件描述符可以关闭，映射仍然有效
    
    printf("Backend: Created shared memory '%s' with size %zu at %p\n", 
           shm_name, sz, ptr);
    return ptr;
}

static inline int dayan_file_shm_enabled(void) {
    const char *dir = getenv("DAYAN_EMU_SHM_DIR");
    return dir && dir[0];
}

static inline int dayan_file_shm_name_valid(const char *name) {
    const char *p;
    if (strcmp(name, "/dram_shm_0") == 0) {
        return 1;
    }
    if (strncmp(name, "/uram_shm_", 10) != 0 || !name[10]) {
        return 0;
    }
    for (p = name + 10; *p; ++p) {
        if (*p < '0' || *p > '9') {
            return 0;
        }
    }
    return 1;
}

// frontend端：打开已创建的共享内存
static inline void* frontend_get_shm_ptr_sized(const char* shm_name,
                                               size_t minimum_size) {
    const int file_mode = dayan_file_shm_enabled();
    int shm_fd;
    struct stat sb;

    if (file_mode) {
        int dir_fd;
        if (!dayan_file_shm_name_valid(shm_name)) {
            errno = EINVAL;
            perror("invalid shared memory name");
            return NULL;
        }
        dir_fd = open(getenv("DAYAN_EMU_SHM_DIR"), O_RDONLY | O_DIRECTORY);
        if (dir_fd == -1) {
            perror("open shared memory directory failed");
            return NULL;
        }
        shm_fd = openat(dir_fd, shm_name + 1, O_RDWR | O_NOFOLLOW);
        int saved_errno = errno;
        close(dir_fd);
        errno = saved_errno;
    } else {
        shm_fd = shm_open(shm_name, O_RDWR, 0666);
    }
    if (shm_fd == -1) {
        perror("open shared memory failed");
        return NULL;
    }
    if (fstat(shm_fd, &sb) == -1) {
        perror("fstat failed");
        close(shm_fd);
        return NULL;
    }
    if (file_mode && (!S_ISREG(sb.st_mode) || sb.st_size < 0 ||
                      (uint64_t)sb.st_size < minimum_size ||
                      (uint64_t)sb.st_size > SIZE_MAX)) {
        fprintf(stderr, "Invalid or undersized shared memory file '%s'\n",
                shm_name);
        close(shm_fd);
        return NULL;
    }
    size_t sz = sb.st_size;
    void* ptr = mmap(NULL, sz, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (ptr == MAP_FAILED) {
        perror("mmap failed");
        close(shm_fd);
        return NULL;
    }
    close(shm_fd);
    printf("Frontend: Opened shared memory '%s' with size %zu at %p\n",
           shm_name, sz, ptr);
    return ptr;
}

static inline void* frontend_get_shm_ptr(const char* shm_name) {
    return frontend_get_shm_ptr_sized(shm_name, 0);
}

// 清理共享内存（可选）
static inline void cleanup_shm_ptr(const char* shm_name, void* ptr, size_t sz) {
    if (ptr && ptr != MAP_FAILED) {
        munmap(ptr, sz);
    }    
    /* File-backed regions are owned and removed by the creator. */
    if (!dayan_file_shm_enabled()) {
        shm_unlink(shm_name);
    }
    printf("Cleaned up shared memory '%s'\n", shm_name);
}
static inline void* frontend_serve_msg_loop(void* arg) {
    dayan_msg_socket_t *socket = (dayan_msg_socket_t *)arg;
    int fdMsg = socket->fd;
    uint64_t msg;
    while(1) {
        // Waiting status;
        dayan_socket_pop_or_fatal(fdMsg, &msg,
                                "frontend message opcode");
        if(msg == 0x0) { // Instruction Commit
            // Get CoreIdx
            uint64_t coreIdx;
            dayan_commit_t commit_info;
            dayan_socket_pop_or_fatal(fdMsg, &coreIdx,
                                    "frontend commit core index");
            dayan_socket_pop_or_fatal(fdMsg, &commit_info.rd,
                                    "frontend commit destination register");
            dayan_socket_pop_or_fatal(fdMsg, &commit_info.data,
                                    "frontend commit data");
            dayan_socket_pop_or_fatal(fdMsg, &commit_info.timestamp,
                                    "frontend commit timestamp");
            frontend_instruction_commit_service(coreIdx, &commit_info);
        } else if(msg == 0x1) { // Interrupt Level Update
            uint64_t coreIdx, level;
            dayan_socket_pop_or_fatal(fdMsg, &coreIdx,
                                    "frontend interrupt core index");
            dayan_socket_pop_or_fatal(fdMsg, &level,
                                    "frontend interrupt level");
            frontend_interrupt_update_service(coreIdx, level != 0);
        } else if(msg == 0x10) { // Memory Read
            // Get Addr and Size
            uint64_t addr, size, buf;
            dayan_socket_pop_or_fatal(fdMsg, &addr,
                                    "frontend memory read address");
            dayan_socket_pop_or_fatal(fdMsg, &size,
                                    "frontend memory read size");
            // Read Loop
            while(size) {
                uint64_t rSz = size < 8 ? size : 8;
                frontend_dram_read_service((char*)&buf, rSz, addr);
                dayan_socket_push_or_fatal(fdMsg, buf,
                                         "frontend memory read data");
                addr += rSz;
                size -= rSz;
            }
        } else if(msg == 0x11) { // Memory Write
            // Get Addr and Size
            uint64_t addr, size, buf;
            dayan_socket_pop_or_fatal(fdMsg, &addr,
                                    "frontend memory write address");
            dayan_socket_pop_or_fatal(fdMsg, &size,
                                    "frontend memory write size");
            // Write Loop
            while(size) {
                uint64_t rSz = size < 8 ? size : 8;
                dayan_socket_pop_or_fatal(fdMsg, &buf,
                                        "frontend memory write data");
                frontend_dram_write_service((char*)&buf, rSz, addr);
                addr += rSz;
                size -= rSz;
            }
        }
    }
    return NULL;
}

// Backend API
static inline int backend_dram_read(int fdMsg, char *buf, size_t size, size_t addr) {
    uint64_t t_buf;
    dayan_socket_push_or_fatal(fdMsg, 0x10ull,
                             "backend memory read opcode");
    dayan_socket_push_or_fatal(fdMsg, (uint64_t)addr,
                             "backend memory read address");
    dayan_socket_push_or_fatal(fdMsg, (uint64_t)size,
                             "backend memory read size");
    while(size) {
        uint64_t rSz = size < 8 ? size : 8;
        dayan_socket_pop_or_fatal(fdMsg, &t_buf,
                                "backend memory read data");
        memcpy(buf, &t_buf, rSz);
        buf += rSz;
        size -= rSz;
    }
    return 0;
}
static inline int backend_dram_write(int fdMsg, char *buf, size_t size, off_t addr) {
    uint64_t t_buf;
    dayan_socket_push_or_fatal(fdMsg, 0x11ull,
                             "backend memory write opcode");
    dayan_socket_push_or_fatal(fdMsg, (uint64_t)addr,
                             "backend memory write address");
    dayan_socket_push_or_fatal(fdMsg, (uint64_t)size,
                             "backend memory write size");
    while(size) {
        uint64_t rSz = size < 8 ? size : 8;
        memcpy(&t_buf, buf, rSz);
        dayan_socket_push_or_fatal(fdMsg, t_buf,
                                 "backend memory write data");
        buf += rSz;
        size -= rSz;
    }
    return 0;
}
static inline int backend_instruction_fetch(int fdInstReq, int fdInstRsp, int coreIdx, dayan_pack_t* pkg) {
    uint64_t msg;
    dayan_socket_push_or_fatal(fdInstReq, coreIdx,
                             "backend instruction request core index");
    dayan_socket_pop_or_fatal(fdInstRsp, &msg,
                            "backend instruction response valid flag");
    if(msg == 0) {
        // No Instruction Valid.
        return 0;
    }
    dayan_socket_pop_or_fatal(fdInstRsp, &(pkg->inst),
                            "backend instruction response inst");
    dayan_socket_pop_or_fatal(fdInstRsp, &(pkg->rd),
                            "backend instruction response rd");
    dayan_socket_pop_or_fatal(fdInstRsp, &(pkg->rs1),
                            "backend instruction response rs1");
    dayan_socket_pop_or_fatal(fdInstRsp, &(pkg->rs2),
                            "backend instruction response rs2");
    dayan_socket_pop_or_fatal(fdInstRsp, &(pkg->timestamp),
                            "backend instruction response timestamp");
    return 1;
}
static inline int backend_instruction_commit(int fdMsg, int coreIdx, dayan_commit_t* pkg) {
    dayan_socket_push_or_fatal(fdMsg, 0x0,
                             "backend instruction commit opcode");
    dayan_socket_push_or_fatal(fdMsg, coreIdx,
                             "backend instruction commit core index");
    dayan_socket_push_or_fatal(fdMsg, pkg->rd,
                             "backend instruction commit register");
    dayan_socket_push_or_fatal(fdMsg, pkg->data,
                             "backend instruction commit data");
    dayan_socket_push_or_fatal(fdMsg, pkg->timestamp,
                             "backend instruction commit timestamp");
    return 1;
}

static inline int backend_interrupt_update(int fdMsg, int coreIdx, int level) {
    dayan_socket_push_or_fatal(fdMsg, 0x1,
                             "backend interrupt opcode");
    dayan_socket_push_or_fatal(fdMsg, coreIdx,
                             "backend interrupt core index");
    dayan_socket_push_or_fatal(fdMsg, level != 0,
                             "backend interrupt level");
    return 1;
}

#ifdef __cplusplus
}
#endif

#endif /* INTERFACE_H */
