/***************************************************************************************
 * Copyright (c) 2025 Institute of Computing Technology, Chinese Academy of
 *Sciences
 *
 * NEMU is licensed under Mulan PSL v2.
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
#ifndef RVTNG_INTF_H
#define RVTNG_INTF_H

#include <stdint.h>
#include "blocking-queue.h"

typedef struct {
	uint32_t inst;
    uint64_t rd;
    uint64_t rs1;
    uint64_t rs2;
    uint64_t timestamp;
} rvt_pack_t;

typedef struct {
	uint8_t rd;
	uint64_t data;
    uint64_t timestamp;
} rvt_commit_t;

// Backend Initialize Scaler Simulater API
int frontend_scaler_sim_main(int argc, char **argv);
// 就绪时，返回 1
int frontend_scaler_sim_ready(void);

// Backend Access System DRAM API
int frontend_dram_read(char *buf, size_t size, off_t addr);
int frontend_dram_write(char *buf, size_t size, off_t addr);

// Backend Tensor Instructions API
int tensor_instruction_fetch(int coreIdx, rvt_pack_t* pkg);
int tensor_instruction_try_fetch(int coreIdx, rvt_pack_t* pkg);
int tensor_instruction_commit(int coreIdx, rvt_commit_t* pkg);

DEFINE_BLOCKING_QUEUE(rvt_pack_t, RVTPackQ)
DEFINE_BLOCKING_QUEUE(rvt_commit_t, RVTCommitQ)

#endif /* INTERFACE_H */
