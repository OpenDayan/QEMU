#ifndef HW_DAYAN_FUSE_H
#define HW_DAYAN_FUSE_H

#include <stdint.h>

/* Shared interrupt-handler MMIO map for guest and FUSE xdma0_user view. */
#define DAYAN_INTC_BASE             0x2000UL
#define DAYAN_INTC_SIZE             0x200UL
#define DAYAN_USER_BASE             DAYAN_INTC_BASE
#define DAYAN_USER_SIZE             0x3000UL
#define DAYAN_INTC_STATUS_OFF       0x00
#define DAYAN_INTC_ENABLE_OFF       0x04
#define DAYAN_INTC_SET_OFF          0x08
#define DAYAN_INTC_CLEAR_OFF        0x0c
#define DAYAN_INTC_ENABLE_SET_OFF   0x10
#define DAYAN_INTC_ENABLE_CLEAR_OFF 0x14
#define DAYAN_INTC_MAILBOX0_OFF     0x20

#define DAYAN_ATOMIC_CACHE_CONTROL_BASE 0x4000UL
#define DAYAN_ATOMIC_CACHE_CONTROL_SIZE 0x1000UL
#define DAYAN_ATOMIC_CACHE_STATUS_OFF   0x0
#define DAYAN_ATOMIC_CACHE_BACKING_OFF  0x8
#define DAYAN_ATOMIC_CACHE_STATUS_DONE  (1U << 1)
#define DAYAN_INTC_MAILBOX_COUNT    32

/*
 * Emit one host-visible interrupt event for a given source.
 * This is invoked by the Dayan INTC MMIO SET write path.
 */
void dayan_fuse_emit_event(int source_id, uint32_t value);

/* Discard host event counts that belong to the previous machine state. */
void dayan_fuse_reset_events(void);

#endif
