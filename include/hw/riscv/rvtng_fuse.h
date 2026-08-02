#ifndef HW_RVTNG_FUSE_H
#define HW_RVTNG_FUSE_H

#include <stdint.h>

/* Shared interrupt-handler MMIO map for guest and FUSE xdma0_user view. */
#define RVTNG_INTC_BASE             0x2000UL
#define RVTNG_INTC_SIZE             0x200UL
#define RVTNG_INTC_STATUS_OFF       0x00
#define RVTNG_INTC_ENABLE_OFF       0x04
#define RVTNG_INTC_SET_OFF          0x08
#define RVTNG_INTC_CLEAR_OFF        0x0c
#define RVTNG_INTC_ENABLE_SET_OFF   0x10
#define RVTNG_INTC_ENABLE_CLEAR_OFF 0x14
#define RVTNG_INTC_MAILBOX0_OFF     0x20
#define RVTNG_INTC_MAILBOX_COUNT    32

/*
 * Emit one host-visible interrupt event for a given source.
 * This is invoked by the RVTNG INTC MMIO SET write path.
 */
void rvtng_fuse_emit_event(int source_id, uint32_t value);

/* Discard host event counts that belong to the previous machine state. */
void rvtng_fuse_reset_events(void);

#endif
