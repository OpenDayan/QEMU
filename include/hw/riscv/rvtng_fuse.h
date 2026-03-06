#ifndef HW_RVTNG_FUSE_H
#define HW_RVTNG_FUSE_H

#include <stdint.h>

/* Shared INTC MMIO map for guest and FUSE xdma0_user view. */
#define RVTNG_INTC_BASE          0x2000UL
#define RVTNG_INTC_SIZE          0x210UL
#define RVTNG_INTC_STATUS_OFF    0x00
#define RVTNG_INTC_ENABLE_OFF    0x04
#define RVTNG_INTC_SET_OFF       0x08
#define RVTNG_INTC_CLEAR_OFF     0x0C
#define RVTNG_INTC_MAILBOX0_OFF  0x10
#define RVTNG_INTC_MAILBOX_COUNT 32

/* Emit one host-visible interrupt event for a given core.
 * This is invoked by the RVTNG INTC MMIO SET write path.
 */
void rvtng_fuse_emit_event(int core_id, uint32_t value);

#endif
