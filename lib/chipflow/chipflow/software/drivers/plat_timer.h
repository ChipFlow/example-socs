/* SPDX-License-Identifier: BSD-2-Clause */
#ifndef PLAT_TIMER_H
#define PLAT_TIMER_H

#include <stdint.h>

typedef struct __attribute__((packed)) {
	uint32_t low;
	uint32_t high;
} plat_timer_regs_t;

uint64_t plat_timer_read(volatile plat_timer_regs_t *timer);
void plat_timer_schedule(volatile plat_timer_regs_t *timer, uint64_t val);

#endif
