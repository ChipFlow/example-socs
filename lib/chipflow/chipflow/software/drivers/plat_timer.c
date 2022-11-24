/* SPDX-License-Identifier: BSD-2-Clause */
#include "plat_timer.h"

uint64_t plat_timer_read(volatile plat_timer_regs_t *timer) {
	uint32_t low_val;
	uint32_t high_val;
	do {
		high_val = timer->high;
		low_val = timer->low;
	} while (high_val != timer->high); // wrapped around
	return (((uint64_t)high_val) << 32ULL) | low_val;
}

void plat_timer_schedule(volatile plat_timer_regs_t *timer, uint64_t val) {
	timer->high = (val >> 32U) & 0xFFFFFFFFU;
	timer->low = val & 0xFFFFFFFFU;
}
