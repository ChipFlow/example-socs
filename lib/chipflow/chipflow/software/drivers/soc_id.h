/* SPDX-License-Identifier: BSD-2-Clause */
#ifndef SOC_ID_H
#define SOC_ID_H

#include <stdint.h>

typedef struct __attribute__((packed)) {
	uint32_t type;
	uint32_t version;
} soc_id_regs_t;

#endif
