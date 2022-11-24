/* SPDX-License-Identifier: BSD-2-Clause */
#include <stdint.h>

#include "generated/soc.h"

static const uintptr_t kernel_base = 0x00800000;
static const uintptr_t dtb_base = 0x00f80000;

void main() {
	puts("🐱: nyaa~!\n");

	puts("SoC type: ");
	puthex(SOC_ID->type);
	puts("\n");

	puts("SoC version: ");
	puthex(SOC_ID->version);
	puts("\n");

	puts("Flash ID: ");
	puthex(spiflash_read_id(FLASH_CTRL));
	puts("\n");

	puts("Entering QSPI mode\n");
	spiflash_set_qspi_flag(FLASH_CTRL);
	spiflash_set_quad_mode(FLASH_CTRL);

	puts("Initialised!\n");

	while (1) {};
}
