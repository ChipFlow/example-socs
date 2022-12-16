/* SPDX-License-Identifier: BSD-2-Clause */
#include <stdint.h>

#include "generated/soc.h"

static const uintptr_t kernel_base = 0x00800000;
static const uintptr_t dtb_base = 0x00f80000;

void main() {
	unsigned last_buttons = 0, next_buttons = 0;

	puts("🐱: nyaa~!\n");
	puts("🐱: rob!~!\n");

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

	while (1) {
		// Listen for button presses
		next_buttons = BTN_GPIO->in;
		if (next_buttons) {
			puts("Buttons pressed: ");
			puthex(next_buttons);
		}
		if ((next_buttons & 1U) && !(last_buttons & 1U))
			puts("button 1 pressed!\n");
		if ((next_buttons & 2U) && !(last_buttons & 2U))
			puts("button 2 pressed!\n");
		last_buttons = next_buttons;
	};
}
