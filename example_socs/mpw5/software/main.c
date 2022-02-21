#include <stdint.h>

#include "generated/soc.h"
#include "mmode.h"

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


	if (SOC_ID->type == 0xBADCA77E) {
		puts("Detected running in simulation!\n");
	} else {
		puts("Zeroing initial RAM...\n");
		volatile uint32_t *const zero_begin = (volatile uint32_t *)0x10000000;
		volatile uint32_t *const zero_end = (volatile uint32_t *)0x10100000;

		for (volatile uint32_t *ptr = zero_begin; ptr < zero_end; ++ptr)
			*ptr = 0;
	}

	boot_kernel(kernel_base, dtb_base);
}
