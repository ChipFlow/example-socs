/* SPDX-License-Identifier: BSD-2-Clause */
#include <stdbool.h>
#include "spiflash.h"

extern uint32_t flashio_worker_begin;
extern uint32_t flashio_worker_end;

void spiflash_io(volatile spiflash_regs_t *flash, uint8_t *data, int len, uint8_t wrencmd) {
	// Flash can't be accessed during IO, so copy to RAM and run that
	volatile uint32_t func[&flashio_worker_end - &flashio_worker_begin];

	uint32_t *src_ptr = &flashio_worker_begin;
	volatile uint32_t *dst_ptr = func;

	while (src_ptr != &flashio_worker_end)
		*(dst_ptr++) = *(src_ptr++);

	__asm__ volatile ("fence.i" : : : "memory");

	((void(*)(volatile spiflash_regs_t *, uint8_t*, uint32_t, uint32_t))func)(flash, data, len, wrencmd);
}

uint32_t spiflash_read_id(volatile spiflash_regs_t *flash) {
	uint8_t buffer[5] = { 0x9F, /* zeros */ };
	spiflash_io(flash, buffer, 5, 0);

	uint32_t id = 0;
	for (int i = 1; i <= 4; i++) {
		id = id << 8U;
		id |= buffer[i];
	}
	return id;
}

bool spiflash_is_winbond(volatile spiflash_regs_t *flash) {
	uint32_t id;
	id = spiflash_read_id(flash);
	if ((id & 0x00ff0000) == WINBOND_ID<<16) return true;
	else return false;
}
	
void spiflash_set_qspi_flag(volatile spiflash_regs_t *flash) {
	uint8_t buffer[8];

	//Check which device we have
	if (spiflash_is_winbond(flash)) {
		// Read Configuration Registers (RDCR1 35h)
		buffer[0] = 0x35;
		buffer[1] = 0x00; // rdata
		spiflash_io(flash, buffer, 2, 0);
		uint8_t sr2 = buffer[1];

		// Write Enable Volatile (50h) + Write Status Register 2 (31h)
		buffer[0] = 0x31;
		buffer[1] = sr2 | 2; // Enable QSPI
		spiflash_io(flash, buffer, 2, 0x50);
	} else {
		// Read Configuration Registers (RDCR1 05h)
		buffer[0] = 0x05;
		buffer[1] = 0x00; // rdata
		spiflash_io(flash, buffer, 2, 0);
		uint8_t sr2 = buffer[1];

		// Write Enable Volatile (06h) + Write Status Register 2 (01h)
		buffer[0] = 0x01;
		buffer[1] = sr2 | 1<<6; // Enable QSPI
		spiflash_io(flash, buffer, 2, 0x06);
	}
}

void spiflash_set_quad_mode(volatile spiflash_regs_t *flash) {
	flash->ctrl = (flash->ctrl & ~0x007f0000) | 0x00240000;
}
