/* SPDX-License-Identifier: BSD-2-Clause */
#ifndef UART_H
#define UART_H

#include <stdint.h>

typedef struct __attribute__((packed)) {
	uint32_t tx_data;
	uint32_t rx_data;
	uint32_t tx_ready;
	uint32_t rx_avail;
} uart_regs_t;

void uart_putc(volatile uart_regs_t *uart, char c);
void uart_puts(volatile uart_regs_t *uart, const char *s);
void uart_puthex(volatile uart_regs_t *uart, uint32_t x);

#endif
