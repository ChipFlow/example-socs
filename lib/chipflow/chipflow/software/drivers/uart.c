/* SPDX-License-Identifier: BSD-2-Clause */
#include "uart.h"

void uart_putc(volatile uart_regs_t *uart, char c) {
	if (c == '\n')
		uart_putc(uart, '\r');
	while (!uart->tx_ready)
		;
	uart->tx_data = c;
}

void uart_puts(volatile uart_regs_t *uart, const char *s) {
	while (*s != 0)
		uart_putc(uart, *s++);
}


void uart_puthex(volatile uart_regs_t *uart, uint32_t x) {
	for (int i = 7; i >= 0; i--) {
		uint8_t nib = (x >> (4 * i)) & 0xF;
		if (nib <= 9)
			uart_putc(uart, '0' + nib);
		else
			uart_putc(uart, 'A' + (nib - 10));
	}
}
