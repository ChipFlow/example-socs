#include <stdint.h>

#include "generated/soc.h"

void main() {
	uart_puts(UART0, "🐱: nyaa~!\n");

	uart_puts(UART0, "SoC type: ");
	uart_puthex(UART0, SOC_ID->type);
	uart_puts(UART0, "\n");

	uart_puts(UART0, "SoC version: ");
	uart_puthex(UART0, SOC_ID->version);
	uart_puts(UART0, "\n");

	uart_puts(UART0, "Flash ID: ");
	uart_puthex(UART0, spiflash_read_id(FLASH_CTRL));
	uart_puts(UART0, "\n");

	while(1) {};
}
