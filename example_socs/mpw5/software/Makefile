all: bios.bin

bios.elf: sections.lds start.s main.c $(wildcard drivers/*.c)
	riscv32-unknown-linux-gnu-gcc -g -march=rv32ima -mabi=ilp32 -Wl,--build-id=none,-Bstatic,-T,sections.lds,--strip-debug -static -ffreestanding -nostdlib -o bios.elf start.s main.c $(wildcard drivers/*.c)

bios.bin: bios.elf
	riscv32-unknown-linux-gnu-objcopy -O binary bios.elf bios.bin

