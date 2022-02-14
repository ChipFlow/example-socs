all: bios.bin

sources=start.s main.c $(wildcard drivers/*.c) $(wildcard drivers/*.S)
headers=$(wildcard drivers/*.h)

bios.elf: sections.lds $(sources) $(headers)
	riscv32-unknown-linux-gnu-gcc -g -march=rv32ima -mabi=ilp32 -Wl,--build-id=none,-Bstatic,-T,sections.lds,--strip-debug -static -ffreestanding -nostdlib -o bios.elf $(sources)

bios.bin: bios.elf
	riscv32-unknown-linux-gnu-objcopy -O binary bios.elf bios.bin

