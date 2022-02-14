LINKER_SCR=sections.lds
CC=riscv32-unknown-linux-gnu-gcc
CINC=-I. -I$(COMMON)
CFLAGS=-g -march=rv32ima -mabi=ilp32 -Wl,--build-id=none,-Bstatic,-T,$(LINKER_SCR),--strip-debug -static -ffreestanding -nostdlib $(CINC)
OBJCOPY=riscv32-unknown-linux-gnu-objcopy
COMMON=../../common/software/

BIOS_START=start.S
DRV_SOURCES=$(wildcard $(COMMON)/drivers/*.c) $(wildcard $(COMMON)/drivers/*.S)
DRV_HEADERS=$(wildcard $(COMMON)/drivers/*.h)

bios.elf: sections.lds $(BIOS_START) $(DRV_SOURCES) $(DRV_HEADERS) $(BIOS_HEADERS) $(BIOS_SOURCES) $(LINKER_SCR)
	riscv32-unknown-linux-gnu-gcc $(CFLAGS) -o bios.elf $(BIOS_START) $(DRV_SOURCES) $(BIOS_SOURCES)

bios.bin: bios.elf
	riscv32-unknown-linux-gnu-objcopy -O binary bios.elf bios.bin

clean:
	rm -f bios.bin bios.elf
