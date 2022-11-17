LINKER_SCR=generated/sections.lds
CC=riscv32-unknown-linux-gnu-gcc
CINC=-I. -I$(COMMON)
CFLAGS=-g -march=rv32ima -mabi=ilp32 -Wl,--build-id=none,-Bstatic,-T,$(LINKER_SCR),--strip-debug -static -ffreestanding -nostdlib $(CINC)
OBJCOPY=riscv32-unknown-linux-gnu-objcopy
# TODO: This dir needs to come from the chipflow python module
CHIPFLOW_SOFTWARE_DIR=../../lib/chipflow/chipflow/software
# TODO: Instead of rewriting common, perhaps copy drivers etc. to folder
COMMON=$(CHIPFLOW_SOFTWARE_DIR)

BIOS_START=generated/start.S
DRV_SOURCES=$(wildcard $(COMMON)/drivers/*.c) $(wildcard $(COMMON)/drivers/*.S)
DRV_HEADERS=$(wildcard $(COMMON)/drivers/*.h)

ifndef RISCVGCC
  ifeq (, $(shell which riscv32-unknown-linux-gnu-gcc))
    ifeq (, $(shell which riscv64-unknown-elf-gcc))
      $(error "Cannot find a riscv toolchain - please install one.")
    else
      RISCVGCC = riscv64-unknown-elf-gcc
      RISCVOBJCOPY = riscv64-unknown-elf-objcopy
    endif
  else
    RISCVGCC = riscv32-unknown-linux-gnu-gcc
    RISCVOBJCOPY = riscv32-unknown-linux-gnu-objcopy
  endif
endif

bios.elf: $(BIOS_START) $(DRV_SOURCES) $(DRV_HEADERS) $(BIOS_HEADERS) $(BIOS_SOURCES) $(LINKER_SCR)
	$(RISCVGCC) $(CFLAGS) -o bios.elf $(BIOS_START) $(DRV_SOURCES) $(BIOS_SOURCES)

bios.bin: bios.elf
	$(RISCVOBJCOPY) -O binary bios.elf bios.bin

clean:
	rm -f bios.bin bios.elf