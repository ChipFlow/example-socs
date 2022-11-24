# SPDX-License-Identifier: BSD-2-Clause
from pathlib import Path


class SoftwareGenerator:
    def __init__(self, *, rom_start, rom_size, ram_start, ram_size):
        self.rom_start = rom_start
        self.rom_size = rom_size
        self.ram_start = ram_start
        self.ram_size = ram_size
        self.defines = []
        self.periphs = []
        self.extra_init = []

    def generate(self, out_dir):
        Path(out_dir).mkdir(parents=True, exist_ok=True)
        with open(Path(out_dir) / "start.S", "w") as f:
            f.write(self.start)
        with open(Path(out_dir) / "sections.lds", "w") as f:
            f.write(self.lds)
        with open(Path(out_dir) / "soc.h", "w") as f:
            f.write(self.soc_h)

    def add_periph(self, periph_type, name, address):
        self.periphs.append((periph_type, name, address))

    def add_extra_init(self, asm):
        self.extra_init.append(asm)

    @property
    def soc_h(self):
        result = "#ifndef SOC_H\n"
        result += "#define SOC_H\n"
        periph_types = sorted(set(x[0] for x in self.periphs))

        for t in periph_types:
            result += f'#include "drivers/{t}.h"\n'
        result += "\n"

        uart = None

        for t, n, a in self.periphs:
            if uart is None and t == "uart":  # first UART
                uart = n
            result += f'#define {n} ((volatile {t}_regs_t *const)0x{a:08x})\n'

        result += '\n'

        if uart is not None:
            result += f'#define putc(x) uart_putc({uart}, x)\n'
            result += f'#define puts(x) uart_puts({uart}, x)\n'
            result += f'#define puthex(x) uart_puthex({uart}, x)\n'
        else:
            result += f'#define putc(x) do {{ (void)x; }} while(0) \n'
            result += f'#define puts(x) do {{ (void)x; }} while(0)\n'
            result += f'#define puthex(x) do {{ (void)x; }} while(0)\n'

        result += "#endif\n"
        return result

    @property
    def start(self):
        joined_init = '\n'.join(self.extra_init)
        return f""".section .text

start:

# zero-initialize register file
addi x1, zero, 0
li x2, 0x{self.ram_start+self.ram_size:08x} # Top of stack
addi x3, zero, 0
addi x4, zero, 0
addi x5, zero, 0
addi x6, zero, 0
addi x7, zero, 0
addi x8, zero, 0
addi x9, zero, 0
addi x10, zero, 0
addi x11, zero, 0
addi x12, zero, 0
addi x13, zero, 0
addi x14, zero, 0
addi x15, zero, 0
addi x16, zero, 0
addi x17, zero, 0
addi x18, zero, 0
addi x19, zero, 0
addi x20, zero, 0
addi x21, zero, 0
addi x22, zero, 0
addi x23, zero, 0
addi x24, zero, 0
addi x25, zero, 0
addi x26, zero, 0
addi x27, zero, 0
addi x28, zero, 0
addi x29, zero, 0
addi x30, zero, 0
addi x31, zero, 0

{joined_init}

# copy data section
la a0, _sidata
la a1, _sdata
la a2, _edata
bge a1, a2, end_init_data
loop_init_data:
lw a3, 0(a0)
sw a3, 0(a1)
addi a0, a0, 4
addi a1, a1, 4
blt a1, a2, loop_init_data
end_init_data:

# zero-init bss section
la a0, _sbss
la a1, _ebss
bge a0, a1, end_init_bss
loop_init_bss:
sw zero, 0(a0)
addi a0, a0, 4
blt a0, a1, loop_init_bss
end_init_bss:

# Update LEDs
li a0, 0xb1000000
li a1, 2
sw a1, 0(a0)

# call main
call main
loop:
j loop
"""

    @property
    def lds(self):
        return f"""MEMORY
{{
    FLASH (rx)      : ORIGIN = 0x{self.rom_start:08x}, LENGTH = 0x{self.rom_size:08x}
    RAM (xrw)       : ORIGIN = 0x{self.ram_start:08x}, LENGTH = 0x{self.ram_size:08x}
}}

SECTIONS {{
    /* The program code and other data goes into FLASH */
    .text :
    {{
        . = ALIGN(4);
        *(.text)           /* .text sections (code) */
        *(.text*)          /* .text* sections (code) */
        *(.rodata)         /* .rodata sections (constants, strings, etc.) */
        *(.rodata*)        /* .rodata* sections (constants, strings, etc.) */
        *(.srodata)        /* .rodata sections (constants, strings, etc.) */
        *(.srodata*)       /* .rodata* sections (constants, strings, etc.) */
        . = ALIGN(4);
        _etext = .;        /* define a global symbol at end of code */
        _sidata = _etext;  /* This is used by the startup in order to initialize the .data secion */
    }} >FLASH


    /* This is the initialized data section
    The program executes knowing that the data is in the RAM
    but the loader puts the initial values in the FLASH (inidata).
    It is one task of the startup to copy the initial values from FLASH to RAM. */
    .data : AT ( _sidata )
    {{
        . = ALIGN(4);
        _sdata = .;        /* create a global symbol at data start; used by startup code in order to initialise the .data section in RAM */
        _ram_start = .;    /* create a global symbol at ram start for garbage collector */
        . = ALIGN(4);
        *(.data)           /* .data sections */
        *(.data*)          /* .data* sections */
        *(.sdata)           /* .sdata sections */
        *(.sdata*)          /* .sdata* sections */
        . = ALIGN(4);
        _edata = .;        /* define a global symbol at data end; used by startup code in order to initialise the .data section in RAM */
    }} >RAM

    /* Uninitialized data section */
    .bss :
    {{
        . = ALIGN(4);
        _sbss = .;         /* define a global symbol at bss start; used by startup code */
        *(.bss)
        *(.bss*)
        *(.sbss)
        *(.sbss*)
        *(COMMON)

        . = ALIGN(4);
        _ebss = .;         /* define a global symbol at bss end; used by startup code */
    }} >RAM

    /* this is to define the start of the heap, and make sure we have a minimum size */
    .heap :
    {{
        . = ALIGN(4);
        _heap_start = .;    /* define a global symbol at heap start */
    }} >RAM
}}
"""  # nopep8
