# Example SoCs

Dependencies:
 - https://gitlab.com/ChipFlow/amaranth-orchard
 - https://gitlab.com/ChipFlow/amaranth-vexriscv
 - 32-bit RISC-V toolchain

## Building in simulation

Build SoC and sim models

```
cd example_socs/mpw5/sim
make
```

Build BIOS:

```
cd example_socs/mpw5/software
make
```

Run simulation:

```
cd example_socs/mpw5/sim
./build/sim_soc
```

## Building for ULX3S

Build SoC:

```
python -m example_socs.mpw5.ulx3s
```

Build and program BIOS:

```
cd example_socs/mpw5/software
make
openFPGALoader -fb ulx3s -o 0x00100000 bios.bin
```

Program bitstream:

```
openFPGALoader -b ulx3s build/top.bit
```

## Building for Sky130

Make sure Yosys and Coriolis are in environment

```
python -m example_socs.mpw5.sky130 --synth --pnr
```
