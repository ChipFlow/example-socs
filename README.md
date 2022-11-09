# Example SoCs

## Preparing your local environment

 - Ensure you have [Poetry installed](https://python-poetry.org/docs/#installation).
 - Ensure you have a riscv compiler:
   - macOS: `brew tap riscv-software-src/riscv` then `brew install riscv-tools` (See [homebrew-riscv](https://github.com/riscv-software-src/homebrew-riscv))
 - Clone this repository to your local environment.
 - Run `poetry install` to install the dependencies.

## Building in simulation

Build SoC and sim models

```
cd chipflow_examples/mpw5/sim
make
```

Build BIOS:

```
cd chipflow_examples/mpw5/software
make
```

Run simulation:

```
cd chipflow_examples/mpw5/sim
./build/sim_soc
```

## Building for ULX3S

Build SoC:

```
python -m chipflow_examples.mpw5.ulx3s
```

Build and program BIOS:

```
cd chipflow_examples/mpw5/software
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
python -m chipflow_examples.mpw5.sky130 --synth --pnr
```
