# Example ChipFlow design

## Preparing your local environment

 - Ensure you have [Poetry installed](https://python-poetry.org/docs/#installation).
 - Ensure you have Docker (or podman) available, which is used for the 
   [dockcross](https://github.com/dockcross/dockcross) builds.
 - Clone this repository to your local environment.
 - Run `make init` to install the dependencies.

## Run the design in simulation

Run your design in a local simulation binary. 
This will build the BIOS and simulation code first:

```
make run-simulation
```

You should see something like this:

![Simulation output](docs/simulation-output.png)

## Run the design on a ULX3S board

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

## Send your design to the ChipFlow cloud

```
make send-to-chipflow
```
