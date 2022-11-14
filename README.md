# Example ChipFlow design

## Preparing your local environment

 - Ensure you have [Poetry installed](https://python-poetry.org/docs/#installation).
 - Ensure you have Docker (or podman) available, which is used for the 
   [dockcross](https://github.com/dockcross/dockcross) builds.
 - If you want to use a board, ensure you have openfpgaloader
  - macOS: `brew install openfpgaloader`
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

Build SoC (doesn't load it):

```
make build-ulx3s
```

Build and program BIOS into the board's flash:

```
make build-mpw5-bios
make load-ulx3s-bios
```

Load SoC onto board (program its bitstream):

```
make load-ulx3s
```

## Send your design to the ChipFlow cloud

```
make send-to-chipflow
```
