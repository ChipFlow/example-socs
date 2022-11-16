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

First we need to build a local simulation binary. The simulation uses blackbox C++ models 
of external peripherals, such as the flash, to interact with:

```
make build-simulation
```

Next we need to build the software/BIOS which will run on our design. The build
of this depends on the design itself.

```
make build-bios
```

Now that we have our simulation and a BIOS, we can run it:

```
make run-simulation
```

You should see something like this:

![Simulation output](docs/simulation-output.png)

## Run the design on a ULX3S board

Build the design into a bitstream for the board (doesn't load it):

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

Your board should now be running. You can connect to it via its serial port:

### Connecting to your board on macOS

* Find the serial port for your board, using `ls /dev/tty.*` or `ls /dev/cu.*`. 
  You should see something like `/dev/tty.usbserial-K00219` for your board.
* Connect to the port via the screen utility, at baud 112200, with the command:
  `screen /dev/tty.usbserial-K00219 115200`.
* Now, press the `PWR` button on your board, which will restart the design.
* Within screen, should now see output like:
  ![Board output](docs/board-output.png)
* To exit screen, use `CTRL-A`, then `CTRL-\`.


## Generate an RTLIL from your design

```
make build-rtlil
```

You should now have an `build/my_design.rtlil`.

## Send your RTLIL to the ChipFlow cloud

```
make send-to-chipflow
```
