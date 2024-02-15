.PHONY: init # Init local environemnt
init:
	pdm install

.PHONY: sim-build # Builds a local binary to run the design in simulation
sim-build:
	pdm run chipflow sim

.PHONY: software-build # Builds the RISC-V software/bios to run on the design
software-build: sim-build
	pdm run chipflow software

.PHONY: board-build # Build a bitstream for the board
board-build:
	pdm run chipflow board

.PHONY: board-load-software-ulx3s # Load the software/bios onto a ulx3s board
board-load-software-ulx3s:
	openFPGALoader -fb ulx3s -o 0x00100000 build/software/software.bin

.PHONY: board-load-ulx3s # Load the design onto a ulx3s board
board-load-ulx3s:
	openFPGALoader -b ulx3s build/top.bit

.PHONY: sim-run # Run the simulation of the design
sim-run: sim-build software-build
	cd build/sim && ./sim_soc

.PHONY: silicon-prepare # Build RTLIL for the design
silicon-prepare:
	pdm run chipflow silicon prepare

.PHONY: silicon-submit # Send to API to submit for manufacture
silicon-submit:
	pdm run chipflow silicon submit

.PHONY: clean # Clean/delete the builds
clean:
	rm -fr build
