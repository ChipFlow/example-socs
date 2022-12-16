.PHONY: init # Init local environemnt
init: 
	poetry install

.PHONY: sim-build # Builds a local binary to run the design in simulation
sim-build:
	poetry run python -m chipflow.cli sim build

.PHONY: software-build # Builds the RISC-V software/bios to run on the design
software-build: sim-build
	poetry run python -m chipflow.cli software build

.PHONY: board-build # Build a bitstream for the board
board-build:
	export NEXTPNR_ECP5=yowasp-nextpnr-ecp5 && \
	export ECPPACK=yowasp-ecppack && \
	export YOSYS=yowasp-yosys && \
	poetry run python -m chipflow.cli board

.PHONY: board-load-software-ulx3s # Load the software/bios onto a ulx3s board
board-load-software-ulx3s:
	openFPGALoader -fb ulx3s -o 0x00100000 build/software/software.bin

.PHONY: board-load-ulx3s # Load the design onto a ulx3s board
board-load-ulx3s:
	openFPGALoader -b ulx3s build/top.bit

.PHONY: sim-run # Run the simulation of the design
sim-run:
	cd build/sim && ./sim_soc

.PHONY: silicon-rtlil # Build RTLIL for the design
silicon-rtlil:
	poetry run python -m chipflow.cli silicon_rtlil

.PHONY: silicon-prepare # Send to API to prepare for manufacture
silicon-prepare:
	@echo "See https://chipflow.io/beta for details on how to join the beta"

.PHONY: clean # Clean/delete the builds
clean: 
	rm -fr build

.PHONY: lint # Lint code
lint: 
	poetry run pycodestyle --config=./.pycodestyle my_design/*
	poetry run pycodestyle --config=./.pycodestyle tests/*
	poetry run pycodestyle --config=./.pycodestyle lib/chipflow/chipflow
