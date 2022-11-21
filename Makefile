.PHONY: init # Init local environemnt
init: 
	poetry install

.PHONY: build-simulation # Builds a local binary to run the design in simulation
build-simulation:
	poetry run python -m chipflow.cli sim build

.PHONY: build-software # Builds the RISC-V software/bios to run on the design
build-software: build-simulation
	poetry run python -m chipflow.cli software build

.PHONY: build-board # Build a bitstream for the board
build-board:
	export NEXTPNR_ECP5=yowasp-nextpnr-ecp5 && \
	export ECPPACK=yowasp-ecppack && \
	export YOSYS=yowasp-yosys && \
	poetry run python -m chipflow.cli board

.PHONY: load-board-software-ulx3s # Load the software/bios onto a ulx3s board
load-board-software-ulx3s:
	openFPGALoader -fb ulx3s -o 0x00100000 build/software/software.bin

.PHONY: load-board-ulx3s # Load the design onto a ulx3s board
load-board-ulx3s:
	openFPGALoader -b ulx3s build/top.bit

.PHONY: run-simulation # Run the simulation of the design
run-simulation:
	cd build/sim && ./sim_soc

.PHONY: build-rtlil # Build RTLIL for the design
build-rtlil:
	poetry run python -m chipflow.cli gen_rtlil

.PHONY: send-to-chipflow # Send the design to ChipFlow
send-to-chipflow:
	echo "See https://chipflow.io for details on how to join the beta"

.PHONY: clean # Clean/delete the builds
clean: 
	rm -fr build

.PHONY: lint # Lint code
lint: 
	poetry run pycodestyle --config=./.pycodestyle my_design/*
	poetry run pycodestyle --config=./.pycodestyle tests/*
