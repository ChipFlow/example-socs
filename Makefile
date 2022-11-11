# Prevent sharing of our host .ssh folder with dockcross - we don't need to git clone anything
DOCKCROSS_CMD := SSH_DIR="/dev/null" ./chipflow_examples/common/software/dockcross-linux-riscv32

init: # Init local environemnt
	poetry install

build-simulation: # Builds a local binary to run the design in simulation
	make -C chipflow_examples/mpw5/sim

build-bios: build-simulation # Builds the RISC-V bios to run on the design
	$(DOCKCROSS_CMD) make -C chipflow_examples/mpw5/software/

build-all: build-bios

run-simulation: build-bios # Run the simulation binary
	cd chipflow_examples/mpw5/sim && ./build/sim_soc

send-to-chipflow:
	echo "See https://chipflow.io for details on how to join the beta"

clean-all: 
	make -C chipflow_examples/mpw5/sim clean
	$(DOCKCROSS_CMD) make -C chipflow_examples/mpw5/software/ clean
