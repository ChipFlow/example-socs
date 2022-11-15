# Prevent sharing of our host .ssh folder with dockcross - we don't need to git clone anything
# TODO: Refactor this to remove file layout dependency
DOCKCROSS_CMD := SSH_DIR="/dev/null" ./lib/chipflow/chipflow/software/dockcross-linux-riscv32

init: # Init local environemnt
	poetry install

build-simulation: # Builds a local binary to run the design in simulation
	make -C my_design/sim

build-bios: build-simulation # Builds the RISC-V bios to run on the design
	$(DOCKCROSS_CMD) make -C my_design/software/

build-ulx3s:
	export NEXTPNR_ECP5=yowasp-nextpnr-ecp5 && \
	export ECPPACK=yowasp-ecppack && \
	export YOSYS=yowasp-yosys && \
	poetry run python -m chipflow.cli ulx3s

build-all: build-bios

load-ulx3s-bios:
	openFPGALoader -fb ulx3s -o 0x00100000 my_design/software/bios.bin

load-ulx3s:
	openFPGALoader -b ulx3s build/top.bit

run-simulation: build-bios
	cd my_design/sim && ./build/sim_soc

send-to-chipflow:
	echo "See https://chipflow.io for details on how to join the beta"

clean-bios: 
	$(DOCKCROSS_CMD) make -C my_design/software/ clean

clean-all: clean-bios
	make -C my_design/sim clean
