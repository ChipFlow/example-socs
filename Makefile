# Prevent sharing of our host .ssh folder with dockcross - we don't need to git clone anything
# TODO: Refactor this to remove file layout dependency
DOCKCROSS_CMD := SSH_DIR="/dev/null" ./lib/chipflow/chipflow/software/dockcross-linux-riscv32

init: # Init local environemnt
	poetry install

build-simulation: # Builds a local binary to run the design in simulation
	poetry run python -m chipflow.cli sim build

build-bios: build-simulation # Builds the RISC-V bios to run on the design
	poetry run python -m chipflow.cli software build

build-ulx3s:
	export NEXTPNR_ECP5=yowasp-nextpnr-ecp5 && \
	export ECPPACK=yowasp-ecppack && \
	export YOSYS=yowasp-yosys && \
	poetry run python -m chipflow.cli ulx3s

load-ulx3s-bios:
	openFPGALoader -fb ulx3s -o 0x00100000 build/software/bios.bin

load-ulx3s:
	openFPGALoader -b ulx3s build/top.bit

run-simulation:
	cd build/sim && ./sim_soc

build-rtlil:
	poetry run python -m chipflow.cli gen_rtlil

send-to-chipflow:
	echo "See https://chipflow.io for details on how to join the beta"

clean: 
	rm -fr build
