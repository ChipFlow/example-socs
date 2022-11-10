
build-mpw5-simulation: 
	make -C chipflow_examples/mpw5/sim

build-mpw5-bios: build-mpw5-simulation
	# Prevent sharing of our host .ssh folder with dockcross - we don't need to git clone anything
	SSH_DIR="/dev/null" ./chipflow_examples/common/software/dockcross-linux-riscv32 make -C chipflow_examples/mpw5/software/

run-mpw5-simulation: build-mpw5-bios
	cd chipflow_examples/mpw5/sim && ./build/sim_soc

clean-all: 
	make -C chipflow_examples/mpw5/sim clean
	make -C chipflow_examples/mpw5/software/ clean
