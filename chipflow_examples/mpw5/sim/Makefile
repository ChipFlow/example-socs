CXXFLAGS=-O3 -g -std=c++17 -I ../../common/models
RTL_CXXFLGAGS=-O1 -std=c++17
YOSYS_DATDIR = $(shell poetry run python print_yosys_datdir.py)

all: build/sim_soc

build/sim_soc.ys: ../soc.py
	mkdir -p build
	(cd ../../.. && BUILD_DIR=chipflow_examples/mpw5/sim/build poetry run python -m chipflow_examples.mpw5.sim_soc)

build/sim_soc.cc: build/sim_soc.ys
	(cd build && yosys sim_soc.ys)

build/sim_soc.h: build/sim_soc.cc

build/sim_soc.o: build/sim_soc.cc build/sim_soc.h
	$(CXX) -I . -I $(YOSYS_DATDIR)/include $(RTL_CXXFLGAGS) -o $@ -c $<

build/models/%.o: ../../common/models/%.cc build/sim_soc.h
	mkdir -p build/models
	$(CXX) -I . -I $(YOSYS_DATDIR)/include $(CXXFLAGS) -o $@ -c $<

build/%.o: %.cc build/sim_soc.h
	$(CXX) -I . -I $(YOSYS_DATDIR)/include $(CXXFLAGS) -o $@ -c $<

models_objs=$(patsubst ../../common/models/%.cc, build/models/%.o, $(wildcard ../../common/models/*.cc))

build/sim_soc: build/sim_soc.o $(models_objs)  build/main.o
	$(CXX) -o $@ $^

run: build/sim_soc
	./build/sim_soc

clean:
	rm -fr build

.PRECIOUS: build/sim_soc.ys build/sim_soc.cc
.PHONY: run
