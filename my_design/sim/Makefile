# TODO: This dir needs to come from the chipflow python module
CHIPFLOW_MODEL_DIR=../../lib/chipflow/chipflow/models
CXXFLAGS=-O3 -g -std=c++17 -I $(CHIPFLOW_MODEL_DIR)
RTL_CXXFLGAGS=-O1 -std=c++17
YOSYS_DATDIR = $(shell poetry run python print_yosys_datdir.py)

all: build/sim_soc

build/sim_soc.ys:
	mkdir -p build
	(cd ../.. && BUILD_DIR=my_design/sim/build poetry run python -m chipflow.cli simulate)

build/sim_soc.cc: build/sim_soc.ys
	cd build && poetry run yowasp-yosys sim_soc.ys

build/sim_soc.h: build/sim_soc.cc

build/sim_soc.o: build/sim_soc.cc build/sim_soc.h
	$(CXX) -I . -I $(YOSYS_DATDIR)/include $(RTL_CXXFLGAGS) -o $@ -c $<

build/models/%.o: $(CHIPFLOW_MODEL_DIR)/%.cc build/sim_soc.h
	mkdir -p build/models
	$(CXX) -I . -I $(YOSYS_DATDIR)/include $(CXXFLAGS) -o $@ -c $<

build/%.o: %.cc build/sim_soc.h
	$(CXX) -I . -I $(YOSYS_DATDIR)/include $(CXXFLAGS) -o $@ -c $<

models_objs=$(patsubst $(CHIPFLOW_MODEL_DIR)/%.cc, build/models/%.o, $(wildcard $(CHIPFLOW_MODEL_DIR)/*.cc))

build/sim_soc: build/sim_soc.o $(models_objs)  build/main.o
	$(CXX) -o $@ $^

run: build/sim_soc
	./build/sim_soc

clean:
	rm -fr build

.PRECIOUS: build/sim_soc.ys build/sim_soc.cc
.PHONY: run
