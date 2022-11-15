all: bios.bin

BIOS_SOURCES=main.c mmode.c
BIOS_HEADERS=$(wildcard generated/*.h) mmode.h riscv_csr.h
# TODO: This dir needs to come from the chipflow python module
CHIPFLOW_SOFTWARE_DIR=../../lib/chipflow/chipflow/software

include $(CHIPFLOW_SOFTWARE_DIR)/bios.mk
