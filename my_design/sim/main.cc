/* SPDX-License-Identifier: BSD-2-Clause */
#undef NDEBUG

#include <backends/cxxrtl/cxxrtl.h>
#include "build/sim/sim_soc.h"
#include "spiflash.h"
#include "wb_mon.h"

#include "log.h"

#include <fstream>
#include <filesystem>

using namespace cxxrtl_design;

int main(int argc, char **argv) {
    cxxrtl_design::p_sim__top top;

#if 0
    // uncomment to log WB transactions
    wb_mon_set_output(*top.cell_p_bus__mon, "build/wishbone_log.csv");
#endif
    spiflash_load(*top.cell_p_flash, "../software/software.bin", 0x00100000U);

    top.step();
    auto tick = [&]() {
        top.p_clk.set(false);
        top.step();
        top.p_clk.set(true);
        top.step();
    };
    top.p_rst.set(true);
    tick();
    top.p_rst.set(false);

    while (1) {
        tick();
    }
    return 0;
}
