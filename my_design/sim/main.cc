/* SPDX-License-Identifier: BSD-2-Clause */
#undef NDEBUG

#include <cxxrtl/cxxrtl.h>
#include <cxxrtl/cxxrtl_vcd.h>
#include "build/sim/sim_soc.h"
#include "spiflash.h"
#include "wb_mon.h"

#include "log.h"

#include <fstream>
#include <filesystem>

#define TRACE 0

using namespace cxxrtl_design;

int main(int argc, char **argv) {
    cxxrtl_design::p_sim__top top;

    /* // uncomment to log WB transactions */
    /* wb_mon_set_output(*top.cell_p_bus__mon, "build/wishbone_log.csv"); */

    spiflash_load(*top.cell_p_spiflash__provider, "../software/software.bin", 0x00100000U);

# if TRACE
    cxxrtl::vcd_writer vcd;
    std::ofstream vcd_file;
    debug_items debug_items;
    uint64_t cycle = 0;

    vcd_file.open("trace.vcd");
    top.debug_info(&debug_items, /*scopes=*/nullptr, "");
    vcd.timescale(1, "us");
    vcd.add_without_memories(debug_items);
#endif

    auto tick = [&]() {
        top.p_clk.set(false);
        top.step();
#if TRACE
	vcd.sample(2 * cycle);
#endif
        top.p_clk.set(true);
        top.step();
#if TRACE
	vcd.sample(2 * cycle + 1);
	vcd_file << vcd.buffer;
	vcd.buffer.clear();
	cycle += 1;
#endif
    };

    top.step();
    top.p_rst.set(true);
    tick();
    top.p_rst.set(false);

    /* int idx = 0; */
    while (1) {
        tick();
        /* idx = (idx + 1) % 1000000; */
        /* // Simulate button presses */
        /* if (idx == 100000) // at t=100000, press button 1 */
        /*     top.p_buttons.set(0b01U); */
        /* else if (idx == 150000) // at t=150000, release button 1 */
        /*     top.p_buttons.set(0b00U); */
        /* else if (idx == 300000) // at t=300000, press button 2 */
        /*     top.p_buttons.set(0b10U); */
        /* else if (idx == 350000) // at t=350000, release button 2 */
        /*     top.p_buttons.set(0b00U); */
    }

    return 0;
}
