#include <backends/cxxrtl/cxxrtl.h>
#include <fstream>
#include <stdexcept>
#include "build/sim_soc.h"
#include "log.h"

namespace cxxrtl_design {

struct uart_model : public bb_p_uart__model {
    struct {
        bool tx_last;
        int counter = 0;
        uint8_t sr = 0;
    } s, sn;

    int baud_div = 0;
    uart_model() {
        // TODO: don't hardcode
        baud_div = (2*25000000)/115200;
    }

    bool eval() override {
        sn = s;
        if (sn.counter == 0) {
            if (sn.tx_last && !p_tx__o) { // start bit
                sn.counter = 1;
            }
        } else {
            ++sn.counter;
            if (sn.counter > (baud_div / 2) && ((sn.counter - (baud_div / 2)) % baud_div) == 0) {
                int bit = ((sn.counter - (baud_div / 2)) / baud_div);
                if (bit >= 1 && bit <= 8) {
                    // update shift register
                    sn.sr = (p_tx__o ? 0x80U : 0x00U) | (sn.sr >> 1U);
                }
                if (bit == 8) {
                    // print to console
                    log("%c", char(sn.sr));
                }
                if (bit == 9) {
                    // end
                    sn.counter = 0;
                }
            }
        }
        sn.tx_last = bool(p_tx__o);
        return true;
    }

    bool commit() override {
        bool changed = bb_p_uart__model::commit();
        s = sn;
        return changed;
    }

    void reset() override {
    };
    ~uart_model() {};
};

std::unique_ptr<bb_p_uart__model> bb_p_uart__model::create(std::string name, metadata_map parameters, metadata_map attributes) {
    return std::make_unique<uart_model>();
}

};
