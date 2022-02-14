#include <backends/cxxrtl/cxxrtl.h>
#include <fstream>
#include <stdexcept>
#include "build/sim_soc.h"
#include "log.h"

namespace cxxrtl_design {

struct hyperram_model : public bb_p_hyperram__model {
    struct {
        int dev = -1;
        unsigned clk_count = 0;
        uint32_t curr_cs = 0;
        uint64_t ca = 0;
        uint32_t addr = 0;
        uint16_t cfg0 = 0x8028;
        uint16_t latency = 7;
    } s, sn;

    std::vector<uint8_t> data;
    int N; // number of devices

    hyperram_model() {
        N = p_csn__o.bits;
        data.resize(N*8*1024*1024);
    }

    int decode_onecold(uint32_t cs) {
        int result = -1;
        for (int i = 0; i < N; i++) {
            if (((cs >> i) & 0x1) == 0x0) {
                if (result != -1)
                    log("multiple hyperram devices asserted! CS=%02x\n", cs);
                result = i;
            }
        }
        return result;
    }

    uint16_t lookup_latency(uint16_t cfg) {
        uint8_t lat_key = (cfg >> 4) & 0xF;
        switch (lat_key) {
            case 0b0000: return 5;
            case 0b0001: return 6;
            case 0b0010: return 7;
            case 0b1110: return 3;
            case 0b1111: return 4;
            default: log("unknown RAM latency %0x\n", lat_key); return 7;
        }
    }

    void handle_clk(bool posedge)
    {
        if (sn.clk_count < 6) {
            p_rwds__i.set(1U); // 2x latency; always
            sn.ca |= uint64_t(p_dq__o.get<uint8_t>()) << ((5U - sn.clk_count) * 8U);
        }
        if (sn.clk_count == 6) {    
            sn.addr = ((((sn.ca & 0x0FFFFFFFFFULL) >> 16U) << 3) | (sn.ca & 0x7)) * 2; // *2 to convert word address to byte address
            sn.addr += sn.dev * (8U * 1024U * 1024U); // device offsets
        }
        if (sn.clk_count >= 6) {
            bool is_reg = (sn.ca >> 46) & 0x1;
            bool is_read = (sn.ca >> 47) & 0x1;
            if (is_reg && !is_read && sn.clk_count < 8) {
                sn.cfg0 <<= 8;
                sn.cfg0 |= p_dq__o.get<uint8_t>();
                if (sn.clk_count == 7) {
                    sn.latency = lookup_latency(sn.cfg0);
                    // log("set latency %d\n", sn.latency);
                }
            } else if (is_read && (sn.clk_count >= (3 + 4 * sn.latency))) {
                // log("read %08x %02x\n", sn.addr, data.at(sn.addr));
                p_dq__i.set(data.at(sn.addr++));
                p_rwds__i.set(posedge);
            } else if (!is_read && (sn.clk_count >= (4 + 4 * sn.latency))) {
                if (!p_rwds__o) { // data mask 
                    // log("write %08x %02x\n", sn.addr, p_dq__o.get<uint8_t>());
                    data.at(sn.addr) = p_dq__o.get<uint8_t>();
                } else {
                    // log("write %08x XX\n", sn.addr);
                }
                sn.addr++;
            }
        }
        if (sn.addr >= data.size())
            sn.addr = 0;
        ++sn.clk_count;
    }

    bool eval() override {
        sn = s;
        sn.curr_cs = p_csn__o.get<uint32_t>();
        if (sn.curr_cs != s.curr_cs) {
            // reset selected device
            sn.dev = decode_onecold(sn.curr_cs);
            // log("sel %d\n", sn.dev);
            sn.clk_count = 0;
            sn.ca = 0;
        }
        if (posedge_p_clk__o() && sn.dev != -1) {
            handle_clk(true);
        } else if (negedge_p_clk__o() && sn.dev != -1) {
            handle_clk(false);
        }
        return true;
    }

    bool commit() override {
        bool changed = bb_p_hyperram__model::commit();
        s = sn;
        return changed;
    }

    void reset() override {
    };
    ~hyperram_model() {};
};

std::unique_ptr<bb_p_hyperram__model> bb_p_hyperram__model::create(std::string name, metadata_map parameters, metadata_map attributes) {
    return std::make_unique<hyperram_model>();
}

};
