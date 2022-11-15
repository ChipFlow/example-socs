#include <backends/cxxrtl/cxxrtl.h>
#include <fstream>
#include <stdexcept>
#include <array>
#include "build/sim_soc.h"
#include "log.h"

namespace cxxrtl_design {

struct spiflash_model : public bb_p_spiflash__model {
    struct {
        int bit_count = 0;
        int byte_count = 0;
        unsigned data_width = 1;
        uint32_t addr = 0;
        uint8_t curr_byte = 0;
        uint8_t command = 0;
        uint8_t out_buffer = 0;
    } s, sn;

    std::vector<uint8_t> data;

    spiflash_model() {
        // TODO: don't hardcode
        data.resize(16*1024*1024);
        std::fill(data.begin(), data.end(), 0xFF); // flash starting value
    }

    void load(const std::string &file, size_t offset) {
        std::ifstream in(file, std::ifstream::binary);
        if (offset >= data.size()) {
            throw std::out_of_range("flash: offset beyond end");
        }
        if (!in) {
            throw std::runtime_error("flash: failed to read input file!");
        }
        in.read(reinterpret_cast<char*>(data.data() + offset), (data.size() - offset));
    }

    void process_byte() {
        sn.out_buffer = 0;
        if (sn.byte_count == 0) {
            sn.addr = 0;
            sn.data_width = 1;
            sn.command = sn.curr_byte;
            if (sn.command == 0xab) {
                // power up
            } else if (sn.command == 0x03 || sn.command == 0x9f || sn.command == 0xff
                || sn.command == 0x35 || sn.command == 0x31 || sn.command == 0x50) {
                // nothing to do
            } else if (sn.command == 0xeb) {
                sn.data_width = 4;
            } else {
                log("flash: unknown command %02x\n", sn.command);
            }
        } else {
            if (sn.command == 0x03) {
                // Single read
                if (sn.byte_count <= 3) {
                    sn.addr |= (uint32_t(sn.curr_byte) << ((3 - sn.byte_count) * 8));
                }
                if (sn.byte_count >= 3) {
                    //if (sn.byte_count == 3)
                    //    log("flash: begin read 0x%06x\n", sn.addr);
                    sn.out_buffer = data.at(sn.addr);
                    sn.addr = (sn.addr + 1) & 0x00FFFFFF;
                }
            } else if (sn.command == 0xeb) {
                // Quad read
                if (sn.byte_count <= 3) {
                    sn.addr |= (uint32_t(sn.curr_byte) << ((3 - sn.byte_count) * 8));
                }
                if (sn.byte_count >= 6) { // 1 mode, 2 dummy clocks
                    // read 4 bytes
                    sn.out_buffer = data.at(sn.addr);
                    sn.addr = (sn.addr + 1) & 0x00FFFFFF;
                }
            }
        }
        if (sn.command == 0x9f) {
            // Read ID
            static const std::array<uint8_t, 4> flash_id{0xCA, 0x7C, 0xA7, 0xFF};
            sn.out_buffer = flash_id.at(sn.byte_count % int(flash_id.size()));
        }
    }

    bool eval() override {
        sn = s;
        if (posedge_p_csn__o()) {
            sn.bit_count = 0;
            sn.byte_count = 0;
            sn.data_width = 1;
        } else if (posedge_p_clk__o() && !p_csn__o) {
            if (sn.data_width == 4)
                sn.curr_byte = (sn.curr_byte << 4U) | (p_d__o.get<uint32_t>() & 0xF);
            else
                sn.curr_byte = (sn.curr_byte << 1U) | p_d__o.bit(0);
            sn.out_buffer = sn.out_buffer << unsigned(sn.data_width);
            sn.bit_count += sn.data_width;
            if ((sn.bit_count) == 8) {
                process_byte();
                ++sn.byte_count;
                sn.bit_count = 0;
            }
        } else if (negedge_p_clk__o() && !p_csn__o) {
            if (sn.data_width == 4) {
                p_d__i.set((sn.out_buffer >> 4U) & 0xFU);
            } else {
                p_d__i.set(((sn.out_buffer >> 7U) & 0x1U) << 1U);
            }
        }
        return true;
    }

    bool commit() override {
        bool changed = bb_p_spiflash__model::commit();
        s = sn;
        return changed;
    }

    void reset() override {
    };
    ~spiflash_model() {};
};

std::unique_ptr<bb_p_spiflash__model> bb_p_spiflash__model::create(std::string name, metadata_map parameters, metadata_map attributes) {
    return std::make_unique<spiflash_model>();
}

void spiflash_load(bb_p_spiflash__model &flash, const std::string &file, size_t offset) {
    dynamic_cast<spiflash_model&>(flash).load(file, offset);
}

};
