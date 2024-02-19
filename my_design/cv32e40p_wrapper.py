from amaranth import *
from amaranth.lib import wiring
from amaranth.lib.wiring import In, Out, flipped, connect

from amaranth_soc import wishbone
from pathlib import Path

class OBISignature(wiring.Signature):
    def __init__(self):
        super().__init__({
            "req":    Out(1),
            "gnt":    In(1),
            "rvalid": In(1),
            "we":     Out(1),
            "be":     Out(4),
            "addr":   Out(32),
            "wdata":  Out(32),
            "rdata": In(32),
        })

class OBI2Wishbone(wiring.Component):
    # From https://github.com/enjoy-digital/litex/blob/master/litex/soc/cores/cpu/cv32e40p/core.py
    def __init__(self):
        super().__init__()
    @property
    def signature(self):
        return wiring.Signature({
            "obi": In(OBISignature()),
            "wb": Out(wishbone.Signature(addr_width=30,
                    data_width=32, granularity=8)),
        })
    def elaborate(self, platform):
        m = Module()
        addr = Signal.like(self.obi.addr)
        be = Signal.like(self.obi.be)
        we = Signal.like(self.obi.we)
        wdata = Signal.like(self.obi.wdata)
        m.d.comb += self.obi.gnt.eq(0)
        with m.FSM():
            with m.State("IDLE"):
                with m.If(self.obi.req):
                    # Drive Wishbone bus from OBI bus.
                    m.d.comb += [
                        self.wb.adr.eq(self.obi.addr[2:]),
                        self.wb.stb.eq(1),
                        self.wb.cyc.eq(1),
                        self.wb.dat_w.eq(self.obi.wdata),
                        self.wb.sel.eq(self.obi.be),
                        self.wb.we.eq(self.obi.we),
                    ]
                    # Store OBI bus values.
                    m.d.sync += [
                        addr.eq(self.obi.addr),
                        be.eq(self.obi.be),
                        we.eq(self.obi.we),
                        wdata.eq(self.obi.wdata),
                    ]
                    m.next = "WAIT_ACK"
                m.d.comb += self.obi.gnt.eq(1)
            with m.State("WAIT_ACK"):
                m.d.comb += [
                    self.wb.adr.eq(addr[2:]),
                    self.wb.stb.eq(1),
                    self.wb.cyc.eq(1),
                    self.wb.dat_w.eq(wdata),
                    self.wb.sel.eq(be),
                    self.wb.we.eq(we),
                ]
                with m.If(self.wb.ack):
                    m.d.comb += [
                        self.obi.rvalid.eq(1)
                    ]
                    m.next = "IDLE"
        m.d.comb += self.obi.rdata.eq(self.wb.dat_r)
        return m

class CV32E40P(Elaboratable):
    def __init__(self, config="default", reset_vector=0x00100000):
        self.config = config
        self.dbus = wishbone.Interface(addr_width=30,
                                      data_width=32, granularity=8) 
        self.ibus = wishbone.Interface(addr_width=30,
                                      data_width=32, granularity=8)
        self.timer_irq = Signal()
        self.software_irq = Signal()
        self.irq_sigs = [None for i in range(32)]
        self.reset_vector = reset_vector
    def add_irq(self, irq, sig):
        assert self.irq_sigs[irq] is None
        self.irq_sigs[irq] = sig
    def elaborate(self, platform):
        m = Module()
        obi_ibus = OBISignature().create(path="obi_ibus")
        obi_dbus = OBISignature().create(path="obi_dbus")
        ext_irq = Signal(32)

        conn = dict(
            i_clk_i=ClockSignal(),
            i_rst_ni=~ResetSignal(),
            i_clock_en_i=1,
            i_test_en_i=0, # not using clock gating for testchip, so we don't need this either
            i_fregfile_disable_i=1,

            i_boot_addr_i=self.reset_vector,
            i_core_id_i=0,
            i_cluster_id_i=0,

            o_instr_req_o=obi_ibus.req,
            i_instr_gnt_i=obi_ibus.gnt,
            i_instr_rvalid_i=obi_ibus.rvalid,
            o_instr_addr_o=obi_ibus.addr,
            i_instr_rdata_i=obi_ibus.rdata,

            o_data_req_o=obi_dbus.req,
            i_data_gnt_i=obi_dbus.gnt,
            i_data_rvalid_i=obi_dbus.rvalid,
            o_data_we_o=obi_dbus.we,
            o_data_be_o=obi_dbus.be,
            o_data_addr_o=obi_dbus.addr,
            o_data_wdata_o=obi_dbus.wdata,
            i_data_rdata_i=obi_dbus.rdata,

            i_irq_software_i=self.software_irq,
            i_irq_timer_i=self.timer_irq,
            i_irq_nmi_i=0,
            i_irq_fast_i=0,
            i_irq_external_i=0,
            i_irq_fastx_i=ext_irq, # TODO: IRQ?

            i_debug_req_i=0,
            i_fetch_enable_i=1, # debug?
        )

        m.submodules.cpu = cpu = Instance("riscv_core", **conn)

        m.submodules.ibus_adapt = ibus_adapt = OBI2Wishbone()
        m.submodules.dbus_adapt = dbus_adapt = OBI2Wishbone()
        connect(m, obi_ibus, ibus_adapt.obi)
        connect(m, obi_dbus, dbus_adapt.obi)
        connect(m, ibus_adapt.wb, flipped(self.ibus))
        connect(m, dbus_adapt.wb, flipped(self.dbus))

        m.d.comb += [
            obi_ibus.we.eq(0)
        ]
        path = Path(__file__).parent / f"verilog/riscv_core_conv_sv2v.v"
        with open(path, 'r') as f:
            platform.add_file(path.name, f)
        return m
