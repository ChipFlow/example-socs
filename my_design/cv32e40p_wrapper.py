from amaranth import *
from amaranth.lib import wiring
from amaranth.lib.wiring import In, Out, flipped, connect
from amaranth.utils import bits_for

from amaranth_soc import wishbone
from pathlib import Path


__all__ = ["CV32E40P"]


class _OBISignature(wiring.Signature):
    def __init__(self):
        super().__init__({
            "req":    Out(1),
            "gnt":    In(1),
            "rvalid": In(1),
            "we":     Out(1),
            "be":     Out(4),
            "addr":   Out(32),
            "wdata":  Out(32),
            "rdata":  In(32),
        })


class _OBI2Wishbone(wiring.Component):
    # From https://github.com/enjoy-digital/litex/blob/master/litex/soc/cores/cpu/cv32e40p/core.py
    def __init__(self):
        super().__init__({
            "obi": In(_OBISignature()),
            "wb":  Out(wishbone.Signature(addr_width=30, data_width=32, granularity=8)),
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


class CV32E40P(wiring.Component):
    def __init__(self, *, config="default", reset_vector=0x00100000):
        if config not in ("default",):
            raise ValueError(f"Unsupported configuration {config!r}; must be one of: 'default'")
        if not isinstance(reset_vector, int) or reset_vector < 0:
            raise TypeError(f"Reset vector address must be a non-negative integer, not "
                            f"{reset_vector!r}")
        if bits_for(reset_vector) > 32:
            raise ValueError(f"Reset vector address {reset_vector:#x} cannot be represented as a "
                             f"32-bit integer")

        super().__init__({
            "ibus":         Out(wishbone.Signature(addr_width=30, data_width=32, granularity=8)),
            "dbus":         Out(wishbone.Signature(addr_width=30, data_width=32, granularity=8)),
            "timer_irq":    In(unsigned(1)),
            "software_irq": In(unsigned(1)),
        })

        self._config = config
        self._reset_vector = reset_vector

    @property
    def config(self):
        return self._config

    @property
    def reset_vector(self):
        return self._reset_vector

    def elaborate(self, platform):
        m = Module()

        ext_irq = Signal(32)

        m.submodules.ibus_adapt = ibus_adapt = _OBI2Wishbone()
        m.submodules.dbus_adapt = dbus_adapt = _OBI2Wishbone()

        m.submodules.riscv_core = Instance("riscv_core",
            i_clk_i=ClockSignal(),
            i_rst_ni=~ResetSignal(),
            i_clock_en_i=1,
            i_test_en_i=0, # not using clock gating for testchip, so we don't need this either
            i_fregfile_disable_i=1,

            i_boot_addr_i=self.reset_vector,
            i_core_id_i=0,
            i_cluster_id_i=0,

            o_instr_req_o=ibus_adapt.obi.req,
            i_instr_gnt_i=ibus_adapt.obi.gnt,
            i_instr_rvalid_i=ibus_adapt.obi.rvalid,
            o_instr_addr_o=ibus_adapt.obi.addr,
            i_instr_rdata_i=ibus_adapt.obi.rdata,

            o_data_req_o=dbus_adapt.obi.req,
            i_data_gnt_i=dbus_adapt.obi.gnt,
            i_data_rvalid_i=dbus_adapt.obi.rvalid,
            o_data_we_o=dbus_adapt.obi.we,
            o_data_be_o=dbus_adapt.obi.be,
            o_data_addr_o=dbus_adapt.obi.addr,
            o_data_wdata_o=dbus_adapt.obi.wdata,
            i_data_rdata_i=dbus_adapt.obi.rdata,

            i_irq_software_i=self.software_irq,
            i_irq_timer_i=self.timer_irq,
            i_irq_nmi_i=0,
            i_irq_fast_i=0,
            i_irq_external_i=0,
            i_irq_fastx_i=ext_irq, # TODO: IRQ?

            i_debug_req_i=0,
            i_fetch_enable_i=1, # debug?
        )

        m.d.comb += ibus_adapt.obi.we.eq(0)

        connect(m, ibus_adapt.wb, flipped(self.ibus))
        connect(m, dbus_adapt.wb, flipped(self.dbus))

        path = Path(__file__).parent / f"verilog/riscv_core_conv_sv2v.v"
        with open(path, 'r') as f:
            platform.add_file(path.name, f)
        return m
