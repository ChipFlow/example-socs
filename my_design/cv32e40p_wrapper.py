from amaranth import *
from amaranth.lib import wiring
from amaranth.lib.wiring import In, Out, flipped, connect
from amaranth.utils import bits_for

from amaranth_soc import wishbone
from amaranth_soc.memory import MemoryMap

from pathlib import Path


__all__ = ["CV32E40P", "DebugModule", "JTAG"]


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

class _Wishbone2OBI(wiring.Component):
    # From https://github.com/enjoy-digital/litex/blob/master/litex/soc/cores/cpu/cv32e40p/core.py
    # This is required because the debug module has a OBI peripheral interface that needs to be adapted, too
    def __init__(self, addr_width=30):
        super().__init__({
            "obi": Out(_OBISignature()),
            "wb":  In(wishbone.Signature(addr_width=addr_width, data_width=32, granularity=8)),
        })
    def elaborate(self, platform):
        m = Module()
        with m.FSM():
            with m.State("IDLE"):
                with m.If(self.wb.cyc & self.wb.stb):
                    m.d.comb += self.obi.req.eq(1)
                    with m.If(self.obi.gnt):
                        m.next = "ACK"
            with m.State("ACK"):
                with m.If(self.wb.we | self.obi.rvalid):
                    m.d.comb += self.wb.ack.eq(1)
                    m.next = "IDLE"
        m.d.comb += [
            self.obi.we.eq(self.wb.we),
            self.obi.be.eq(self.wb.sel),
            self.obi.addr.eq(Cat(C(0, 2), self.wb.adr)),
            self.obi.wdata.eq(self.wb.dat_w),
            self.wb.dat_r.eq(self.obi.rdata),
        ]
        return m

class CV32E40P(wiring.Component):
    def __init__(self, *, config="default", reset_vector=0x00100000, dm_haltaddress=0xa0000800):
        if config not in ("default",):
            raise ValueError(f"Unsupported configuration {config!r}; must be one of: 'default'")
        if not isinstance(reset_vector, int) or reset_vector < 0:
            raise TypeError(f"Reset vector address must be a non-negative integer, not "
                            f"{reset_vector!r}")
        if bits_for(reset_vector) > 32:
            raise ValueError(f"Reset vector address {reset_vector:#x} cannot be represented as a "
                             f"32-bit integer")

        if not isinstance(dm_haltaddress, int) or dm_haltaddress < 0:
            raise TypeError(f"DM halt address must be a non-negative integer, not "
                            f"{dm_haltaddress!r}")
        if bits_for(dm_haltaddress) > 32:
            raise ValueError(f"DM halt address {dm_haltaddress:#x} cannot be represented as a "
                             f"32-bit integer")

        super().__init__({
            "ibus":         Out(wishbone.Signature(addr_width=30, data_width=32, granularity=8)),
            "dbus":         Out(wishbone.Signature(addr_width=30, data_width=32, granularity=8)),
            "timer_irq":    In(unsigned(1)),
            "software_irq": In(unsigned(1)),
            "debug_req": In(unsigned(1)),
        })

        self._config = config
        self._reset_vector = reset_vector
        self._dm_haltaddress = dm_haltaddress

    @property
    def config(self):
        return self._config

    @property
    def reset_vector(self):
        return self._reset_vector

    @property
    def dm_haltaddress(self):
        return self._dm_haltaddress

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

            p_DM_HaltAddress=self._dm_haltaddress,
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

            i_debug_req_i=self.debug_req,
            i_fetch_enable_i=1, # debug?
        )

        m.d.comb += ibus_adapt.obi.we.eq(0)

        connect(m, ibus_adapt.wb, flipped(self.ibus))
        connect(m, dbus_adapt.wb, flipped(self.dbus))

        path = Path(__file__).parent / f"verilog/riscv_core_conv_sv2v.v"
        with open(path, 'r') as f:
            platform.add_file(path.name, f)
        return m

class DebugModule(wiring.Component):
    def __init__(self):
        self._addr_width=12

        super().__init__({
            "initiator":   Out(wishbone.Signature(addr_width=30, data_width=32, granularity=8)),
            "target":      In(wishbone.Signature(addr_width=self._addr_width-2, data_width=32, granularity=8)),
            "jtag_tms":    In(1),
            "jtag_tdi":    In(1),
            "jtag_tdo":    Out(1),
            "jtag_tck":    In(1),
            "debug_req":   Out(1),
            "ndmreset":    Out(1),
        })

        memory_map = MemoryMap(addr_width=self._addr_width, data_width=8)
        memory_map.add_resource(name=("debug", ), size=2**self._addr_width, resource=self)
        self.target.memory_map = memory_map

    def elaborate(self, platform):
        m = Module()

        m.submodules.init_adapt = init_adapt = _OBI2Wishbone()
        m.submodules.tgt_adapt = tgt_adapt = _Wishbone2OBI(addr_width=self._addr_width-2)

        m.submodules.dm_wrap = Instance("dm_wrap",
            i_clk_i=ClockSignal(),
            i_rst_ni=~ResetSignal(),
            o_ndmreset_o=self.ndmreset,
            o_debug_req_o=self.debug_req,

            i_dm_req_i=tgt_adapt.obi.req,
            o_dm_gnt_o=tgt_adapt.obi.gnt,
            i_dm_we_i=tgt_adapt.obi.we,
            i_dm_addr_i=tgt_adapt.obi.addr,
            i_dm_be_i=tgt_adapt.obi.be,
            i_dm_wdata_i=tgt_adapt.obi.wdata,
            o_dm_rdata_o=tgt_adapt.obi.rdata,
            o_dm_rvalid_o=tgt_adapt.obi.rvalid,

            o_sb_req_o=init_adapt.obi.req,
            o_sb_addr_o=init_adapt.obi.addr,
            o_sb_we_o=init_adapt.obi.we,
            o_sb_wdata_o=init_adapt.obi.wdata,
            o_sb_be_o=init_adapt.obi.be,
            i_sb_gnt_i=init_adapt.obi.gnt,
            i_sb_rvalid_i=init_adapt.obi.rvalid,
            i_sb_rdata_i=init_adapt.obi.rdata,

            i_tck_i=self.jtag_tck,
            i_tms_i=self.jtag_tms,
            i_trst_ni=1, # TODO
            i_tdi_i=self.jtag_tdi,
            o_tdo_o=self.jtag_tdo
        )

        connect(m, init_adapt.wb, flipped(self.initiator))
        connect(m, tgt_adapt.wb, flipped(self.target))


        path = Path(__file__).parent / f"verilog/dm_wrap_conv_sv2v.v"
        with open(path, 'r') as f:
            platform.add_file(path.name, f)

        return m