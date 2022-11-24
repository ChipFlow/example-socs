# SPDX-License-Identifier: BSD-2-Clause
import subprocess
from pathlib import Path

from amaranth import *
from amaranth.back import rtlil


class Sky130Platform():
    def __init__(self, pin_map, io_count=38, core_size=(280*10.0, 330*10.0)):
        self.pin_map = pin_map
        self.build_dir = "build/sky130"
        self.io_in = Signal(io_count)
        self.io_out = Signal(io_count)
        self.io_oeb = Signal(io_count)
        self.extra_files = set()
        self.chipflow_context = "silicon"
        self.core_size = core_size

    def add_file(self, filename, content):
        self.extra_files.add(filename)

    def connect_io(self, m, rec, name):
        pins = {}
        inputs = set()
        outputs = set()
        for field, width, _ in rec.layout:
            if "_" in field:
                pin, _, d = field.rpartition('_')
            else:
                pin = ""
                d = field
            if d == "oe":
                continue
            pins[pin] = width
            if d == "i":
                inputs.add(pin)
            else:
                assert d == "o"
                outputs.add(pin)
        for pin, width in sorted(pins.items(), key=lambda x: x[0]):
            for i in range(width):
                ext_name = f"{name}_{pin}{i}" if width > 1 else f"{name}_{pin}"
                prefix = "" if pin == "" else f"{pin}_"
                assert ext_name in self.pin_map, ext_name
                idx = self.pin_map[ext_name]
                if pin in inputs:
                    m.d.comb += getattr(rec, f"{prefix}i")[i].eq(self.io_in[idx])
                out = 0
                oeb = 1
                if pin in outputs:
                    out = getattr(rec, f"{prefix}o")[i]
                    if hasattr(rec, f"{prefix}oe"):
                        bus_oe = getattr(rec, f"{prefix}oe")
                        if len(bus_oe) == 1:
                            oeb = ~bus_oe[0]
                        else:
                            assert len(bus_oe) == width
                            oeb = ~bus_oe[i]
                    else:
                        oeb = 0

                m.submodules += Instance(
                    "buf_x1",
                    i_i=out,
                    o_q=self.io_out[idx],
                    a_keep=True
                )
                m.submodules += Instance(
                    "buf_x1",
                    i_i=oeb,
                    o_q=self.io_oeb[idx],
                    a_keep=True
                )

    def build(self, e):
        rtlil_file = "build/my_design.rtlil"

        # Ensure build dir exists
        Path(self.build_dir).mkdir(parents=True, exist_ok=True)
        top_name = "user_project_core_mpw5"
        output = rtlil.convert(e, name=top_name, ports=[self.io_out, self.io_oeb, self.io_in], platform=self)

        top_rtlil = Path(self.build_dir) / f"{top_name}.il"
        with open(top_rtlil, "w") as f:
            f.write(output)

        top_ys = Path(self.build_dir) / "sky130.ys"
        with open(top_ys, "w") as f:
            for extra in sorted(self.extra_files):
                if extra.endswith(".il"):
                    print(f"read_rtlil {extra}", file=f)
                else:
                    print(f"read_verilog -defer {extra}", file=f)
            print(f"read_ilang {top_rtlil}", file=f)
            print(f"hierarchy -top {top_name}", file=f)
            print(f"hierarchy -generate buf_x1 i:i o:q", file=f)
            print(f"write_rtlil {rtlil_file}", file=f)

        subprocess.run(["yowasp-yosys", "-ql", Path(self.build_dir) / "synth.log", top_ys], check=True)
