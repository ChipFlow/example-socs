# SPDX-License-Identifier: BSD-2-Clause
import argparse
import sys
import os
from pathlib import Path

from amaranth import *
from amaranth.back import rtlil


class SimPlatform():
    def __init__(self):
        self.chipflow_context = "sim"
        self.build_dir = os.environ['BUILD_DIR']
        self.extra_files = set()
        self.clk = Signal()
        self.rst = Signal()
        self.buttons = Signal(2)
        self.sim_boxes = dict()

    def add_file(self, filename, content):
        self.extra_files.add(filename)

    def add_model(self, inst_type, rec, edge_det=[]):
        conns = dict(a_keep=True)

        def is_model_out(pin):
            assert field.endswith("_o") or field.endswith("_oe") or field.endswith("_i"), field
            return field.endswith("_i")
        for field, _, _ in rec.layout:
            if is_model_out(field):
                conns[f"o_{field}"] = getattr(rec, field)
            else:
                conns[f"i_{field}"] = getattr(rec, field)
        if inst_type not in self.sim_boxes:
            box = 'attribute \\blackbox 1\n'
            box += 'attribute \\cxxrtl_blackbox 1\n'
            box += 'attribute \\keep 1\n'
            box += f'module \\{inst_type}\n'
            for i, (field, width, _) in enumerate(rec.layout):
                if field in edge_det:
                    box += '  attribute \\cxxrtl_edge "a"\n'
                box += f'  wire width {width} {"output" if is_model_out(field) else "input"} {i} \\{field}\n'
            box += 'end\n\n'
            self.sim_boxes[inst_type] = box
        return Instance(inst_type, **conns)

    def add_monitor(self, inst_type, rec):
        conns = dict(i_clk=ClockSignal(), a_keep=True)
        for field, width, _ in rec.layout:
            conns[f'i_{field}'] = getattr(rec, field)
        if inst_type not in self.sim_boxes:
            box = 'attribute \\blackbox 1\n'
            box += 'attribute \\cxxrtl_blackbox 1\n'
            box += 'attribute \\keep 1\n'
            box += f'module \\{inst_type}\n'
            box += '  attribute \\cxxrtl_edge "a"\n'
            box += '  wire width 1 input 0 \\clk\n'
            for i, (field, width, _) in enumerate(rec.layout):
                box += f'  wire width {width} input {i+1} \\{field}\n'
            box += 'end\n\n'
            self.sim_boxes[inst_type] = box
        return Instance(inst_type, **conns)

    def build(self, e):
        Path(self.build_dir).mkdir(parents=True, exist_ok=True)

        output = rtlil.convert(e, name="sim_top", ports=[self.clk, self.rst, self.buttons], platform=self)

        top_rtlil = Path(self.build_dir) / "sim_soc.il"
        with open(top_rtlil, "w") as f:
            for box_content in self.sim_boxes.values():
                f.write(box_content)
            f.write(output)
        top_ys = Path(self.build_dir) / "sim_soc.ys"
        with open(top_ys, "w") as f:
            for extra in sorted(self.extra_files):
                if extra.endswith(".il"):
                    print(f"read_rtlil {extra}", file=f)
                else:
                    print(f"read_verilog -defer {extra}", file=f)
            print("read_ilang sim_soc.il", file=f)
            print("hierarchy -top sim_top", file=f)
            print("write_cxxrtl -g1 -header sim_soc.cc", file=f)
