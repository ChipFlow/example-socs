import argparse, sys, os, subprocess
from pathlib import Path

from amaranth import *
from amaranth.back import rtlil

class Sky130Platform():
    def __init__(self, pin_map, io_count=38, core_size=(280*10.0, 330*10.0)):
        self.pin_map = pin_map
        self.build_dir = "build/sky130"
        self.pdk_dir = os.environ["PDK_DIR"] if "PDK_DIR" in os.environ else os.environ["HOME"] + "/mpw4/thirdparty/open_pdk/C4M.Sky130"
        self.io_in = Signal(io_count)
        self.io_out = Signal(io_count)
        self.io_oeb = Signal(io_count)
        self.extra_files = set()
        self.is_sky130 = True
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
        for pin, width in sorted(pins.items(), key=lambda x:x[0]):
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

                m.submodules += Instance("buf_x1",
                    i_i=out,
                    o_q=self.io_out[idx],
                    a_keep=True
                )
                m.submodules += Instance("buf_x1",
                    i_i=oeb,
                    o_q=self.io_oeb[idx],
                    a_keep=True
                )

    def do_fixup(self):
        # TODO: make this importable
        script = f"{self.pdk_dir}/libs.tech/klayout/bin/conv_c4msky130_to_sky130.py"
        subprocess.run(["python3", script,  "user_project_wrapper.gds", "user_project_wrapper_fixedup.gds"], check=True)

    def run_pnr(self, top_name):
        os.chdir(self.build_dir)
        import CRL, Hurricane as Hur, Katana, Etesian, Anabatic, Cfg
        from Hurricane import DataBase, Transformation, Box, Instance
        from helpers import u, l, setNdaTopDir
        from helpers.overlay import CfgCache, UpdateSession

        sys.path.append(f"{self.pdk_dir}/libs.tech/coriolis/techno/etc/coriolis2")
        from node130 import sky130 as tech
        tech.setup()
        tech.StdCellLib_setup()

        from plugins.alpha.block.block         import Block
        from plugins.alpha.block.configuration import IoPin, GaugeConf
        from plugins.alpha.block.spares        import Spares
        from plugins.alpha.chip.configuration  import ChipConf
        from plugins.alpha.chip.chip           import Chip
        from plugins.alpha.core2chip.sky130    import CoreToChip

        cell_name = top_name
        cell = CRL.Blif.load(cell_name)

        from .sky130_scripts.split_inv_clocks import split_inv_clocks
        split_inv_clocks(cell)

        af = CRL.AllianceFramework.get()
        env = af.getEnvironment()
        env.setCLOCK('io_in_from_pad(0)')

        lg5 = af.getRoutingGauge('StdCellLib').getLayerGauge( 5 )
        lg5.setType( CRL.RoutingLayerGauge.PowerSupply )

        conf = ChipConf( cell, ioPins=[], ioPads=[] ) 
        conf.cfg.etesian.bloat               = 'Flexlib'
        conf.cfg.etesian.uniformDensity      = True
        conf.cfg.etesian.aspectRatio         = 1.0
       # etesian.spaceMargin is ignored if the coreSize is directly set.
        conf.cfg.etesian.spaceMargin         = 0.10
        conf.cfg.etesian.antennaGateMaxWL = u(400.0)
        conf.cfg.etesian.antennaDiodeMaxWL = u(800.0)
        conf.cfg.etesian.feedNames = 'tie_diff,tie_diff_w2'
        conf.cfg.anabatic.searchHalo         = 2
        conf.cfg.anabatic.globalIterations   = 20
        conf.cfg.anabatic.topRoutingLayer    = 'm4'
        conf.cfg.katana.hTracksReservedLocal = 25
        conf.cfg.katana.vTracksReservedLocal = 20
        conf.cfg.katana.hTracksReservedMin   = 12
        conf.cfg.katana.vTracksReservedMin   = 10
        conf.cfg.katana.trackFill            = 0
        conf.cfg.katana.runRealignStage      = True
        conf.cfg.katana.dumpMeasures         = True
        conf.cfg.katana.longWireUpReserve1   = 3.0
        conf.cfg.block.spareSide             = u(7*10)
        conf.cfg.chip.minPadSpacing          = u(1.46)
        conf.cfg.chip.supplyRailWidth        = u(20.0)
        conf.cfg.chip.supplyRailPitch        = u(40.0)
        conf.cfg.harness.path                = f"{self.pdk_dir}/../../../resources/user_project_wrapper.def"
        conf.useSpares           = True
        # conf.useClockTree        = True
        # conf.useHFNS             = True
        conf.bColumns            = 2
        conf.bRows               = 2
        conf.chipName            = 'chip'
        conf.coreSize            = ( u(self.core_size[0]), u(self.core_size[1]) )
        conf.useHTree( 'io_in_from_pad(0)', Spares.HEAVY_LEAF_LOAD )

        coreToChip = CoreToChip( conf )
        coreToChip.buildChip()
        chipBuilder = Chip( conf )
        chipBuilder.doChipFloorplan()
        chipBuilder.doPnR()
        chipBuilder.save()
        self.do_fixup()

        # Write LEF and DEF for PEX etc
        Path("export").mkdir(parents=True, exist_ok=True)
        os.chdir("export")
        db = DataBase.getDB()
        rootlib = db.getRootLibrary()
        lib = rootlib.getLibrary("StdCellLib")
        CRL.LefExport.drive(lib, 1)
        CRL.DefExport.drive(conf.corona, 0)
        for cell in lib.getCells():
            if cell.getName() in (conf.corona.getName(), conf.core.getName(), conf.chip.getName()):
                continue
            CRL.DefExport.drive(cell, 0)

    def build(self, e, yowasp=False, gen_rtlil=False, synth=True, pnr=True):
        Path(self.build_dir).mkdir(parents=True, exist_ok=True)
        top_name = "user_project_core_mpw5"
        output = rtlil.convert(e, name=top_name, ports=[self.io_out, self.io_oeb, self.io_in], platform=self)

        top_rtlil = Path(self.build_dir) / f"{top_name}.il"
        with open(top_rtlil, "w") as f:
            f.write(output)

        liberty = self.pdk_dir + "/libs.ref/StdCellLib/liberty/StdCellLib_slow.lib"
        target_delay = 10000 # 100MHz
        max_fanout = 32

        abc_constr = Path(self.build_dir) / "abc_sky130.constr"
        with open(abc_constr, "w") as f:
            print("set_driving_cell inv_x4", file=f)
            print("set_load 1.0", file=f)

        top_ys = Path(self.build_dir) / "sky130.ys"
        if gen_rtlil is not None:
            assert not synth and not pnr
        with open(top_ys, "w") as f:
            if synth:
                print(f"read_liberty -lib {liberty}", file=f)
            for extra in sorted(self.extra_files):
                if extra.endswith(".il"):
                    print(f"read_rtlil {extra}", file=f)
                else:
                    print(f"read_verilog -defer {extra}", file=f)
            print(f"read_ilang {top_rtlil}", file=f)
            if gen_rtlil is not None:
                print(f"hierarchy -top {top_name}", file=f)
                if not synth:
                    # HACK: avoid a dependency on the liberty for just generating rtlil
                    # where we just need a blackbox for the buffer
                    print(f"hierarchy -generate buf_x1 i:i o:q", file=f)
                print(f"write_rtlil {gen_rtlil}", file=f)
            if synth:
                print(f"synth -flatten -top {top_name}", file=f)
                print(f"dfflibmap -liberty {liberty}", file=f)
                print(f"opt", file=f)
                print(f"abc -script +strash;ifraig;scorr;dc2;dretime;strash;&get,-n;&dch,-f;&nf,-D,{target_delay};&put;buffer,-G,1000,-N,{max_fanout};upsize,-D,{target_delay};dnsize,-D,{target_delay};stime,-p -constr {abc_constr} -liberty {liberty}", file=f)
                print(f"setundef -zero", file=f)
                print(f"clean -purge", file=f)
                print(f"setundef -zero", file=f)
                print(f"write_blif {self.build_dir}/{top_name}.blif", file=f)
                print(f"write_json {self.build_dir}/{top_name}.json", file=f)
                print(f"splitnets -ports", file=f) # required for tas
                print(f"write_verilog -noattr {self.build_dir}/{top_name}_syn.v", file=f)
                print(f"stat", file=f)
        if synth or gen_rtlil is not None:
            subprocess.run(["yowasp-yosys" if yowasp else "yosys", "-ql", Path(self.build_dir) / "synth.log", top_ys], check=True)
        if pnr:
            self.run_pnr(top_name)
