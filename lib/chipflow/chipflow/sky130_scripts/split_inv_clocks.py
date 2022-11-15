import CRL, Hurricane as Hur, Katana, Etesian, Anabatic, Cfg
from Hurricane import Box, Instance, Net
from helpers import u, l, setNdaTopDir
from helpers.overlay import CfgCache, UpdateSession


"""
This script is used to split inverters driving clocks that have been merged into one into local inverters for each FF,
to reduce the skew associated with long-ish routing of the inverted clock
"""
def split_inv_clocks(cell):
    with UpdateSession():
        todo_clks = []
        for inst in cell.getInstances():
            if not inst.getMasterCell().getName().startswith("inv_"):
                continue
            nq = inst.getPlug(inst.getMasterCell().getNet("nq")).getNet()
            if nq is None:
                continue
            inv_clks = []
            for plug in nq.getPlugs():
                if plug.getMasterNet().getName() != "ck":
                    continue
                inv_clks.append(plug)
            if len(inv_clks) > 0:
                todo_clks.append((inst, inv_clks))
        for inst, plugs in todo_clks:
            inp = inst.getPlug(inst.getMasterCell().getNet("i")).getNet()
            assert inp is not None, inst
            for i, plug in enumerate(plugs[1:]): # can retain the original inverter for one sink
                new_inv = Instance.create(cell, f"{inst.getName()}__dup__{i}_", inst.getMasterCell())
                new_inv.getPlug(new_inv.getMasterCell().getNet("i")).setNet(inp)
                new_clk = Net.create(cell, f"{new_inv.getName()}_nq_")
                new_inv.getPlug(new_inv.getMasterCell().getNet("nq")).setNet(new_clk)
                plug.setNet(new_clk)
                print(f"Moved inverted clock {plug} to new driver {new_inv} and net {new_clk}")
