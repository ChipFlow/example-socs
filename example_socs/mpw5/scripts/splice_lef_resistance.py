import sys
import os
# Update a LEF file with layer thicknesses and resistances
# From the original PDK

layer_map = {
    "li": "li1",
    "mcon": "mcon",
    "m1": "met1",
    "via": "via",
    "m2": "met2",
    "via2": "via2",
    "m3": "met3",
    "via3": "via3",
    "m4": "met4",
    "via4": "via4",
    "m5": "met5",
}

orig_root = f"{os.environ['HOME']}/openrcx-calibration/tech/sky130B"

def main():
    curr_layer = None
    for corner in ("min", "nom", "max"):
        layer_props = {}
        curr_layer = None
        is_port_via = False
        with open(f"{orig_root}/sky130_fd_sc_hd.{corner}.tlef", "r") as f:
            for line in f:
                sl = line.strip().split(" ")
                if len(sl) == 0:
                    continue
                if sl[0] == "LAYER" and not is_port_via:
                    curr_layer = sl[1]
                    layer_props[curr_layer] = []
                elif sl[0] in ("VIA", "PORT", "VIARULE"):
                    is_port_via = True
                elif sl[0] == "END":
                    is_port_via = False
                    curr_layer = None
                elif curr_layer is not None and sl[0] in ("THICKNESS", "EDGECAPACITANCE", "CAPACITANCE", "RESISTANCE"):
                    layer_props[curr_layer].append(" ".join(sl))
        curr_layer = None
        is_port_via = False
        with open(f"{sys.argv[1]}/export/StdCellLib.lef", "r") as fi:
            with open(f"{sys.argv[1]}/export/StdCellLib.patched.{corner}.lef", "w") as fo:
                for line in fi:
                    sl = line.strip().split(" ")
                    if len(sl) > 0:
                        if sl[0] == "LAYER" and not is_port_via:
                            curr_layer = sl[1]
                        elif sl[0] in ("VIA", "PORT", "VIARULE"):
                            is_port_via = True
                        elif sl[0] == "END":
                            is_port_via = False
                            if curr_layer is not None:
                                if curr_layer in layer_map:
                                    for prop in layer_props[layer_map[curr_layer]]:
                                        print(f"   {prop}", file=fo)
                            curr_layer = None
                    fo.write(line)

if __name__ == '__main__':
    main()
