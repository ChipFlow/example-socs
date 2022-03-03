import sys
# Update a LEF file with layer thicknesses and resistances
# From the original PDK
data = {
    "li": (0.1, 12.2),
    "m1": (0.35, 0.125),
    "m2": (0.35, 0.125),
    "m3": (0.8, 0.047),
    "m4": (0.8, 0.047),
    "m5": (1.2, 0.047),
}

def main():
    curr_layer = None
    with open(sys.argv[1], "r") as f0:
        with open(sys.argv[2], "w") as f1:
            for line in f0:
                f1.write(line)
                sl = line.strip().split(" ")
                if sl[0] == "LAYER":
                    curr_layer = sl[1]
                elif sl[0] == "SPACING" and curr_layer is not None and curr_layer in data:
                    th, r = data[curr_layer]
                    print(f"   THICKNESS {th} ;", file=f1)
                    print(f"   RESISTANCE RPERSQ {r} ;", file=f1)
                    curr_layer = None

if __name__ == '__main__':
    main()
