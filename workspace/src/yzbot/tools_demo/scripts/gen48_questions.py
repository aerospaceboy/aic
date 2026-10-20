#!/usr/bin/env python3
"""Print the 48 legal task questions, one per line as '<tag> <question>'.

Both sentence orders are emitted per (zone pair, count split), giving
2 x 4 x 6 = 48 tasks. The tag reads <n><colorletter><zone>_<n><colorletter>
<zone> in sentence order, matching the pick-order policy table keys
(e.g. 1bA_4rB = 'Move 1 blue cube to zone A and 4 red cubes to zone B.').
"""
SPLITS = ((1, 4), (2, 3), (3, 2), (4, 1))
ZONE_PAIRS = (("A", "B"), ("A", "C"), ("B", "A"), ("B", "C"),
              ("C", "A"), ("C", "B"))


def plural(count, color):
    return "%d %s cube%s" % (count, color, "" if count == 1 else "s")


def main():
    lines = []
    for zones in ZONE_PAIRS:
        for red, blue in SPLITS:
            rz, bz = zones
            for order in ((("red", red, rz), ("blue", blue, bz)),
                          (("blue", blue, bz), ("red", red, rz))):
                first, second = order
                tag = "%d%s%s_%d%s%s" % (
                    first[1], first[0][0], first[2],
                    second[1], second[0][0], second[2])
                question = "Move %s to zone %s and %s to zone %s." % (
                    plural(first[1], first[0]), first[2],
                    plural(second[1], second[0]), second[2])
                lines.append("%s %s" % (tag, question))
    print("\n".join(lines))


if __name__ == "__main__":
    main()
