#!/usr/bin/env python3
"""Print VROOM `capacities` arrays generated from docs/waste_rules.json.

Amount components are container kinds in this order: e<size> for every
size, then f<size> for every size (sizes as listed in the rules file),
then optionally `mat` (a full truck of materials) with --materials.
One capacity vector is emitted per rule of a truck type; a load is
valid for VROOM when it fits at least one of them.

Usage:
    python scripts/waste_rules_to_capacities.py [--materials] [--truck TYPE]
"""

import argparse
import json
import os
import sys

RULES_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          "..", "docs", "waste_rules.json")


def kinds_of(rules, materials):
    sizes = rules["sizes"]
    kinds = [f"e{s}" for s in sizes] + [f"f{s}" for s in sizes]
    if materials:
        kinds.append("mat")
    return kinds


def parse_load(spec, kinds):
    vec = [0] * len(kinds)
    for part in spec.replace(" ", "").split("+"):
        if not part:
            continue
        i = 0
        while i < len(part) and part[i].isdigit():
            i += 1
        n = int(part[:i]) if i else 1
        kind = part[i:]
        if kind not in kinds:
            raise ValueError(f"unknown container kind {kind!r} in rule {spec!r}")
        vec[kinds.index(kind)] += n
    return vec


def capacities(rules, truck, materials):
    kinds = kinds_of(rules, materials)
    caps = [parse_load(r, kinds) for r in rules["trucks"][truck]["rules"]]
    if materials:
        mat = [0] * len(kinds)
        mat[kinds.index("mat")] = 1
        caps.append(mat)
    return caps


def main(argv):
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--materials", action="store_true",
                        help="append the materials component and vector")
    parser.add_argument("--truck", help="only this truck type")
    parser.add_argument("--rules", default=RULES_FILE, help="rules file")
    args = parser.parse_args(argv)

    with open(args.rules, encoding="utf-8") as f:
        rules = json.load(f)

    kinds = kinds_of(rules, args.materials)
    print(f"// components: {kinds}")
    trucks = [args.truck] if args.truck else list(rules["trucks"])
    for truck in trucks:
        caps = capacities(rules, truck, args.materials)
        print(f'// {truck}: {" | ".join(rules["trucks"][truck]["rules"])}')
        print(f'"capacities": {json.dumps(caps)}')
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
