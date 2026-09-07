#!/usr/bin/env python3
"""Print VROOM `capacities` arrays generated from docs/waste_rules.json.

Amount components are container kinds in this order: e<size> for every
size, then f<size> for every size (sizes as listed in the rules file).
Materials travel in a full container of a standard size, so they need
no component of their own. One capacity vector is emitted per rule of
a truck type; a load is valid for VROOM when it fits at least one of
them.

With --chico, the vectors of a truck fitted with a chico are printed
instead: any (extra_loads + 1) rules of the truck type together, one
per bed, duplicates and dominated vectors dropped.

Usage:
    python scripts/waste_rules_to_capacities.py [--truck TYPE] [--chico]
"""

import argparse
import json
import os
import sys

RULES_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          "..", "docs", "waste_rules.json")


def kinds_of(rules):
    sizes = rules["sizes"]
    return [f"e{s}" for s in sizes] + [f"f{s}" for s in sizes]


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


def capacities(rules, truck):
    kinds = kinds_of(rules)
    return [parse_load(r, kinds) for r in rules["trucks"][truck]["rules"]]


def maximal(vectors):
    """Drop duplicates and vectors dominated component-wise by another."""
    out = []
    for v in vectors:
        if any(all(a <= b for a, b in zip(v, o)) for o in out):
            continue
        out = [o for o in out if not all(a <= b for a, b in zip(o, v))]
        out.append(v)
    return out


def chico_capacities(rules, chico):
    """Sum of any (extra_loads + 1) rules of the truck the chico fits."""
    spec = rules["chicos"][chico]
    base = capacities(rules, spec["attaches_to"])
    sums = [list(v) for v in base]
    for _ in range(int(spec.get("extra_loads", 2))):
        sums = maximal([[a + b for a, b in zip(s, v)] for s in sums for v in base])
    return sums


def chicos_of(rules):
    return [k for k in rules.get("chicos", {}) if not k.startswith("_")]


def main(argv):
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--truck", help="only this truck type (or chico type with --chico)")
    parser.add_argument("--chico", action="store_true",
                        help="print the vectors of trucks fitted with a chico")
    parser.add_argument("--rules", default=RULES_FILE, help="rules file")
    args = parser.parse_args(argv)

    with open(args.rules, encoding="utf-8") as f:
        rules = json.load(f)

    kinds = kinds_of(rules)
    print(f"// components: {kinds}")
    if args.chico:
        for chico in [args.truck] if args.truck else chicos_of(rules):
            spec = rules["chicos"][chico]
            caps = chico_capacities(rules, chico)
            print(f'// {spec["attaches_to"]} + chico {chico}: any '
                  f'{int(spec.get("extra_loads", 2)) + 1} rules together, '
                  f'{len(caps)} vectors')
            print(f'"capacities": {json.dumps(caps)}')
        return 0
    trucks = [args.truck] if args.truck else list(rules["trucks"])
    for truck in trucks:
        caps = capacities(rules, truck)
        print(f'// {truck}: {" | ".join(rules["trucks"][truck]["rules"])}')
        print(f'"capacities": {json.dumps(caps)}')
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
