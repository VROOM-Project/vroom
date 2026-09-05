#!/usr/bin/env python3
"""Check whether the truck loading rules of the waste transport problem
(see docs/waste_transport_problem.md) can be expressed with VROOM
linear capacities, and validate a candidate encoding.

VROOM checks capacity as: sum of the amount vectors of everything on
board <= vehicle capacity vector, component-wise. A set of allowed
loads is therefore only expressible exactly if it is "integrally
convex": every integer point of the convex hull of the allowed loads
must itself be allowed. This script

1. builds the set of allowed loads from the rule lists (a rule is a
   maximal allowed combination; anything smaller is allowed too);
2. reports integer points of the convex hull that are NOT allowed
   (witnesses that no exact linear encoding exists) and which pairs
   of rules conflict;
3. evaluates a candidate encoding (amount vector per container kind,
   capacity vector per truck configuration) and reports what it gets
   wrong in both directions: forbidden loads it would accept (unsafe)
   and allowed loads it would reject (lost).

Requires numpy and scipy (for the convex hull membership LP).

Usage: python scripts/waste_capacity_check.py [--rules strict|assumed]
"""

import argparse
import itertools
import sys
from collections import Counter

import numpy as np
from scipy.optimize import linprog

# Container kinds: e = empty, f = full, number = size in m3.
KINDS = ["e2", "e6", "e12", "e20", "e40", "f2", "f6", "f12", "f20", "f40"]


def load(spec):
    """Parse '2f6 + 1e2' into a count vector over KINDS."""
    counts = Counter()
    for part in spec.replace(" ", "").split("+"):
        if not part:
            continue
        i = 0
        while part[i].isdigit():
            i += 1
        n = int(part[:i]) if i else 1
        kind = part[i:]
        if kind not in KINDS:
            raise ValueError(f"unknown container kind {kind!r} in {spec!r}")
        counts[kind] += n
    return tuple(counts[k] for k in KINDS)


def fmt(vec):
    parts = [f"{n}{k}" for n, k in zip(vec, KINDS) if n]
    return " + ".join(parts) if parts else "(empty)"


# ---------------------------------------------------------------------------
# Rules: maximal allowed loads per truck configuration.
# ---------------------------------------------------------------------------

RULES_STRICT = {
    "small": ["3e2", "1f2"],
    "multiban": [
        # empties only
        "6e2",
        "6e6",
        "1e12",
        "1e12 + 3e6",
        # full only
        "2f2",
        "2f6",
        "1f12",
        # mixed
        "1f6 + 3e2",
        "2f6 + 1e2",
        "1f6 + 3e6",
        "1f12 + 1e2",
    ],
    "poliban": ["1e20", "1f20", "1e40", "1f40"],
}

# Assumption needed for an exact linear encoding (open question 1 of
# the problem document): container kinds in the same class below are
# interchangeable for the loading rules. If a load is allowed, so is
# any load obtained by swapping kinds within a class (e.g. "6e6"
# allowed implies "4e2 + 2e6" allowed; "1f6 + 3e2" implies
# "1f2 + 3e2"). Keep this in sync with the analysis document.
INTERCHANGEABLE = [
    {"e2", "e6"},  # small empties nest/stack the same way
    {"f2", "f6"},  # a full 2 takes a bed position like a full 6
]


def interchange_closure(allowed, truck):
    """Close a set of loads under swaps within INTERCHANGEABLE classes,
    restricted to the kinds the truck can carry at all."""
    idx = {k: i for i, k in enumerate(KINDS)}
    classes = [sorted(idx[k] for k in c & COMPATIBLE[truck]) for c in INTERCHANGEABLE]
    classes = [c for c in classes if len(c) > 1]
    out = set()
    for x in allowed:
        # For each class, redistribute the class total over its kinds.
        options = []
        for c in classes:
            total = sum(x[i] for i in c)
            options.append([
                dict(zip(c, split))
                for split in itertools.product(range(total + 1), repeat=len(c))
                if sum(split) == total
            ])
        for combo in itertools.product(*options):
            y = list(x)
            for assign in combo:
                for i, v in assign.items():
                    y[i] = v
            out.add(tuple(y))
    return out


def rules(variant):
    """Return {truck: list of maximal allowed loads} for a rule variant."""
    out = {}
    for truck, specs in RULES_STRICT.items():
        maxima = [load(s) for s in specs]
        if variant == "assumed":
            maxima = maximal_elements(interchange_closure(down_closure(maxima), truck))
        out[truck] = maxima
    return out


# ---------------------------------------------------------------------------
# Candidate encoding.
# ---------------------------------------------------------------------------
# Dimensions of the amount vector:
#   U  "stack units": how much of the bed/stack height an item uses
#   C  "bed positions": floor positions used by fulls and by a 12
#   P  "poliban slot": one hook-lift container at a time
DIMS = ["U", "C", "P"]

ENCODING = {
    "e2": (2, 0, 0),
    "e6": (2, 0, 0),
    "e12": (6, 2, 0),
    "e20": (0, 0, 1),
    "e40": (0, 0, 1),
    "f2": (5, 1, 0),
    "f6": (5, 1, 0),
    "f12": (10, 2, 0),
    "f20": (0, 0, 1),
    "f40": (0, 0, 1),
}

# Capacity per truck configuration. Container/truck type compatibility
# is handled with VROOM skills, not with capacity, so e.g. the small
# truck does not need to reject a 6 m3 through capacity.
CAPACITY = {
    "small": (6, 1, 0),
    "multiban": (12, 2, 0),
    "poliban": (0, 0, 1),
}

# Which container kinds each truck type may carry at all (skills).
COMPATIBLE = {
    "small": {"e2", "f2"},
    "multiban": {"e2", "e6", "e12", "f2", "f6", "f12"},
    "poliban": {"e20", "f20", "e40", "f40"},
}


# ---------------------------------------------------------------------------
# Set machinery.
# ---------------------------------------------------------------------------


def box(maxima):
    bounds = [max(m[i] for m in maxima) for i in range(len(KINDS))]
    return itertools.product(*(range(b + 1) for b in bounds))


def dominated(x, m):
    return all(a <= b for a, b in zip(x, m))


def down_closure(maxima):
    return {x for x in box(maxima) if any(dominated(x, m) for m in maxima)}


def maximal_elements(points):
    pts = list(points)
    return sorted(
        p for p in pts if not any(q != p and dominated(p, q) for q in pts)
    )


def in_convex_hull(x, generators):
    """LP: is x a convex combination of the generators?"""
    g = np.array(generators, dtype=float).T  # dims x n
    n = g.shape[1]
    a_eq = np.vstack([g, np.ones((1, n))])
    b_eq = np.concatenate([np.array(x, dtype=float), [1.0]])
    res = linprog(np.zeros(n), A_eq=a_eq, b_eq=b_eq, bounds=(0, None), method="highs")
    return res.status == 0


def hull_witnesses(maxima, allowed):
    """Integer points of conv(allowed) that are not allowed."""
    gens = list(allowed)
    return sorted(x for x in box(maxima) if x not in allowed and in_convex_hull(x, gens))


def pair_conflicts(maxima, allowed):
    """Pairs of rules whose segment contains a forbidden integer point."""
    out = []
    for a, b in itertools.combinations(maxima, 2):
        for k in range(1, 4):
            # Look at points at 1/2, 1/3, 2/3 of the segment.
            for num in range(1, k):
                p = tuple((num * ai + (k - num) * bi) for ai, bi in zip(a, b))
                if all(v % k == 0 for v in p):
                    p = tuple(v // k for v in p)
                    if p not in allowed:
                        out.append((a, b, p))
                        break
            else:
                continue
            break
    return out


def encoded(x):
    total = [0] * len(DIMS)
    for n, k in zip(x, KINDS):
        for d, v in enumerate(ENCODING[k]):
            total[d] += n * v
    return tuple(total)


def accepted_by_encoding(x, truck):
    if any(n and k not in COMPATIBLE[truck] for n, k in zip(x, KINDS)):
        return False
    return all(a <= b for a, b in zip(encoded(x), CAPACITY[truck]))


# ---------------------------------------------------------------------------
# Report.
# ---------------------------------------------------------------------------


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--rules", choices=["strict", "assumed"], default="strict")
    args = parser.parse_args()

    all_ok = True
    for truck, maxima in rules(args.rules).items():
        allowed = down_closure(maxima)
        print(f"== {truck} ({args.rules} rules): {len(maxima)} rules, "
              f"{len(allowed)} allowed loads (incl. empty)")

        witnesses = hull_witnesses(maxima, allowed)
        if witnesses:
            all_ok = False
            print(f"  NOT linearly representable: {len(witnesses)} forbidden "
                  "loads lie inside the convex hull of the allowed ones, e.g.")
            for w in witnesses[:10]:
                print(f"    - {fmt(w)}")
            conflicts = pair_conflicts(maxima, allowed)
            if conflicts:
                print("  conflicting rule pairs (segment crosses a forbidden load):")
                for a, b, p in conflicts:
                    print(f"    - [{fmt(a)}] vs [{fmt(b)}]  ->  {fmt(p)}")
        else:
            print("  linearly representable (all hull integer points allowed)")

        # Evaluate the candidate encoding on a box large enough to see
        # everything the encoding accepts.
        enc_box_max = [tuple(max(m[i] for m in maxima) + 1 if KINDS[i] in COMPATIBLE[truck] else 0
                             for i in range(len(KINDS)))]
        accepted = {x for x in box(enc_box_max) if accepted_by_encoding(x, truck)}
        unsafe = accepted - allowed
        lost = allowed - accepted
        print(f"  candidate encoding: accepts {len(accepted)} loads")
        if unsafe:
            all_ok = False
            print(f"  UNSAFE: accepts {len(unsafe)} forbidden loads, maximal ones:")
            for m in maximal_elements(unsafe):
                print(f"    - {fmt(m)}  (encoded {encoded(m)})")
        if lost:
            all_ok = False
            print(f"  LOST: rejects {len(lost)} allowed loads, maximal ones:")
            for m in maximal_elements(lost):
                print(f"    - {fmt(m)}  (encoded {encoded(m)})")
        if not unsafe and not lost:
            print("  candidate encoding matches the rules exactly")
        print()

    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())
