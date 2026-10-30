#!/usr/bin/env python3
"""
Aggregate soft-skills benchmark results into markdown tables.

Reads the TSV produced by scripts/benchmark_soft_skills.sh plus the
per-run solution JSONs it references, and prints one markdown table
per instance size, comparing the baseline (no soft-skills fields)
against the soft-skills-enabled variant at each coverage level.

Usage:

    python3 benchmarks/aggregate_results.py benchmarks/results/results.tsv
"""

import json
import os
import statistics
import sys


def load_rows(tsv_path):
    with open(tsv_path) as fh:
        header = fh.readline().rstrip("\n").split("\t")
        rows = []
        for line in fh:
            fields = line.rstrip("\n").split("\t")
            row = dict(zip(header, fields))
            # The driver may emit locale-dependent decimal separators.
            row["wall_s"] = row["wall_s"].replace(",", ".")
            rows.append(row)
    return rows


def solution_metrics(sol_path):
    with open(sol_path) as fh:
        sol = json.load(fh)
    if sol.get("code", 0) != 0:
        return None
    summary = sol["summary"]
    pref = summary.get("soft_skill_preference")
    return {
        "code": summary["cost"],
        "distance_km": summary.get("distance", 0) / 1000.0,
        "routes": summary["routes"],
        "unassigned": summary["unassigned"],
        "eligible": pref["eligible"] if pref else 0,
        "satisfied": pref["satisfied"] if pref else 0,
        "unsatisfied": pref["unsatisfied"] if pref else 0,
    }


def p95(samples):
    ordered = sorted(samples)
    idx = min(len(ordered) - 1, max(0, round(0.95 * (len(ordered) - 1))))
    return ordered[idx]


def aggregate(tsv_path):
    rows = load_rows(tsv_path)
    results_dir = os.path.dirname(os.path.abspath(tsv_path))
    cells = {}
    for row in rows:
        key = (row["size"], int(row["coverage"]), row["variant"])
        cells.setdefault(key, []).append(row)

    lines = []
    for size in ("300x20", "600x40"):
        lines.append(f"### {size} ({size.split('x')[0]} shipments, "
                     f"{size.split('x')[1]} vehicles)\n")
        lines.append("| Coverage | Variant | Runs | Median wall (s) | "
                     "p95 wall (s) | Peak RSS (MiB) | Cost | Distance (km) "
                     "| Unassigned | Pref. satisfied / eligible |")
        lines.append("| --- | --- | --- | --- | --- | --- | --- | --- "
                     "| --- | --- |")
        for cov in (0, 25, 50, 100):
            for variant in ("baseline", "soft"):
                cell = cells.get((size, cov, variant), [])
                if not cell:
                    continue
                walls = [float(r["wall_s"]) for r in cell]
                rss = [int(r["max_rss_kib"]) for r in cell]
                metrics = [solution_metrics(os.path.join(results_dir,
                                                         r["file"]))
                           for r in cell]
                metrics = [m for m in metrics if m]
                if not metrics:
                    continue
                med = lambda xs: statistics.median(xs)  # noqa: E731
                elig = med([m["eligible"] for m in metrics])
                sat = med([m["satisfied"] for m in metrics])
                pref_txt = (f"{sat:g} / {elig:g}"
                            if elig else ("n/a" if variant == "baseline"
                                          else "0 / 0"))
                lines.append(
                    f"| {cov}% | {variant} | {len(cell)} | "
                    f"{med(walls):.2f} | {p95(walls):.2f} | "
                    f"{med(rss) / 1024:.0f} | {med([m['code'] for m in metrics]):g} | "
                    f"{med([m['distance_km'] for m in metrics]):.1f} | "
                    f"{med([m['unassigned'] for m in metrics]):g} | "
                    f"{pref_txt} |")
        lines.append("")
    return "\n".join(lines)


if __name__ == "__main__":
    path = sys.argv[1] if len(sys.argv) > 1 else \
        "benchmarks/results/results.tsv"
    print(aggregate(path))
