#!/usr/bin/env python3
"""
Synthetic benchmark generator for the soft-skills feature.

This file is part of VROOM.

Generates VROOM-solvable JSON instances combining:

- sizes:  300 shipments / 20 vehicles  and  600 shipments / 40 vehicles
- soft-skill coverage: 0%, 25%, 50% and 100% of shipments carry
  ``soft_skills`` (random assignment from a fixed skill universe)
- variants: ``baseline`` (no soft-skills field anywhere, pure cost
  optimization) and ``soft`` (shipment and vehicle ``soft_skills`` plus
  an ``options.soft_skills`` block)

Both variants of a given (size, coverage) pair are generated from the
same seeded geometry, so cost and distance totals are directly
comparable. All data is synthetic: locations are random points in a
bounding box and duration/distance/cost matrices are Euclidean
(equirectangular) projections of those points. No production data is
used.

Random seeds are fixed per (size, coverage) pair, so re-running this
script reproduces byte-identical files.

Usage:

    python3 benchmarks/generate_synthetic.py [--outdir benchmarks/instances]

Output files (not meant to be committed):

    synth_<shipments>x<vehicles>_cov<coverage>_<variant>.json
"""

import argparse
import json
import math
import os
import random

# Fixed skill universe used for soft-skills assignment.
SKILL_UNIVERSE = list(range(100, 108))  # 8 skills

# Share of soft-skill shipments that get two skills and an explicit
# "any" match mode override (the rest use one skill and the default
# "all" mode). Exercises both match modes in every non-zero coverage.
ANY_MODE_SHARE = 0.30

# Travel speed used to derive durations from Euclidean distances (m/s).
SPEED_MPS = 40_000 / 3600  # 40 km/h

# Hard bounds that keep routes realistic and solvable at -t 1: without
# them a single round-trip route can legally absorb every shipment
# (pickup/delivery interleaving keeps the transit load low), which
# makes the instances degenerate and glacially slow to solve.
VEHICLE_CAPACITY = [26]
VEHICLE_MAX_TASKS = 50

# Bounding box for random locations (synthetic, Stuttgart area).
LON_RANGE = (8.40, 8.80)
LAT_RANGE = (48.95, 49.25)

# Reference latitude for the equirectangular projection.
LAT0 = (LAT_RANGE[0] + LAT_RANGE[1]) / 2

# (shipments, vehicles) pairs to generate, per the plan Q15 decision.
SIZES = [(300, 20), (600, 40)]
COVERAGES = [0, 25, 50, 100]

# Base seed for reproducibility.
BASE_SEED = 20261030


def seeded(size_idx: int, coverage: int) -> random.Random:
    return random.Random(BASE_SEED + 1000 * size_idx + coverage)


def draw_points(rng: random.Random, count: int):
    return [(rng.uniform(*LON_RANGE), rng.uniform(*LAT_RANGE))
            for _ in range(count)]


def euclidean_meters(points):
    """Full symmetric Euclidean distance matrix in meters."""
    kx = 111_320 * math.cos(math.radians(LAT0))
    ky = 110_540
    n = len(points)
    dist = [[0] * n for _ in range(n)]
    for i in range(n):
        lon_i, lat_i = points[i]
        row = dist[i]
        for j in range(i + 1, n):
            lon_j, lat_j = points[j]
            dx = (lon_j - lon_i) * kx
            dy = (lat_j - lat_i) * ky
            d = int(round(math.hypot(dx, dy)))
            row[j] = d
            dist[j][i] = d
    return dist


def scale_matrix(source, factor):
    """Scale an integer matrix, rounding to integers (diagonal stays 0)."""
    return [[int(round(v * factor)) for v in row] for row in source]


def generate_instance(shipments: int, vehicles: int, coverage: int,
                      variant: str) -> dict:
    rng = seeded(SIZES.index((shipments, vehicles)), coverage)

    # Geometry: one point per pickup, one per delivery, one per vehicle
    # (round trips: vehicle start == end).
    points = draw_points(rng, 2 * shipments + vehicles)

    # Soft-skill assignment on shipments (same rng order in both
    # variants, then dropped for baseline).
    soft_shipments = set(
        rng.sample(range(shipments), shipments * coverage // 100))
    shipment_skills = {}
    shipment_modes = {}
    for s in sorted(soft_shipments):
        two = rng.random() < ANY_MODE_SHARE
        skills = sorted(rng.sample(SKILL_UNIVERSE, 2 if two else 1))
        shipment_skills[s] = skills
        if two:
            shipment_modes[s] = "any"

    vehicle_skills = [sorted(rng.sample(SKILL_UNIVERSE, 3))
                      for _ in range(vehicles)]
    shipment_amounts = [rng.randint(1, 2) for _ in range(shipments)]

    # Vehicle start/end indices come after all shipment endpoints.
    veh_base = 2 * shipments

    ship_list = []
    for s in range(shipments):
        pickup = {
            "id": 10_000 + 2 * s,
            "location_index": 2 * s,
            "location": [round(points[2 * s][0], 6),
                         round(points[2 * s][1], 6)],
            "service": 300,
        }
        delivery = {
            "id": 10_000 + 2 * s + 1,
            "location_index": 2 * s + 1,
            "location": [round(points[2 * s + 1][0], 6),
                         round(points[2 * s + 1][1], 6)],
            "service": 300,
        }
        shipment = {
            "pickup": pickup,
            "delivery": delivery,
            "amount": [shipment_amounts[s]],
        }
        if variant == "soft" and s in soft_shipments:
            shipment["soft_skills"] = shipment_skills[s]
            if s in shipment_modes:
                shipment["soft_skill_match_mode"] = shipment_modes[s]
        ship_list.append(shipment)

    veh_list = []
    for v in range(vehicles):
        idx = veh_base + v
        vehicle = {
            "id": v + 1,
            "start_index": idx,
            "end_index": idx,
            "start": [round(points[idx][0], 6), round(points[idx][1], 6)],
            "capacity": VEHICLE_CAPACITY,
            "max_tasks": VEHICLE_MAX_TASKS,
        }
        if variant == "soft":
            vehicle["soft_skills"] = vehicle_skills[v]
        veh_list.append(vehicle)

    distances = euclidean_meters(points)
    durations = scale_matrix(distances, 1.0 / SPEED_MPS)
    costs = [row[:] for row in durations]  # uniform speed: cost == duration

    instance = {
        "vehicles": veh_list,
        "shipments": ship_list,
        "matrices": {
            "car": {
                "durations": durations,
                "distances": distances,
                "costs": costs,
            },
        },
    }
    if variant == "soft" and coverage > 0:
        instance["options"] = {
            "soft_skills": {
                "default_match_mode": "all",
                "threshold_mode": "cost",
                "max_cost_factor": 1.3,
            },
        }
    return instance


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--outdir", default="benchmarks/instances")
    args = parser.parse_args()
    os.makedirs(args.outdir, exist_ok=True)

    for shipments, vehicles in SIZES:
        for coverage in COVERAGES:
            for variant in ("baseline", "soft"):
                inst = generate_instance(shipments, vehicles, coverage,
                                         variant)
                path = os.path.join(
                    args.outdir,
                    f"synth_{shipments}x{vehicles}_cov{coverage}_"
                    f"{variant}.json")
                with open(path, "w") as fh:
                    json.dump(inst, fh, separators=(",", ":"))
                size_kb = os.path.getsize(path) // 1024
                print(f"wrote {path} ({size_kb} KiB)")


if __name__ == "__main__":
    main()
