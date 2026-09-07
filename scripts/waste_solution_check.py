#!/usr/bin/env python3
"""Independent checker for a VROOM solution against its input.

Recomputes the load at every step of every route from the input
(without trusting the "load" arrays of the solution) and verifies:

- capacity: each step load fits the vehicle "capacity" vector or at
  least one of its "capacities" vectors (component-wise);
- loads never go negative;
- shipments: pickup before delivery in the same route, both or none
  assigned;
- skills: a task is only served by a vehicle having all its skills;
- vehicle groups: no more routes than "max_vehicles" for the vehicles
  of each "vehicle_groups" entry;
- every task appears exactly once, either in a route or in
  "unassigned";
- the "load" arrays reported by VROOM match the recomputation when
  present;
- no-go zones, when a zone file is given: no route of a vehicle stops
  in, or drives through, an area closed to its routing profile.

The zone check is the one that can genuinely fail without the solver
being at fault. Zones are not a solver constraint: they are enforced by
making the roads inside them prohibitively slow in the routing data of
the profile they are closed to (docs/no_go_zones.md), which keeps the
map connected but leaves crossing possible where there is no
alternative at all. This is where such a crossing shows up. It needs
the route geometry, so run VROOM with -g.

Exit status is 0 when no violation is found, 1 otherwise. Usage:

    python scripts/waste_solution_check.py input.json solution.json \
                                           [docs/no_go_zones.json]
"""

import json
import sys


def decode_polyline(encoded, precision=5):
    """Google encoded polyline (what OSRM and VROOM return) to [lng, lat]."""
    coords = []
    index = lat = lng = 0
    factor = 10 ** precision
    while index < len(encoded):
        for which in range(2):
            shift = result = 0
            while True:
                byte = ord(encoded[index]) - 63
                index += 1
                result |= (byte & 0x1F) << shift
                shift += 5
                if byte < 0x20:
                    break
            delta = ~(result >> 1) if result & 1 else result >> 1
            if which == 0:
                lat += delta
            else:
                lng += delta
        coords.append([lng / factor, lat / factor])
    return coords


def point_in_ring(ring, lng, lat):
    """Ray casting on a [lng, lat] ring."""
    inside = False
    j = len(ring) - 1
    for i in range(len(ring)):
        xi, yi = ring[i][0], ring[i][1]
        xj, yj = ring[j][0], ring[j][1]
        if (yi > lat) != (yj > lat) and lng < (xj - xi) * (lat - yi) / (yj - yi) + xi:
            inside = not inside
        j = i
    return inside


def segments_cross(a, b, c, d):
    """Whether segment ab properly crosses segment cd."""

    def side(p, q, r):
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])

    d1, d2 = side(c, d, a), side(c, d, b)
    d3, d4 = side(a, b, c), side(a, b, d)
    return ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0))


def path_touches_zone(path, ring):
    """Whether a polyline enters a zone: a vertex inside, or an edge
    crossing the boundary (a straight leg over a small area has no
    vertex inside it)."""
    for lng, lat in path:
        if point_in_ring(ring, lng, lat):
            return True
    for i in range(len(path) - 1):
        for j in range(len(ring)):
            if segments_cross(path[i], path[i + 1], ring[j], ring[(j + 1) % len(ring)]):
                return True
    return False


def check_zones(input_data, solution, zone_config):
    """Routes that stop in or drive through an area closed to them."""
    errors = []
    zones = [
        z for z in zone_config.get("zones", [])
        if z.get("blocked_profiles") and len(z.get("polygon", [])) >= 3
    ]
    if not zones:
        return errors

    vehicles = {v["id"]: v for v in input_data.get("vehicles", [])}
    default_profile = (zone_config.get("vehicle_profiles", {}) or {}).get("default", "car")

    for route in solution.get("routes", []):
        vehicle = vehicles.get(route.get("vehicle"), {})
        profile = vehicle.get("profile", default_profile)
        closed = [z for z in zones if profile in z["blocked_profiles"]]
        if not closed:
            continue

        path = []
        if route.get("geometry"):
            path = decode_polyline(route["geometry"])

        for zone in closed:
            ring = zone["polygon"]
            name = zone.get("name", zone.get("id"))
            for step in route.get("steps", []):
                loc = step.get("location")
                if loc and point_in_ring(ring, loc[0], loc[1]):
                    errors.append(
                        f"vehicle {route.get('vehicle')} (profile {profile}) stops at "
                        f"{step.get('type')} {step.get('id', '')} inside no-go zone "
                        f"{name!r}, which is closed to it"
                    )
            if path and path_touches_zone(path, ring):
                errors.append(
                    f"vehicle {route.get('vehicle')} (profile {profile}) drives through "
                    f"no-go zone {name!r}, which is closed to it: the penalty on those "
                    f"roads was not enough, or its routing data is out of date "
                    f"(scripts/build_zone_graphs.sh)"
                )
        if not path:
            errors.append(
                f"vehicle {route.get('vehicle')} (profile {profile}) has no geometry, "
                f"so only its stops could be checked against the no-go zones: "
                f"re-run VROOM with -g"
            )

    return errors


def fits(load, vectors):
    return any(all(l <= c for l, c in zip(load, vec)) for vec in vectors)


def vehicle_vectors(vehicle, amount_size):
    if "capacities" in vehicle and vehicle["capacities"]:
        return [list(c) for c in vehicle["capacities"]]
    if "capacity" in vehicle:
        return [list(vehicle["capacity"])]
    return [[0] * amount_size]


def check(input_data, solution):
    errors = []
    vehicles = {v["id"]: v for v in input_data.get("vehicles", [])}

    # Task tables: id -> (kind, pickup vector, delivery vector, skills,
    # shipment key or None).
    tasks = {}
    amount_size = 0
    for job in input_data.get("jobs", []):
        delivery = job.get("delivery", job.get("amount", []))
        pickup = job.get("pickup", [])
        amount_size = max(amount_size, len(delivery), len(pickup))
        tasks[("job", job["id"])] = {
            "pickup": pickup,
            "delivery": delivery,
            "skills": set(job.get("skills", [])),
            "shipment": None,
        }
    for s_rank, shipment in enumerate(input_data.get("shipments", [])):
        amount = shipment.get("amount", [])
        amount_size = max(amount_size, len(amount))
        skills = set(shipment.get("skills", []))
        tasks[("pickup", shipment["pickup"]["id"])] = {
            "pickup": amount,
            "delivery": [],
            "skills": skills,
            "shipment": s_rank,
        }
        tasks[("delivery", shipment["delivery"]["id"])] = {
            "pickup": [],
            "delivery": amount,
            "skills": skills,
            "shipment": s_rank,
        }

    def vec(a):
        return list(a) + [0] * (amount_size - len(a))

    seen = {}

    for route in solution.get("routes", []):
        v_id = route["vehicle"]
        if v_id not in vehicles:
            errors.append(f"route for unknown vehicle {v_id}")
            continue
        vehicle = vehicles[v_id]
        vectors = [vec(c) for c in vehicle_vectors(vehicle, amount_size)]
        v_skills = set(vehicle.get("skills", []))
        steps = route.get("steps", [])

        # Initial load: deliveries of single jobs in this route.
        load = [0] * amount_size
        for step in steps:
            if step.get("type") == "job":
                for i, x in enumerate(vec(tasks[("job", step["id"])]["delivery"])):
                    load[i] += x

        pending_pickups = {}
        for rank, step in enumerate(steps):
            s_type = step.get("type")
            key = (s_type, step.get("id"))
            if s_type in ("job", "pickup", "delivery"):
                task = tasks.get(key)
                if task is None:
                    errors.append(f"vehicle {v_id}: unknown {s_type} {step.get('id')}")
                    continue
                if key in seen:
                    errors.append(f"vehicle {v_id}: {s_type} {step['id']} served twice")
                seen[key] = v_id

                if not task["skills"] <= v_skills:
                    errors.append(
                        f"vehicle {v_id}: missing skills for {s_type} {step['id']}"
                    )

                if s_type == "pickup":
                    pending_pickups[task["shipment"]] = rank
                if s_type == "delivery":
                    if task["shipment"] not in pending_pickups:
                        errors.append(
                            f"vehicle {v_id}: delivery {step['id']} before its pickup"
                        )
                    else:
                        del pending_pickups[task["shipment"]]

                for i, x in enumerate(vec(task["pickup"])):
                    load[i] += x
                for i, x in enumerate(vec(task["delivery"])):
                    load[i] -= x

            if any(x < 0 for x in load):
                errors.append(f"vehicle {v_id}: negative load {load} at step {rank}")
            if not fits(load, vectors):
                errors.append(
                    f"vehicle {v_id}: load {load} at step {rank} "
                    f"({s_type} {step.get('id', '')}) fits no capacity vector"
                )
            if "load" in step and list(step["load"]) != load:
                errors.append(
                    f"vehicle {v_id}: reported load {step['load']} at step {rank} "
                    f"differs from recomputed {load}"
                )

        if pending_pickups:
            errors.append(f"vehicle {v_id}: shipments picked up but never delivered")

    for u in solution.get("unassigned", []):
        key = (u.get("type"), u.get("id"))
        if key in seen:
            errors.append(f"{key[0]} {key[1]} both assigned and unassigned")
        seen[key] = None

    for key, task in tasks.items():
        if key not in seen:
            errors.append(f"{key[0]} {key[1]} neither assigned nor unassigned")

    used = {
        r["vehicle"]
        for r in solution.get("routes", [])
        if any(s.get("type") in ("job", "pickup", "delivery") for s in r.get("steps", []))
    }
    for group in input_data.get("vehicle_groups", []):
        members = [v_id for v_id, v in vehicles.items() if group["id"] in v.get("groups", [])]
        n = sum(1 for v_id in members if v_id in used)
        if n > group["max_vehicles"]:
            errors.append(
                f"vehicle group {group['id']}: {n} vehicles used, max {group['max_vehicles']}"
            )
    for s_rank, shipment in enumerate(input_data.get("shipments", [])):
        p = seen.get(("pickup", shipment["pickup"]["id"]))
        d = seen.get(("delivery", shipment["delivery"]["id"]))
        if p != d:
            errors.append(
                f"shipment {shipment['pickup']['id']}/{shipment['delivery']['id']}: "
                f"pickup on vehicle {p}, delivery on vehicle {d}"
            )

    return errors


def main(argv):
    if len(argv) not in (3, 4):
        print(__doc__)
        return 2
    with open(argv[1], encoding="utf-8") as f:
        input_data = json.load(f)
    with open(argv[2], encoding="utf-8") as f:
        solution = json.load(f)

    errors = check(input_data, solution)
    checked = "capacity, precedence, skills and vehicle group constraints"
    if len(argv) == 4:
        with open(argv[3], encoding="utf-8") as f:
            zone_config = json.load(f)
        errors += check_zones(input_data, solution, zone_config)
        checked += ", and stays out of the no-go zones"

    for e in errors:
        print("VIOLATION:", e)
    if errors:
        print(f"{len(errors)} violation(s) in {argv[2]}")
        return 1
    print(f"OK: {argv[2]} satisfies {checked}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
