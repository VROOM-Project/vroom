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
- every task appears exactly once, either in a route or in
  "unassigned";
- the "load" arrays reported by VROOM match the recomputation when
  present.

Exit status is 0 when no violation is found, 1 otherwise. Usage:

    python scripts/waste_solution_check.py input.json solution.json
"""

import json
import sys


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
    if len(argv) != 3:
        print(__doc__)
        return 2
    with open(argv[1], encoding="utf-8") as f:
        input_data = json.load(f)
    with open(argv[2], encoding="utf-8") as f:
        solution = json.load(f)

    errors = check(input_data, solution)
    for e in errors:
        print("VIOLATION:", e)
    if errors:
        print(f"{len(errors)} violation(s) in {argv[2]}")
        return 1
    print(f"OK: {argv[2]} satisfies capacity, precedence and skills constraints")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
