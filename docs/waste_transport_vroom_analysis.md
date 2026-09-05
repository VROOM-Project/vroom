# Solving the waste transport problem with VROOM: feasibility analysis

Companion to [waste_transport_problem.md](./waste_transport_problem.md),
which states the problem. This document answers: can VROOM (as it is
in this repository, v1.15) solve it, how would the problem be
expressed, and what does not fit.

Contents:
- [Verdict](#verdict)
- [How VROOM sees the problem](#how-vroom-sees-the-problem)
- [Mapping operations to VROOM tasks](#mapping-operations-to-vroom-tasks)
- [The multiban capacity](#the-multiban-capacity)
- [Time constraints](#time-constraints)
- [Objective](#objective)
- [Re-planning during the day](#re-planning-during-the-day)
- [Gaps and limitations](#gaps-and-limitations)
- [Recommended path](#recommended-path)
- [Example input](#example-input)

## Verdict

VROOM solves the daily plan as a pickup-and-delivery problem. The
loading rules of the company are enforced literally through the
`capacities` extension of this repository (plan B, see
[waste_transport_plan_b.md](./waste_transport_plan_b.md)), which is
now implemented: each truck lists its allowed maximal loads and a
load is valid when it fits one of them. The linear encoding described
in [The multiban capacity](#the-multiban-capacity) is kept for the
record, as it explains why the extension was needed. One aspect of
the real problem is not modelled and needs a post-check.

| Requirement | VROOM feature | Status |
| --- | --- | --- |
| Which truck does which operation, in which order | vehicles + shipments, heterogeneous fleet | native |
| Trucks come back to the company to unload, several trips a day | shipments whose delivery is at the company | native (falls out of the shipment model) |
| Pure transport A to B without passing through the company | shipment A to B | native |
| Truck/container compatibility | `capacities` (a kind with 0 in every vector cannot board) | native with the extension |
| Loading rules of small and poliban | `capacities`, one vector per rule | exact |
| Loading rules of the multiban | `capacities`, one vector per rule | exact with the extension; impossible with a single linear capacity under the strict rule list (see below) |
| Start time, finishing time | vehicle `time_window` | native |
| Lunch break | vehicle `breaks` | native |
| Leave operations for tomorrow, with priorities | `priority`, `unassigned` output | native |
| Objective: most operations, then time, then km | solution ranking + `costs` | native |
| New operation mid-day | re-run with current state as input, travel times from OSRM, `steps` warm start | supported by a re-planning procedure, not by a live mode |
| Extender on multiban / poliban | separate `capacities` list per configuration | must be decided before solving |
| Order of stacking | not needed: operators reorder on site | not a constraint |
| Leave-empty and pick-full at the same client done by the same truck in one visit | none | **gap**: usually happens through cost, not guaranteed |

## How VROOM sees the problem

VROOM solves pickup-and-delivery problems with time windows on a
heterogeneous fleet (see [API.md](./API.md)). The pieces that matter
here:

- **Shipments** are pickup/delivery pairs served by the same vehicle,
  pickup first. Several shipments can be interleaved in one route
  (pick A, pick B, deliver A, deliver B). A shipment carries a
  constant `amount` vector between its pickup and its delivery.
- **Jobs** are single stops. A job `delivery` amount is loaded at the
  vehicle start; a job `pickup` amount stays on board until the end.
- **Capacity** is a vector of non-negative integers. At every step of
  a route the component-wise sum of what is on board must be `<=` the
  vehicle `capacity` (`RawRoute::update_amounts` and
  `is_valid_addition_for_capacity` in
  [raw_route.cpp](../src/structures/vroom/raw_route.cpp)). Amounts are
  parsed as unsigned integers, so no negative quantities.
- **Skills** give hard compatibility between tasks and vehicles.
- **Vehicles** have working hours, breaks, start/end locations, a
  `type` string used for per-type service times, and `costs`.
- **Solutions are ranked** (in
  [solution_indicators.h](../src/structures/vroom/solution_indicators.h))
  by: higher priority sum, then more assigned tasks, then lower cost,
  then fewer vehicles used, then duration, then distance.

What VROOM does **not** have: a multi-trip concept, "same vehicle" or
"same visit" links between independent tasks, or any capacity rule
that is not a linear inequality. (It has no loading-order constraints
either, which is fine: order does not matter in this problem.)

## Mapping operations to VROOM tasks

The key modelling decision: **every container movement is a
shipment**, and the company is an ordinary location that appears as
the pickup of empties and materials and as the delivery of full
containers. A truck that picks up two fulls, drives to the company,
unloads, and goes out again is simply a route whose shipment
deliveries happen to be at the company. Multi-trip behaviour needs no
special support.

The planner works with four operation types, each translated into one
or two shipments (implemented in
[frontend/public/waste_model.js](../frontend/public/waste_model.js)):

| Operation | Shipments | Pickup | Delivery | `amount` |
| --- | --- | --- | --- | --- |
| Deliver an empty container of size N at A | 1 | company | A | one-hot `eN` |
| Pick up an empty container of size N at A | 1 | A | company | one-hot `eN` |
| Exchange at A: leave an empty N, take the full N | 2 | company / A | A / company | one-hot `eN` and one-hot `fN` |
| Sell materials at A (a full truck) | 1 | company | A | one-hot `mat` |

Other movements from the problem statement remain expressible the
same way (pure transport A to B: shipment A to B with `fN`).

**Materials** are an eleventh amount component `mat`. Every truck
gets one extra capacity vector with `mat` = 1 and every container
kind at 0, so a materials delivery occupies the whole truck: nothing
else can be on board on that trip.

Details:

- **Amounts** have one component per container kind, in the order
  `e2, e6, e12, e20, e40, f2, f6, f12, f20, f40`; a container task has
  a one-hot amount. Each vehicle lists its allowed maximal loads in
  `capacities`, one vector per rule of
  [waste_rules.json](./waste_rules.json). Size
  compatibility follows: a kind with 0 in every vector of a truck
  cannot board it, so no skills are needed (they remain useful to pin
  an in-flight load to a truck when re-planning).
- **Company visits**: use `setup` on the delivery steps at the company
  for the fixed overhead of a visit (VROOM applies setup once for
  consecutive tasks at the same location) and `service` for the
  per-container unloading time. Same for pickups of empties.
- **Vehicles** start and end at the company, with `time_window`,
  `breaks`, `skills`, `capacity` (per truck configuration, see below)
  and `type` (`small`, `multiban`, `poliban`) so that operation
  durations can differ per truck through `service_per_type`.
- **Swap at a client** (leave empty, take full) is two shipments. The
  cost structure pushes VROOM to serve both in the same visit, but
  nothing forces it; see the gaps section.
- **Trucks already loaded in the morning**: a pickup at the company
  with zero travel from the vehicle start costs nothing, so no special
  case is needed.
- **Matrices**: the company is a single row/column; it is visited as
  many times as needed.

## The multiban capacity

With the `capacities` extension the multiban is not a special case
any more: its rule list from [waste_rules.json](./waste_rules.json)
goes in as is, see [waste_example.json](./waste_example.json). This
section documents why
a single linear capacity vector was not enough, and the encoding that
would work under an assumption if the extension were not available.

### Why it is hard

VROOM only accepts loads of the form `sum of amount vectors <=
capacity`. A list of allowed combinations can be reproduced exactly by
such a check only if the list is *integrally convex*: every integer
point inside the convex hull of the allowed loads must itself be
allowed. The script
[scripts/waste_capacity_check.py](../scripts/waste_capacity_check.py)
tests this and evaluates a candidate encoding.

### Strict rule list: no exact linear encoding exists

Run `python scripts/waste_capacity_check.py --rules strict`. For the
multiban it finds 28 forbidden loads inside the convex hull. The
conflicting rule pairs it reports are the whole story:

| Allowed | Allowed | Forbidden point between them |
| --- | --- | --- |
| `6e2` | `6e6` | `3e2 + 3e6` |
| `6e2` | `2f2` | `3e2 + 1f2` |
| `6e6` | `2f2` | `3e6 + 1f2` |
| `2f2` | `2f6` | `1f2 + 1f6` |
| `1f6 + 3e2` | `1f6 + 3e6` | `1f6 + 1e2 + 2e6` |

Any set of linear inequalities that accepts `6e2` and `6e6` accepts
their midpoint `3e2 + 3e6`. Worse, a strictly safe linear encoding
that keeps `6e6` can only allow one `e2` at a time before some mixed
load slips in. So with the rules read literally, a linear capacity is
either unsafe or useless for the multiban. This is why previous
additive models (volume or count based) were wrong.

### With one assumption: exact encoding

All five conflicts vanish if the following holds physically:

> **Assumption.** For the loading rules, an empty 2 m³ and an empty
> 6 m³ are interchangeable (small empties nest and stack the same
> way), and a full 2 m³ is interchangeable with a full 6 m³ (a full
> container occupies one bed position regardless of size).

Under it, the allowed multiban loads become these families (each with
anything smaller):

| Family | Examples |
| --- | --- |
| six small empties, any mix of 2 and 6 | `6e2`, `6e6`, `4e2 + 2e6` |
| one empty 12 plus three small empties | `1e12 + 3e6`, `1e12 + 2e2 + 1e6` |
| two fulls (2 or 6) plus one small empty | `2f6 + 1e2`, `1f2 + 1f6 + 1e6` |
| one full (2 or 6) plus three small empties | `1f6 + 3e2`, `1f2 + 2e2 + 1e6` |
| one full 12 plus one small empty | `1f12 + 1e2`, `1f12 + 1e6` |

Every rule of the strict list is included; the additions are exactly
the "unlisted mixtures" of open question 1 in the problem document.
`python scripts/waste_capacity_check.py --rules assumed` confirms that
the set is integrally convex and that the encoding below reproduces it
exactly (70 allowed loads, none lost, none extra).

The encoding uses three components:

| Component | Meaning |
| --- | --- |
| `U` | stack units: how much of the bed and stack height an item uses |
| `C` | bed positions: floor positions used by full containers and by a 12 m³ |
| `P` | poliban slot: one hook-lift container at a time |

| Container | `amount` `[U, C, P]` |
| --- | --- |
| `e2`, `e6` | `[2, 0, 0]` |
| `e12` | `[6, 2, 0]` |
| `f2`, `f6` | `[5, 1, 0]` |
| `f12` | `[10, 2, 0]` |
| `e20`, `e40`, `f20`, `f40` | `[0, 0, 1]` |

| Truck | `capacity` `[U, C, P]` |
| --- | --- |
| small | `[6, 1, 0]` |
| multiban | `[12, 2, 0]` |
| poliban | `[0, 0, 1]` |

Checks worth reading off the table: `2f6 + 1e2` is `[12, 2, 0]`
(fits), `2f6 + 2e2` is `[14, 2, 0]` (does not), `1f12 + 1e2` is
`[12, 2, 0]` (fits), `1e12 + 1f2` is `[11, 3, 0]` (does not: a 12
takes the whole bed), `3e2` on the small truck is `[6, 0, 0]` (fits)
and `1f2 + 1e2` is `[7, 1, 0]` (does not). Type compatibility (no
6 m³ on a small truck) is left to skills.

If the company rejects part of the assumption, rerun the script after
editing `RULES_STRICT` / `INTERCHANGEABLE`: it will say whether the
new set is still linearly representable and what the encoding gets
wrong. If it is not representable, the only exact option is plan B.

### Extender

A truck with the extender is a different capacity vector. VROOM
cannot express "this truck is either configuration A or B, choose
one", so fitting the extender is an input decision per truck per day
(or run the solver once per scenario). Once the extender rule lists
are known, add them as new entries in `RULES_STRICT` and `CAPACITY`
in the script and verify the same way.

### Plan B: alternative capacity vectors in the core (implemented)

The loading rules are enforced literally by giving each vehicle a list
of capacity vectors (`capacities`); a load is valid when it fits at
least one of them. The component-wise maximum of the list (the hull)
is kept as a relaxation for the O(1) peak-based shortcuts of
`RawRoute`, and the exact step-by-step check only runs when the hull
accepts a load that no single vector accepts. Design, decisions and
code map are in
[waste_transport_plan_b.md](./waste_transport_plan_b.md).

## Time constraints

- **Working hours**: vehicle `time_window` `[start, end]`, in seconds
  (relative to midnight or absolute timestamps, as long as all inputs
  agree).
- **Lunch break**: one entry in vehicle `breaks` with the allowed
  start window and `service` equal to the break length. VROOM places
  the break between two tasks.
- **Operation durations**: `service` on each pickup and delivery step,
  `service_per_type` when they differ per truck type, `setup` for the
  per-visit overhead at the company.
- **Client opening hours**: `time_windows` on the corresponding
  shipment step, if needed.
- Because a shipment delivery must be in the route, every full
  container picked up is unloaded before the truck ends its day.

## Objective

VROOM ranks solutions by priority sum, then assigned tasks, then cost,
then vehicles used, then duration, then distance. This matches the
stated order (operations first, then time, then km).

- Give each shipment a `priority` in `[0, 100]` (0 default). Higher
  values are dropped last. Both steps of a shipment must carry the
  same priority.
- Cost: with `costs.per_hour` = 3600 (default) the cost is travel
  time; add `costs.per_task_hour` = 3600 to make it total working
  time (travel + service). Add a small `costs.per_km` to break ties on
  kilometres (this requires distances: `-g`, or `distances` in the
  custom matrices).
- Unassigned operations appear in the `unassigned` output array; they
  are the input of the next day.
- Latency being a non-issue, run with the highest exploration level
  (`-x 5`) and several threads (`-t`).

## Re-planning during the day

VROOM has no incremental mode, but a new run from the current state is
cheap at this scale, and the OSRM instance that backs VROOM provides
the travel times from wherever the trucks are: give each vehicle its
current GPS coordinates as `start` and VROOM fetches the matrix from
OSRM itself (no precomputed matrices needed). Procedure at
re-planning time `t`:

1. For each truck: `start` = its current coordinates (or its next stop
   if it is about to arrive), `time_window` = `[t, end]`; keep the
   lunch `break` if not taken yet; `end` unchanged (the company).
2. Drop shipments fully done.
3. Shipments picked up but not yet delivered become **jobs with a
   `delivery` amount** at the destination. VROOM loads job deliveries
   at the vehicle start, which is exactly "already on board". Assign
   them to that truck only, for instance with a dedicated skill.
4. Shipments not started stay shipments; add the new operation(s) as
   shipments.
5. Optionally pass the current plan as vehicle `steps` to warm-start
   the search from it (in solving mode this follows a single search
   path from the given solution, which keeps the new plan close to the
   old one). The steps must describe a valid route or VROOM errors.

## Gaps and limitations

1. **Same-visit linking.** The two shipments of a swap are
   independent. VROOM will normally do both in one visit because it is
   cheaper, but it can split them across trucks or visits when
   capacity or time pushes that way. Detect in a post-check; if it
   happens too often, it can be reduced by giving both shipments the
   same tight `time_windows`.
2. **Loading rules** are enforced exactly through `capacities`; the
   assumption above is no longer needed.
3. **Extender** is an input decision, not an optimisation decision.
4. **Materials** travel as a full truck (`mat` component, exclusive
   capacity vector) on any truck type; if partial loads or mixing with
   containers turn out to be allowed, this becomes a matter of adding
   vectors.
5. **Stock of empties at the company** is assumed unlimited. If stock
   per size is limited, create at most that many leave-empty shipments
   per size (VROOM will drop the extra ones).
6. **Company unloading throughput** (trucks queueing) is out of scope,
   as stated in the problem.

## Recommended path

1. Get answers to the remaining open questions of the problem
   document (extender lists, materials, small truck with a full 2).
   Each answer is just another vector in the relevant `capacities`
   list.
2. Write the input generator (operations database to VROOM JSON) using
   the example below as a template: one-hot amounts per container
   kind, one `capacities` list per truck configuration. Use the OSRM
   instance that will run next to VROOM (`-r osrm`, `-a`, `-p`
   flags).
3. Run [scripts/waste_solution_check.py](../scripts/waste_solution_check.py)
   on VROOM output as a post-check, and extend it with same-visit
   linking.
4. Build the stack from the local sources with
   [docker-compose.local.yml](../docker-compose.local.yml) so that the
   deployed solver includes the extension.

## Example input

[waste_example.json](./waste_example.json) is a small instance with
custom matrices (no routing server needed): the company, three
clients, a disposal site, one multiban, one small truck, one poliban,
and eight operations including a swap, a pure transport, a full 12 m³
and two poliban containers. Vehicles use `capacities` generated from
[waste_rules.json](./waste_rules.json) with
`scripts/waste_rules_to_capacities.py`. Run it with:

```bash
vroom -i docs/waste_example.json -x 5 -o out.json
python scripts/waste_solution_check.py docs/waste_example.json out.json
```

Times in the example are seconds since midnight (8:00 to 17:00, lunch
window 12:00 to 13:00).
