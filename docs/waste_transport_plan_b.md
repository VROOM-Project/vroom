# Plan B: capacity as a set of alternative capacity vectors

Design draft for replacing VROOM's single linear capacity check by a
rule-table check, so that the loading rules of
[waste_transport_problem.md](./waste_transport_problem.md) can be
enforced literally. Status: **implemented** in this repository (see
[Implementation status](#implementation-status) at the end); the rest
of the document is the original design, kept as the rationale.

Contents:
- [Idea](#idea)
- [Input format](#input-format)
- [Semantics](#semantics)
- [What changes in the solver](#what-changes-in-the-solver)
- [Performance](#performance)
- [Backward compatibility](#backward-compatibility)
- [Testing](#testing)
- [Implementation steps](#implementation-steps)
- [Open decisions](#open-decisions)

## Idea

Today a load `L` (component-wise sum of what is on board) is valid iff
`L <= capacity`. The generalisation: a vehicle has **several**
capacity vectors and a load is valid iff it fits **at least one** of
them:

```
valid(L)  <=>  exists k : L <= capacities[k]   (component-wise)
```

Why this is the right shape for the problem:

- The company already expresses its rules as a list of *maximal
  allowed combinations* (`6e2`, `2f6 + 1e2`, ...). Each one is a
  capacity vector; the list is the vehicle's capacity. No encoding
  step, no assumption, and the JSON can be checked against the
  company table by eye.
- "Anything smaller than an allowed combination is allowed" is built
  in: every finite down-closed set is exactly the union of the boxes
  below its maximal elements. So this covers *any* rule table of the
  kind the company can produce, for any truck or chico
  configuration.
- With one capacity vector it is exactly today's VROOM, so nothing
  changes for existing users and the change is generic enough to be
  proposed upstream.

Amount components become **container kinds**, one per kind
(`e2, e6, e12, e20, e40, f2, f6, f12, f20, f40`, plus one for
materials if needed). Every container task has a one-hot `amount`.
Size compatibility no longer needs skills: a kind that has `0` in
every capacity vector of a truck cannot go on it, and VROOM already
turns "does not fit an empty route" into an incompatibility
(`Input::set_extra_compatibility`).

The rule lists live in [waste_rules.json](./waste_rules.json);
`python scripts/waste_rules_to_capacities.py` prints the corresponding
`capacities` arrays (one vector per rule, components in the order
above; add `--materials` for the extra materials component the
planner uses) ready to paste into a VROOM input.
A rule dominated by another one (e.g. `1e12` next to `1e12 + 3e6`) is
redundant and is dropped by the solver at parse time.

## Input format

Vehicle object:

| Key | Description |
| --- | --- |
| `capacity` | unchanged: a single vector |
| `capacities` | **new**: array of vectors; the load must fit at least one of them |

Rules to settle (see open decisions): providing both is an error
(simplest), or `capacity` is treated as one more alternative. Every
vector must have the amount size of the instance. Dominated vectors
may be dropped at parse time. An empty array means no capacity
restriction, like today.

Nothing changes for jobs and shipments. Output is unchanged; the
per-step `load` array now reads directly as "what is on the truck".

## Semantics

Internally `Vehicle` holds:

- `capacities`: the list of vectors;
- `capacity`: the **hull**, i.e. the component-wise maximum over the
  list. It is a relaxation: `valid(L)` implies `L <= hull`. All
  existing code that uses `capacity` as a *necessary* condition keeps
  working unchanged with the hull.

New predicate: `Vehicle::can_carry(const Amount& L)`, linear in
`|capacities| * amount_size`.

## What changes in the solver

The solver never checks a full route from scratch; it checks *moves*
incrementally through a handful of `RawRoute` methods (inherited by
`TWRoute`) and a few direct comparisons in operators. Three kinds of
existing checks must be classified:

1. **Exact step scans.** `is_valid_addition_for_capacity_inclusion`
   already walks the inserted range step by step comparing to
   `capacity`. Replace the comparison by `can_carry`. Same for the
   route validation of input `steps` (`heuristics.cpp`) and plan-mode
   validation (`choose_ETA.cpp`). Trivial.

2. **Peak-based checks.** `is_valid_addition_for_capacity` and
   `is_valid_addition_for_capacity_margins` compare `fwd_peak +
   delivery` and `bwd_peak + pickup` to `capacity`, where a peak is the
   component-wise maximum of the loads over a prefix or suffix. With
   several capacity vectors a peak test is no longer equivalent to
   testing every step (loads `6e2` then `6e6` are each fine while
   their peak `6e2 + 6e6` fits nothing). Replace by a three-tier test:
   - reject if `peak + delta` does not fit the hull (necessary
     condition, O(1));
   - accept if `peak + delta` fits one of the capacity vectors
     (sufficient condition, O(k));
   - otherwise scan the affected steps: `can_carry(load[s] + delta)`
     for each step `s` in the affected prefix or suffix (exact,
     O(n·k)).
   `is_valid_addition_for_load` is a single-step check: just
   `can_carry`.

3. **Direct comparisons in operators and candidate filters.**
   - Pruning only (`if (!(x <= margin)) continue;`): the margin
     early-aborts in `local_search.cpp` and `swap_star_utils.h`. They
     stay as they are, computed against the hull. Slightly weaker
     pruning, never wrong.
   - Used as the actual validity test: `RouteExchange`
     (`max_load() <= other.capacity`), `route_split_utils.h`
     (`sub_route_max_load_* <= capacity`), tail swaps in `TwoOpt` /
     `ReverseTwoOpt` and the matching candidate filters in
     `local_search.cpp`. Under the new semantics "max load fits one
     vector" is sufficient but not necessary, so keeping them as is
     makes these operators *conservative* (they miss some valid moves,
     never accept an invalid one). To make them exact, add one generic
     helper on `RawRoute`:

     ```
     bool loads_fit(const Vehicle& v, Index first_step, Index last_step,
                    const Amount& delta) const;
     // every _current_loads[s] + delta for s in [first, last) can be
     // carried by v (v may be another vehicle than the route's own)
     ```

     and express each of those checks as one or two `loads_fit` calls
     (source head with the target tail's deliveries added, target tail
     with the source head's pickups added, and so on).

`set_vehicles_max_tasks` (job-only instances) and the vehicle sort
order use the hull; both are heuristics or relaxations and stay valid.
Break `max_load` is a different constraint (load at break time
`<= max_load`) and is untouched.

Files touched: `vehicle.h/.cpp`, `raw_route.h/.cpp`,
`input_parser.cpp`, `input.cpp` (validation of the new key),
`heuristics.cpp`, `choose_ETA.cpp`, `route_exchange.cpp`,
`route_split_utils.h`, `two_opt.cpp`, `reverse_two_opt.cpp`,
`local_search.cpp` (a few filters), `libvroom_examples/libvroom.cpp`
(constructor signature), `docs/API.md`, `CHANGELOG.md`.

## Performance

For this problem: routes of a few dozen tasks, about 10 capacity
vectors, about 10 components. The exact scan is a few thousand integer
comparisons instead of about ten, but it only runs when the O(1)
reject and O(k) accept tiers are inconclusive, which is the minority
of checks. A slowdown of one order of magnitude on capacity checks is
plausible; against a stated budget of minutes for a small fleet this
is irrelevant. `-l` (time limit) remains available.

## Backward compatibility

With a single capacity vector, hull = that vector, tier 1 and tier 2
coincide with today's test and tier 3 is never reached. The solver
behaves identically on every existing input, so no regression is
expected for the unchanged `capacity` key.

## Testing

There is no unit-test suite in this repository, so:

- **Oracle checker** in Python: extend
  [scripts/waste_capacity_check.py](../scripts/waste_capacity_check.py)
  (or add a sibling) to read a VROOM input and solution and verify
  every step load against the capacity vectors, precedence and
  vehicle compatibility. Independent of the C++ code.
- **Regression**: run the existing `docs/example_*.json` before and
  after; outputs must be byte-identical.
- **Targeted instances**: small hand-built cases where the peak test
  and the exact test disagree (the `6e2` then `6e6` route, tail swaps
  between a multiban and a small truck), checked with the oracle.
- **Randomised**: generate small random instances with random
  capacity lists, run VROOM, run the oracle, assert no violation.

## Implementation steps

1. Data model and parsing: `capacities` key, `Vehicle::can_carry`,
   hull, input validation, libvroom constructor.
2. Replace the trivial comparisons (tier "exact step scans").
3. Three-tier logic in `is_valid_addition_for_capacity` and
   `is_valid_addition_for_capacity_margins`; `can_carry` in
   `is_valid_addition_for_load`.
4. `loads_fit` helper and exact versions of `RouteExchange`, route
   split, `TwoOpt`, `ReverseTwoOpt` (or leave them conservative in a
   first release, see decisions).
5. Docs: `API.md`, `CHANGELOG.md`; rewrite
   [waste_example.json](./waste_example.json) with kinds as components
   and `capacities`; update the analysis document.
6. Oracle checker and the test instances above.
7. Build and run everything. **Prerequisite**: a build environment
   (Linux, WSL or a container with a C++20 compiler, asio, OpenSSL and
   optionally GLPK); none is available on the current Windows machine.

Rough effort: steps 1 to 3 are a day; step 4 another day; tests and
docs a day; plus the build environment.

## Open decisions

1. **Representation**: alternative capacity vectors (recommended, the
   company's own format) versus an explicit list of every allowed
   load.
2. **Key semantics**: `capacities` exclusive with `capacity`, or
   `capacity` as one more alternative when both are present.
3. **Exactness of tail-swap operators** from day one, or conservative
   first and exact in a second pass.
4. **Amount components = container kinds** with one-hot amounts (drop
   the `[U, C, P]` encoding and the size skills), keeping skills only
   to pin in-flight loads to a truck during re-planning.
5. **Build/test environment**: WSL, a Linux box, or a container.
6. **Upstream**: keep the change free of anything waste-specific so
   that it can be offered to the upstream project.

## Implementation status

Decisions taken (2026-09-05):

1. Representation: `capacities`, an array of alternative capacity
   vectors on the vehicle (see [API.md](./API.md#alternative-capacity-vectors)).
2. `capacity` and `capacities` are exclusive: providing both is an
   input error. A `capacities` list reduced to one vector (after
   dropping dominated vectors) is folded into `capacity`, so the
   solver takes exactly the old code path.
3. All checks are exact from day one: insertion checks use the
   three-tier test (hull, single vector, step scan), `RouteExchange`
   and route splitting fall back to a step scan through
   `RawRoute::loads_fit`, and `TwoOpt` / `ReverseTwoOpt` are exact
   because their margin and inclusion checks are. The remaining
   direct comparisons in `local_search.cpp` and `swap_star_utils.h`
   are pruning only (necessary conditions on the hull).
4. Amount components are container kinds with one-hot amounts;
   [waste_example.json](./waste_example.json) uses this format and no
   longer needs skills.
5. Build and test environment: Docker, see
   [vroom-custom/Dockerfile.local](../vroom-custom/Dockerfile.local),
   [docker-compose.local.yml](../docker-compose.local.yml) and
   [scripts/vroom_local.sh](../scripts/vroom_local.sh).
6. Nothing waste-specific in the C++.

Where things are in the code:

| Piece | Location |
| --- | --- |
| `capacities` member, hull, `can_carry`, dominated vector pruning | `src/structures/vroom/vehicle.h/.cpp` |
| JSON key parsing and exclusivity check | `src/utils/input_parser.cpp` |
| Three-tier checks, `loads_fit`, `nb_steps` | `src/structures/vroom/raw_route.h/.cpp` |
| Exact `RouteExchange` | `src/problems/cvrp/operators/route_exchange.cpp` |
| Exact route split | `src/algorithms/local_search/route_split_utils.h` |
| Input route validation and plan mode | `src/algorithms/heuristics/heuristics.cpp`, `src/algorithms/validation/choose_ETA.cpp` |
| Oracle checker | [scripts/waste_solution_check.py](../scripts/waste_solution_check.py) |
