# Vroom Fork Bug: Solver Violates Pinned Constraints at explore >= 1

## Summary

The Trexity vroom fork (`trexity-1.15.0-202602262014`) has a solver bug where local search optimization (explore levels 1–5) moves pinned tasks off their pinned vehicles or reorders `pinned_position: "first"` tasks away from the start of routes. The post-solve validation then correctly rejects the solution, causing an INPUT_ERROR (exit code 2). At `explore=0` (greedy construction only, no local search), all inputs produce valid solutions.

## Affected Binary

- **Version**: `vroom 1.15.0 (Trexity edition 202602262014)`
- **Git ref**: `trexity-1.15.0-202602262014` from `github.com/trexitycode/vroom`
- **Nix derivation**: `devbox.d/packages/vroom-1.15.0/vroom.nix`
- **Docker**: `packages/routing/Dockerfile` builds from the same ref

## Error Messages

Two distinct post-solve validation errors:

1. `"Pinned task {id} not assigned to pinned vehicle {vehicleId}."` — A task with `pinned: true` that appears in a vehicle's `steps` array ended up assigned to a different vehicle (or unassigned) in the final solution.

2. `"Pinned-first task {id} not first on vehicle {vehicleId}."` — A task with `pinned_position: "first"` did not end up as the first task on its vehicle in the final solution.

Both return vroom error code 2 (INPUT_ERROR) even though the input is structurally valid — the solver itself is producing an invalid solution.

## Reproduction

Any of the JSON files in `logs/vroom-failures/` with `pinned_lateness_limit_sec: 3600` can reproduce the bug. For example:

```bash
# Fails at explore=5 (default)
vroom -i logs/vroom-failures/1774023123553_428447.json -t 4 -x 5
# Output: {"code":2,"error":"Pinned task 82 not assigned to pinned vehicle 3."}

# Succeeds at explore=0
vroom -i logs/vroom-failures/1774023123553_428447.json -t 4 -x 0
# Output: {"code":0,...} (valid solution)
```

### Systematic proof across all 13 post-deploy failure files

| explore | files that succeed | files that fail |
|---------|-------------------|-----------------|
| 0       | 13                | 0               |
| 1       | 5                 | 8               |
| 2       | 4                 | 9               |
| 3       | 2                 | 11              |
| 4       | 2                 | 11              |
| 5       | 2                 | 11              |

### Not a lateness issue

Increasing `pinned_lateness_limit_sec` to any value (tested up to 999,999 seconds) does NOT fix the problem at `explore >= 1`. The solver is not failing due to time window infeasibility — it is actively displacing pinned tasks during its local search moves.

```
limit=  3600, explore=5 -> FAIL
limit=  7200, explore=5 -> FAIL
limit= 14400, explore=5 -> FAIL
limit= 36000, explore=5 -> FAIL
limit= 86400, explore=5 -> FAIL
limit=999999, explore=5 -> FAIL
limit=  3600, explore=0 -> SUCCESS
```

### Isolated vehicles work fine

When a failing vehicle is isolated with only its pinned tasks (removing all other vehicles and tasks from the input), the solve succeeds at any explore level. The bug only manifests during global multi-vehicle optimization where the local search has cross-vehicle moves available.

## Root Cause Analysis

The vroom solver operates in two phases:

1. **Construction phase** (`explore=0`): Builds an initial feasible solution using a greedy heuristic. This phase respects pinned assignments — tasks in a vehicle's `steps` array are placed on that vehicle, and `pinned_position: "first"` tasks are placed at the start of the route.

2. **Local search phase** (`explore >= 1`): Iteratively improves the solution using neighborhood moves (swap, relocate, cross-exchange, etc.) between routes. This phase does NOT properly preserve pinned constraints. During optimization, it may:
   - Move a pinned task from its pinned vehicle to another vehicle (or to unassigned) if doing so reduces the objective function
   - Reorder a `pinned_position: "first"` task to a non-first position within its route

The post-solve validation (correctly) checks that all pinned constraints are satisfied in the final solution and rejects solutions where they are violated. But the local search operators are not constrained to maintain these invariants.

## Expected Fix

The local search operators need to be modified to treat pinned assignments as hard constraints that cannot be violated during any move. Specifically:

1. **Pinned task assignment**: A task that appears in a vehicle's `steps` must NOT be moved to a different vehicle or to unassigned during any local search move (relocate, swap, cross-exchange, etc.).

2. **Pinned-first ordering**: A task with `pinned_position: "first"` must NOT be moved away from the first position on its vehicle during any intra-route or inter-route move.

These constraints should be enforced in the move feasibility checks within the local search, not just in the post-solve validation.

## Relevant Vroom Source Areas

In standard vroom 1.15.0, the local search is implemented in:

- `src/algorithms/local_search/local_search.h` — Main local search loop
- `src/algorithms/local_search/operator.h` — Base class for move operators
- `src/algorithms/local_search/swap_star_utils.h` — SWAP* operator
- `src/problems/vrptw/operators/` — VRPTW-specific operators (relocate, swap, cross, etc.)

The Trexity fork additions for pinned tasks likely include:

- Validation in `src/utils/input_parser.cpp` or similar (the post-solve checks)
- Step seeding in the construction phase
- The `pinned`, `pinned_position`, `pinned_soft_timing`, and `pinned_lateness_limit_sec` input fields

The fix needs to add feasibility guards in the local search operators that skip any move which would violate a pinned constraint.

## Current Workaround

FRCS sets `explore: 0` in the `optimizeFleet` call to bypass the local search entirely. This produces solutions that are 0–4% worse in cost/distance compared to `explore=5` but eliminates all pinned constraint failures.

```js
// packages/common-server/src/frcs/services/FluidRouteCommitmentService/index.js
options: {
  zoneMultipliers,
  baseMultiplier,
  pinnedSoftTiming: true,
  pinnedLatenessLimitSec: 3600,
  explore: 0,  // Workaround for vroom fork solver bug
  includeActionTimeInBudget: true,
  exclusiveTagsAllowPinnedConflicts: true
}
```

## Quality Impact of explore=0

Measured on production inputs that succeed at both explore levels:

| Input size | Cost at x=0 | Cost at x=5 | Diff |
|---|---|---|---|
| 137 vehicles, 88 jobs, 96 shipments | 286,492 | 276,523 | +3.6% |
| 124 vehicles, 91 jobs, 96 shipments | 235,767 | 236,549 | -0.3% |
| 106 vehicles, 91 jobs, 73 shipments | 276,144 | 277,922 | -0.6% |
| 107 vehicles, 91 jobs, 73 shipments | 280,649 | 275,240 | +2.0% |

Average quality loss is ~1–2% in cost. Some cases are actually better at `explore=0` since the higher exploration is a heuristic, not guaranteed to improve.

## Test Data

81 failure dump files are available in `logs/vroom-failures/`. The 13 post-deploy files (with `pinned_lateness_limit_sec: 3600`) are the most relevant for reproducing and testing fixes. Each file is a complete vroom input JSON that can be passed directly to the `vroom` binary with `-i`.
