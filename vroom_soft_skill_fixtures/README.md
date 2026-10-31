# VROOM soft-skills proposed feature: example fixtures

**Status:** The `soft_skills`, task-level `soft_skill_match_mode`, and `options.soft_skills` fields are a **proposed extension**, not supported fields of upstream VROOM. Do not submit 01–06 to an unmodified upstream solver expecting these fields to work. The `00_baseline_upstream.json` file uses only documented upstream fields.

All data are synthetic. Matrices have 3 indices (0, 1, 2), travel durations in seconds, distances in meters, and arbitrary nonnegative cost units. Time windows in seconds, capacity in a single unit. These toy cases emphasize semantics, **not** real routing economics.

## Proposed extension contract (adapt field names after maintainer review)

- Vehicle-level `soft_skills`: integer array of characteristics possessed by a vehicle.
- Job/shipment-level `soft_skills`: integer array of characteristics used solely for preference classification; they do **not** create a hard eligibility restriction.
- Job/shipment-level `soft_skill_match_mode`: `all` or `any`, overrides the global `options.soft_skills.default_match_mode` (default `all`).
- Empty task `soft_skills`: **neutral**, no preference applies; it is NOT considered a positive match even under `all`.
- Shipment-level preferences are evaluated **once per shipment**; pickup and delivery must remain on the same route.
- Hard `skills` never become optional. Capacity, time windows and pickup/delivery precedence remain hard.
- `options.soft_skills.threshold_mode`: `distance`, `cost`, or `both`; factors are dimensionless and >=1.0. If both active, **both** comparisons must pass (inclusive `<=`).
- `options.soft_skills.max_distance_factor` and `options.soft_skills.max_cost_factor` are *proposed* global settings.

## Fixture index

- `00_baseline_upstream.json`: upstream-compatible control (no soft extensions).
- `01_all_vs_any.json`: global `all`, task override `all`; only vehicle 2 matches task `[101,202]` fully.
- `02_any_override.json`: task override `any`; both vehicles match.
- `03_shipment_override.json`: global `any`, shipment override `all`; only vehicle 1 matches. Pickup/delivery share vehicle.
- `04_hard_skills_override_soft.json`: vehicle 1 matches preferences but fails mandatory hard skill `8`; only vehicle 2 is hard-eligible. Its soft match does **not** override hard eligibility.
- `05_unassigned_capacity.json`: no vehicle has capacity for the job; preference cannot make it feasible.
- `06_empty_skills_neutral.json`: no task soft skills, so there is no preference or penalty.
- `matching_cases.csv`: deterministic unit cases for preference classification.
- `threshold_cases.csv`: deterministic *pure threshold predicate* cases; see warning below.

## Critical threshold caveat

The threshold CSV is a **unit test for a proposed comparison function**, NOT a complete solver expectation. `reference_distance_m` and `reference_cost` stand for explicitly defined non-preferred alternative **marginal insertion deltas**, and `preferred_*` the preferred alternative deltas for a controlled solver state. A ratio is not well-defined for zero or negative reference deltas. Decide and document zero/negative handling, exact equality, whether to compare marginal insertion deltas or whole-solution deltas, whether reference means best feasible non-preferred alternative, and how to avoid path dependence BEFORE solver integration. The toy numbers must not be interpreted as guaranteed VROOM route choices.

## Test guidance

1. First test matching logic independently using `matching_cases.csv`.
2. Test the factor predicates using `threshold_cases.csv` once their semantics are agreed.
3. Validate parsing defaults and invalid values (unknown mode, factor below 1, malformed skill IDs, task overrides, empty lists).
4. Integrate with solver and assert **invariants**, not brittle exact tour order: no hard-skill violations, capacities respected, time windows respected, shipment pickup before delivery on the same vehicle, neutral tasks unaffected by preference logic.
5. Add controlled local-search / insertion tests with known feasible candidate deltas to validate preference decisions; compare against baseline on 300 shipments / 20 vehicles with seeded or repeated runs.
6. Check legacy inputs for behavior and performance regressions.

Official upstream API reference: https://github.com/VROOM-Project/vroom/blob/master/docs/API.md
