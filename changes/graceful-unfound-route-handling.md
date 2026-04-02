# Graceful Handling of Unfound Routes (Big-M Fallback)

## Problem

When OSRM returns `null` matrix entries (no route between points) or `NoRoute` errors for individual route requests, vroom threw a `RoutingException` and aborted the entire solve. This meant **zero routes** were returned even though most location pairs were reachable.

The errors that triggered this failure looked like:

```json
{
  "message": "Impossible route between points",
  "code": "NoRoute"
}

{
  "message": "No route found between points",
  "code": "NoRoute"
}
```

The specific failure points were:

1. **`check_unfound` in `wrapper.h`**: When OSRM's table endpoint returned `null` cells for unreachable pairs, the matrix builder counted them and then threw `RoutingException("Unfound route(s) from/to location [lon,lat]")`, killing the entire solve.

2. **`check_response` in `osrm_routed_wrapper.cpp`**: When OSRM's route endpoint returned `NoRoute`, it threw a `RoutingException`. In sparse matrix mode (`get_sparse_matrices`), one vehicle's routing failure killed all vehicles.

3. **Matrix default value of 0**: When null entries were detected but not assigned, cells remained at 0 (the `Matrix(n)` default), meaning "free travel" — which would cause the solver to eagerly route through impossible edges.

4. **Geometry decoration**: If a single route's geometry request failed (e.g. the OSRM route endpoint returned `NoRoute` for a solved route's sequence), the entire solution was discarded.

## Solution

Replace null/unreachable matrix entries with very large fallback penalty values (Big-M approach). The solver naturally avoids these edges due to their extreme cost, and any truly unreachable jobs end up in the `unassigned` list rather than causing a total failure.

### Fallback constants

```cpp
constexpr UserDuration UNFOUND_ROUTE_DURATION = 500000; // ~5.8 days
constexpr UserDistance UNFOUND_ROUTE_DISTANCE = 500000; // ~500 km
```

These values are large enough that the solver will never voluntarily use these edges (139x–833x typical urban travel times), but small enough that summing across all locations in `check_cost_bound` won't overflow `uint32_t` even for problems with ~8000 locations.

## Files Changed

| File | Change |
|------|--------|
| `src/structures/typedefs.h` | Added `UNFOUND_ROUTE_DURATION` and `UNFOUND_ROUTE_DISTANCE` constants |
| `src/routing/wrapper.h` | Renamed `check_unfound` → `warn_unfound` (warns to stderr instead of throwing). Made `get_sparse_matrices` catch per-vehicle `RoutingException`s and fill fallback values instead of aborting |
| `src/routing/http_wrapper.cpp` | `get_matrices()` fills null OSRM table entries with fallback values instead of leaving them at 0 and throwing |
| `src/routing/libosrm_wrapper.cpp` | Same change as `http_wrapper.cpp` for the libosrm code path |
| `src/structures/vroom/input/input.cpp` | `add_geometry()` catches per-route geometry failures. Routes that fail geometry are dropped and their jobs moved to unassigned, preserving the response contract (every returned route has geometry) |

## Behavioral Changes

**Before:** Any `null` in the OSRM matrix → `RoutingException` → exit code 3 → zero routes returned.

**After:**

- `null` matrix entries → fallback penalty values → solver avoids those edges → unreachable jobs listed as `unassigned` → all feasible routes still returned
- Per-vehicle sparse routing failure → fallback values for that vehicle's edges → vehicle may end up empty or with unassigned jobs
- Per-route geometry failure → route dropped, its jobs moved to unassigned. Response contract preserved: every route in the output has geometry

## Response Contract

The JSON response shape is unchanged. Every route in the `routes` array has a `geometry` field when geometry is requested. Jobs from dropped routes appear in the `unassigned` array. The `summary` reflects the filtered set.

## Warnings Emitted

All warnings are printed to stderr so they're visible in logs without polluting the JSON output on stdout:

- `[Warning] N unfound route(s) in matrix, worst: from/to location [lon,lat]. Using fallback values for unreachable pairs.`
- `[Warning] Sparse matrix routing failed for vehicle V: <message>. Using fallback values.`
- `[Warning] Failed to get geometry for route V: <message>. Route will lack geometry.`
- `[Warning] Dropped N route(s) due to geometry failure; M job(s) moved to unassigned.`
