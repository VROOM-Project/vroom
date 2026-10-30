#ifndef PREFERENCE_BASELINES_H
#define PREFERENCE_BASELINES_H

/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <optional>
#include <vector>

#include "algorithms/local_search/insertion_search.h"
#include "structures/typedefs.h"
#include "structures/vroom/eval.h"
#include "structures/vroom/input/input.h"
#include "structures/vroom/solution_state.h"
#include "utils/soft_skills.h"

namespace vroom::ls {

// On-demand threshold reference for soft-skills preference decisions
// (Task 16). Computed fresh at each call from the evaluation context
// passed in: nothing is cached, so results are only valid for that
// frozen (typically pre-move) state.
struct PreferenceBaselines {
  // Best (lowest cost) feasible insertion eval across all vehicles
  // that do **not** match the job's soft-skills.
  std::optional<Eval> best_nonmatch_cost;

  // Best (lowest distance) feasible insertion eval across the same
  // non-matching vehicles, picked from their per-vehicle best
  // insertions.
  std::optional<Eval> best_nonmatch_distance;

  // False when no non-matching vehicle admits a feasible insertion
  // of the job at all, in which case threshold comparisons
  // automatically pass (Q3 auto-pass).
  bool nonmatch_feasible;
};

// For the job at rank job_rank (JOB_TYPE::SINGLE or JOB_TYPE::PICKUP,
// never its DELIVERY counterpart), scanning all vehicles: matching
// vehicles are skipped, then each remaining vehicle is checked for
// hard compatibility (vehicle_ok_with_job) and capacity/TW-feasible
// insertion positions into the current content of its route, tracked
// as non-matching insertion cost and distance references.
template <class Route>
inline PreferenceBaselines
compute_preference_baselines(const Input& input,
                             const utils::SolutionState& sol_state,
                             const std::vector<Route>& sol,
                             Index job_rank) {
  const auto& job = input.jobs[job_rank];
  assert(job.type == JOB_TYPE::SINGLE || job.type == JOB_TYPE::PICKUP);

  const auto& opts = input.get_soft_skills_options();
  const auto mode = effective_match_mode(job, opts);

  PreferenceBaselines baselines{};

  const auto nb_vehicles = input.vehicles.size();
  for (Index v = 0; v < nb_vehicles; ++v) {
    if (soft_skill_match(job.soft_skills,
                         input.vehicles[v].soft_skills,
                         mode)) {
      // Matching vehicle: does not take part in the non-matching
      // reference.
      continue;
    }

    if (!input.vehicle_ok_with_job(v, job_rank)) {
      continue;
    }

    // Best insertion of the job into the current content of route v,
    // if any (capacity and time windows included).
    const auto insertion =
      (job.type == JOB_TYPE::SINGLE)
        ? compute_best_insertion_single(input, sol_state, job_rank, v, sol[v])
        : compute_best_insertion_pd(input,
                                    sol_state,
                                    job_rank,
                                    v,
                                    sol[v],
                                    NO_EVAL);

    if (insertion.eval == NO_EVAL) {
      continue;
    }

    baselines.nonmatch_feasible = true;
    if (!baselines.best_nonmatch_cost.has_value()
        || insertion.eval.cost < baselines.best_nonmatch_cost->cost) {
      baselines.best_nonmatch_cost = insertion.eval;
    }
    if (!baselines.best_nonmatch_distance.has_value()
        || insertion.eval.distance
             < baselines.best_nonmatch_distance->distance) {
      baselines.best_nonmatch_distance = insertion.eval;
    }
  }

  return baselines;
}

} // namespace vroom::ls

#endif
