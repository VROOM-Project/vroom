#ifndef PREFERENCE_BASELINES_H
#define PREFERENCE_BASELINES_H

/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <algorithm>
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

    if (std::find(sol[v].route.begin(), sol[v].route.end(), job_rank)
        != sol[v].route.end()) {
      // The vehicle currently holding the task is not an "insertion
      // alternative" for it: evaluating an insertion into the route
      // that already contains the (unique) task would produce a bogus
      // near-zero duplicate-insertion reference.
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

// Raw soft-skill match of the task at job_rank with a vehicle: true
// iff the task carries soft skills and the vehicle's skills match
// under the effective mode. Empty task skills are neutral, so this
// is false for them (matching predicate contract).
inline bool task_matches_vehicle(const Input& input,
                                 Index job_rank,
                                 Index v_rank) {
  const auto& job = input.jobs[job_rank];
  if (job.soft_skills.empty()) {
    return false;
  }
  const auto mode = effective_match_mode(job, input.get_soft_skills_options());
  return soft_skill_match(job.soft_skills,
                          input.vehicles[v_rank].soft_skills,
                          mode);
}

// Post-move preference status of placing the task at job_rank on
// vehicle to_vehicle with the given (frozen) insertion eval: the
// vehicle must match and the eval must stay within threshold of the
// frozen best non-matching insertion (Q3 auto-pass when no feasible
// non-matching alternative exists).
inline bool task_post_match(const Input& input,
                            Index job_rank,
                            Index to_vehicle,
                            const Eval& placement_eval,
                            const PreferenceBaselines& baselines) {
  if (!task_matches_vehicle(input, job_rank, to_vehicle)) {
    return false;
  }
  if (!baselines.nonmatch_feasible) {
    return true;
  }
  const auto& opts = input.get_soft_skills_options();
  return threshold_passes(placement_eval.cost,
                          baselines.best_nonmatch_cost->cost,
                          opts.max_cost_factor,
                          placement_eval.distance,
                          baselines.best_nonmatch_distance->distance,
                          opts.max_distance_factor,
                          opts.threshold_mode,
                          opts.missing_distance_policy,
                          input.has_distance_data());
}

} // namespace vroom::ls

#endif
