#ifndef PREFERENCE_SCORING_H
#define PREFERENCE_SCORING_H

/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <cassert>
#include <vector>

#include "algorithms/local_search/preference_baselines.h"
#include "structures/typedefs.h"
#include "structures/vroom/solution/soft_skill_preference.h"
#include "structures/vroom/solution_state.h"
#include "utils/helpers.h"

namespace vroom::ls {

// Authoritative soft-skill preference counts for the route served by
// vehicle v_rank of sol (Task 25). Every assigned task carrying soft
// skills is eligible and falls in exactly one bucket (Q14):
// satisfied iff the assigned vehicle matches the task skills under
// its effective match mode AND the task passes the active cost and
// distance thresholds; unsatisfied otherwise. Scoring is binary (Q8).
// Shipments count once through their PICKUP endpoint, DELIVERY
// endpoints never count, unassigned tasks never count.
//
// The threshold reference is the best feasible insertion eval over
// non-matching vehicles, recomputed from the very same (sol,
// sol_state) context through compute_preference_baselines -- exactly
// the evaluator the move operators use -- so reported counts and
// incumbent decisions share one source of truth. Q3 auto-pass: with
// no feasible non-matching insertion anywhere, a matching assignment
// satisfies automatically.
//
// The task's own placement eval is its marginal contribution to the
// route: remove the task (and its delivery counterpart for shipments)
// from the route, then evaluate re-insertion at the very same
// position(s) with utils::addition_eval, the stateless workhorse
// behind the baseline insertion evals (same cost and distance units).
template <class Route>
inline SoftSkillPreference
compute_route_preference(const Input& input,
                         const std::vector<Route>& sol,
                         const utils::SolutionState& sol_state,
                         Index v_rank) {
  SoftSkillPreference counts;
  const auto& route = sol[v_rank].route;

  for (Index rank = 0; rank < route.size(); ++rank) {
    const auto job_rank = route[rank];
    const auto& job = input.jobs[job_rank];
    if (job.type == JOB_TYPE::DELIVERY || job.soft_skills.empty()) {
      // Deliveries are counted through their pickup, tasks without
      // soft skills are neutral.
      continue;
    }

    ++counts.eligible;

    bool satisfied = false;
    if (task_matches_vehicle(input, job_rank, v_rank)) {
      const auto baselines = compute_preference_baselines(input,
                                                          sol_state,
                                                          sol,
                                                          job_rank);
      if (!baselines.nonmatch_feasible) {
        satisfied = true;
      } else {
        Eval placement;
        if (job.type == JOB_TYPE::SINGLE) {
          std::vector<Index> remaining;
          remaining.reserve(route.size() - 1);
          for (Index i = 0; i < route.size(); ++i) {
            if (i != rank) {
              remaining.push_back(route[i]);
            }
          }
          placement = utils::addition_eval(input,
                                           job_rank,
                                           input.vehicles[v_rank],
                                           remaining,
                                           rank);
        } else {
          // Shipment: the delivery endpoint sits on the same route by
          // solution validity.
          Index delivery_rank = route.size();
          for (Index i = rank + 1; i < route.size(); ++i) {
            if (route[i] == job_rank + 1) {
              delivery_rank = i;
              break;
            }
          }
          assert(delivery_rank < route.size());

          std::vector<Index> remaining;
          remaining.reserve(route.size() - 2);
          for (Index i = 0; i < route.size(); ++i) {
            if (i != rank && i != delivery_rank) {
              remaining.push_back(route[i]);
            }
          }
          // Inserting the pickup at rank and then the delivery at
          // delivery_rank within the route-with-pickup restores the
          // exact current placement.
          placement = utils::addition_eval(input,
                                           job_rank,
                                           input.vehicles[v_rank],
                                           remaining,
                                           rank,
                                           delivery_rank);
        }

        satisfied = task_post_match(input,
                                    job_rank,
                                    v_rank,
                                    placement,
                                    baselines);
      }
    }

    if (satisfied) {
      ++counts.satisfied;
    } else {
      ++counts.unsatisfied;
    }
  }

  return counts;
}

// Authoritative soft-skill preference counts for a complete solution:
// sum of the per-vehicle counts. Single source of truth for final
// reporting (utils/helpers.cpp) and for the cross-thread merge
// (problems/vrp.h), which must never trust memoized thread-local
// counts (Q20). sol_state must be set up for sol. Returns all-zero
// counts for inputs without soft-skills.
template <class Route>
inline SoftSkillPreference
compute_authoritative_preference(const Input& input,
                                 const std::vector<Route>& sol,
                                 const utils::SolutionState& sol_state) {
  SoftSkillPreference totals;
  if (!input.has_soft_skills()) {
    return totals;
  }

  for (Index v_rank = 0; v_rank < sol.size(); ++v_rank) {
    totals += compute_route_preference(input, sol, sol_state, v_rank);
  }

  return totals;
}

} // namespace vroom::ls

#endif
