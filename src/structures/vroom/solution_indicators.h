#ifndef SOLUTION_INDICATORS_H
#define SOLUTION_INDICATORS_H

/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <algorithm>
#include <tuple>

#include "structures/typedefs.h"
#include "structures/vroom/input/input.h"
#include "utils/helpers.h"
#include "utils/soft_skills.h"

namespace vroom::utils {

struct SolutionIndicators {
  Priority priority_sum{0};
  // Number of assigned jobs whose soft skills are matched by the
  // vehicle serving them (raw status-quo match, same predicate as
  // the operators' pre-move status). Remains 0 for inputs without
  // any soft-skills, so lexicographic comparisons that factor it in
  // degrade to the plain ordering on those inputs.
  uint32_t preference{0};
  unsigned assigned{0};
  Eval eval;
  unsigned used_vehicles{0};
  // Hash based on the ordered sizes of routes in the solution.
  uint32_t routes_hash;

  SolutionIndicators() = default;

  template <class Route>
  SolutionIndicators(const Input& input, const std::vector<Route>& sol)
    : SolutionIndicators() {
    const auto& opts = input.get_soft_skills_options();
    const bool has_soft_skills = input.has_soft_skills();

    Index v_rank = 0;
    for (const auto& r : sol) {
      priority_sum += utils::priority_sum_for_route(input, r.route);
      assigned += r.route.size();

      if (has_soft_skills) {
        // Status-quo preference count: each job of the route served
        // by its own vehicle under that vehicle's soft skills.
        const auto& vehicle_skills = input.vehicles[v_rank].soft_skills;
        for (const auto j_rank : r.route) {
          const auto& job = input.jobs[j_rank];
          if (soft_skill_match(job.soft_skills,
                               vehicle_skills,
                               effective_match_mode(job, opts))) {
            ++preference;
          }
        }
      }

      eval += utils::route_eval_for_vehicle(input, v_rank, r.route);
      ++v_rank;

      if (!r.empty()) {
        used_vehicles += 1;
      }
    }

    std::vector<uint32_t> routes_sizes;
    routes_sizes.reserve(sol.size());
    std::ranges::transform(sol,
                           std::back_inserter(routes_sizes),
                           [](const auto& r) { return std::size(r); });
    std::ranges::sort(routes_sizes);
    routes_hash = get_vector_hash(routes_sizes);
  }

  friend bool operator<(const SolutionIndicators& lhs,
                        const SolutionIndicators& rhs) {
    return std::tie(rhs.priority_sum,
                    rhs.assigned,
                    lhs.eval.cost,
                    lhs.used_vehicles,
                    lhs.eval.duration,
                    lhs.eval.distance,
                    lhs.routes_hash) < std::tie(lhs.priority_sum,
                                                lhs.assigned,
                                                rhs.eval.cost,
                                                rhs.used_vehicles,
                                                rhs.eval.duration,
                                                rhs.eval.distance,
                                                rhs.routes_hash);
  }
};

} // namespace vroom::utils

#endif
