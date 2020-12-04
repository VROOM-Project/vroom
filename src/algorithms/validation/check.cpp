/*

This file is part of VROOM.

Copyright (c) 2015-2019, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <unordered_set>

#include "algorithms/validation/check.h"
#include "algorithms/validation/choose_ETA.h"
#include "structures/vroom/tw_route.h"
#include "utils/exception.h"
#include "utils/helpers.h"

namespace vroom {
namespace validation {

Solution check_and_set_ETA(const Input& input) {
  std::vector<Route> routes;
  std::unordered_set<Index> unassigned_ranks;

  for (Index j = 0; j < input.jobs.size(); ++j) {
    unassigned_ranks.insert(j);
  }

  for (Index v = 0; v < input.vehicles.size(); ++v) {
    const auto& current_vehicle = input.vehicles[v];
    if (current_vehicle.input_steps.empty()) {
      continue;
    }

    routes.push_back(
      choose_ETA(input, v, current_vehicle.input_steps, unassigned_ranks));
  }

  // Handle unassigned jobs.
  std::vector<Job> unassigned_jobs;
  std::transform(unassigned_ranks.begin(),
                 unassigned_ranks.end(),
                 std::back_inserter(unassigned_jobs),
                 [&](auto j) { return input.jobs[j]; });

  return Solution(0,
                  input.zero_amount().size(),
                  std::move(routes),
                  std::move(unassigned_jobs));
}

} // namespace validation
} // namespace vroom
