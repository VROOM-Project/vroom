/*

This file is part of VROOM.

Copyright (c) 2015-2019, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <unordered_set>

#include "algorithms/validation/check.h"
#include "algorithms/validation/choose_invalid.h"
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

    // Get job ranks from ids.
    std::vector<Index> job_ranks;
    for (auto step : current_vehicle.input_steps) {
      switch (step.type) {
      case JOB_TYPE::SINGLE: {
        auto search = input.job_id_to_rank.find(step.id);
        if (search == input.job_id_to_rank.end()) {
          throw Exception(ERROR::INPUT,
                          "Invalid job id" + std::to_string(step.id) + " .");
        }
        job_ranks.push_back(search->second);
        break;
      }
      case JOB_TYPE::PICKUP: {
        auto search = input.pickup_id_to_rank.find(step.id);
        if (search == input.pickup_id_to_rank.end()) {
          throw Exception(ERROR::INPUT,
                          "Invalid pickup id" + std::to_string(step.id) + " .");
        }
        job_ranks.push_back(search->second);
        break;
      }
      case JOB_TYPE::DELIVERY: {
        auto search = input.delivery_id_to_rank.find(step.id);
        if (search == input.delivery_id_to_rank.end()) {
          throw Exception(ERROR::INPUT,
                          "Invalid delivery id" + std::to_string(step.id) +
                            " .");
        }
        job_ranks.push_back(search->second);
        break;
      }
      }
    }

    // TODO generate route directly if input steps make it a valid
    // one.

    routes.push_back(
      choose_invalid_route(input, v, job_ranks, unassigned_ranks));
    for (auto rank : job_ranks) {
      unassigned_ranks.erase(rank);
    }
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
