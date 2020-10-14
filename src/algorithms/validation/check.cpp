/*

This file is part of VROOM.

Copyright (c) 2015-2019, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <algorithm>
#include <numeric>
#include <set>

#include "algorithms/validation/check.h"
#include "structures/vroom/tw_route.h"
#include "utils/exception.h"
#include "utils/helpers.h"

namespace vroom {
namespace validation {

Solution check_and_set_ETA(const Input& input) {
  std::vector<TWRoute> tw_routes;
  std::set<Index> unassigned;
  for (Index j = 0; j < input.jobs.size(); ++j) {
    unassigned.insert(j);
  }

  for (Index v = 0; v < input.vehicles.size(); ++v) {
    tw_routes.emplace_back(input, v);
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

    // TODO check for route amount and skills.

    // Populate new route by adding jobs if valid for TW.
    auto& current_r = tw_routes[v];

    for (const auto job_rank : job_ranks) {
      const Index rank = current_r.route.size();
      if (current_r.is_valid_addition_for_tw(input, job_rank, rank)) {
        current_r.add(input, job_rank, rank);
      }
    }
  }

  return utils::format_solution(input, tw_routes);
}

} // namespace validation
} // namespace vroom
