/*

This file is part of VROOM.

Copyright (c) 2015-2020, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <thread>
#include <unordered_map>
#include <unordered_set>

#include "algorithms/validation/check.h"
#include "algorithms/validation/choose_ETA.h"
#include "structures/vroom/tw_route.h"
#include "utils/exception.h"
#include "utils/helpers.h"

namespace vroom {
namespace validation {

Solution check_and_set_ETA(const Input& input, unsigned nb_thread) {
  // Keep track of assigned job ranks.
  std::unordered_set<Index> assigned_ranks;

  // Split the work among threads.
  const unsigned nb_vehicles_with_input =
    std::count_if(input.vehicles.begin(),
                  input.vehicles.end(),
                  [](const auto& v) { return !v.input_steps.empty(); });
  const auto nb_buckets = std::min(nb_thread, nb_vehicles_with_input);

  std::vector<std::vector<Index>> thread_ranks(nb_buckets,
                                               std::vector<Index>());

  unsigned actual_route_rank = 0;
  std::unordered_map<Index, Index> v_rank_to_actual_route_rank;
  for (Index v = 0; v < input.vehicles.size(); ++v) {
    const auto& current_vehicle = input.vehicles[v];
    if (current_vehicle.input_steps.empty()) {
      continue;
    }

    for (const auto& step : current_vehicle.input_steps) {
      if (step.type == STEP_TYPE::JOB) {
        assigned_ranks.insert(step.rank);
      }
    }

    thread_ranks[actual_route_rank % nb_buckets].push_back(v);
    v_rank_to_actual_route_rank.insert({v, actual_route_rank});
    ++actual_route_rank;
  }

  std::vector<Route> routes(actual_route_rank);
  std::vector<std::exception_ptr> thread_exceptions(nb_buckets, nullptr);

  auto run_check = [&](unsigned bucket,
                       const std::vector<Index>& vehicle_ranks) {
    for (auto v : vehicle_ranks) {
      auto search = v_rank_to_actual_route_rank.find(v);
      assert(search != v_rank_to_actual_route_rank.end());
      const auto route_rank = search->second;

      try {
        routes[route_rank] =
          choose_ETA(input, v, input.vehicles[v].input_steps);
      } catch (...) {
        thread_exceptions[bucket] = std::current_exception();
      }
    }
  };
  std::vector<std::thread> solving_threads;

  for (unsigned i = 0; i < nb_buckets; ++i) {
    solving_threads.emplace_back(run_check, i, thread_ranks[i]);
  }

  for (unsigned i = 0; i < nb_buckets; ++i) {
    solving_threads[i].join();

    if (thread_exceptions[i] != nullptr) {
      std::rethrow_exception(thread_exceptions[i]);
    }
  }

  // Handle unassigned jobs.
  std::vector<Job> unassigned_jobs;
  for (Index j = 0; j < input.jobs.size(); ++j) {
    if (assigned_ranks.find(j) == assigned_ranks.end()) {
      unassigned_jobs.push_back(input.jobs[j]);
    }
  }

  return Solution(0,
                  input.zero_amount().size(),
                  std::move(routes),
                  std::move(unassigned_jobs));
}

} // namespace validation
} // namespace vroom
