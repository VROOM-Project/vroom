/*

This file is part of VROOM.

Copyright (c) 2015-2024, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <algorithm>
#include <mutex>
#include <thread>
#include <unordered_set>
#include <vector>

#include "algorithms/validation/check.h"
#include "algorithms/validation/choose_ETA.h"
#include "structures/vroom/tw_route.h"
#include "utils/exception.h"
#include "utils/helpers.h"

namespace vroom::validation {

Solution
check_and_set_ETA(const Input& input,
                  unsigned nb_thread,
                  std::unordered_map<Index, Index>& route_rank_to_v_rank) {
  // Keep track of assigned job ranks.
  std::unordered_set<Index> assigned_ranks;

  // Split the work among threads.
  const unsigned nb_vehicles_with_input =
    std::ranges::count_if(input.vehicles,
                          [](const auto& v) { return !v.steps.empty(); });
  const auto nb_buckets = std::min(nb_thread, nb_vehicles_with_input);

  std::vector<std::vector<Index>> thread_ranks(nb_buckets,
                                               std::vector<Index>());

  unsigned actual_route_rank = 0;
  std::unordered_map<Index, Index> v_rank_to_actual_route_rank;
  for (Index v = 0; v < input.vehicles.size(); ++v) {
    const auto& current_vehicle = input.vehicles[v];
    if (current_vehicle.steps.empty()) {
      continue;
    }

    for (const auto& step : current_vehicle.steps) {
      if (step.type == STEP_TYPE::JOB) {
        assigned_ranks.insert(step.rank);
      }
    }

    thread_ranks[actual_route_rank % nb_buckets].push_back(v);
    v_rank_to_actual_route_rank.insert({v, actual_route_rank});
    route_rank_to_v_rank.insert({actual_route_rank, v});
    ++actual_route_rank;
  }

  std::vector<Route> routes(actual_route_rank);

  std::exception_ptr ep = nullptr;
  std::mutex ep_m;

  auto run_check = [&](const std::vector<Index>& vehicle_ranks) {
    try {
      for (auto v : vehicle_ranks) {
        auto search = v_rank_to_actual_route_rank.find(v);
        assert(search != v_rank_to_actual_route_rank.end());
        const auto route_rank = search->second;

        routes[route_rank] = choose_ETA(input, v, input.vehicles[v].steps);
      }
    } catch (...) {
      std::scoped_lock<std::mutex> lock(ep_m);
      ep = std::current_exception();
    }
  };

  std::vector<std::jthread> solving_threads;
  solving_threads.reserve(thread_ranks.size());

  for (const auto& v_ranks : thread_ranks) {
    solving_threads.emplace_back(run_check, v_ranks);
  }

  for (auto& t : solving_threads) {
    t.join();
  }

  if (ep != nullptr) {
    std::rethrow_exception(ep);
  }

  // Handle unassigned jobs.
  std::vector<Job> unassigned_jobs;
  unassigned_jobs.reserve(input.jobs.size() - assigned_ranks.size());

  for (Index j = 0; j < input.jobs.size(); ++j) {
    if (!assigned_ranks.contains(j)) {
      unassigned_jobs.push_back(input.jobs[j]);
    }
  }

  return Solution(input.zero_amount(),
                  std::move(routes),
                  std::move(unassigned_jobs));
}

} // namespace vroom::validation
