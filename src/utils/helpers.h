#ifndef HELPERS_H
#define HELPERS_H

/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/typedefs.h"
#include "utils/exceptions.h"

inline cost_t add_without_overflow(cost_t a, cost_t b) {
  if (a > std::numeric_limits<cost_t>::max() - b) {
    throw custom_exception(
      "Too high cost values, stopping to avoid overflowing.");
  }
  return a + b;
}

// Compute cost of adding job with index job_index in given route at
// given rank for vehicle v.
inline gain_t addition_cost(const input& input,
                            const matrix<cost_t>& m,
                            index_t job_rank,
                            const vehicle_t& v,
                            const std::vector<index_t>& route,
                            index_t rank) {
  index_t job_index = input._jobs[job_rank].index();
  gain_t previous_cost = 0;
  gain_t next_cost = 0;
  gain_t old_edge_cost = 0;

  if (rank == route.size()) {
    if (route.size() == 0) {
      // Adding job to an empty route.
      if (v.has_start()) {
        previous_cost = m[v.start.get().index()][job_index];
      }
      if (v.has_end()) {
        next_cost = m[job_index][v.end.get().index()];
      }
    } else {
      // Adding job past the end after a real job.
      auto p_index = input._jobs[route[rank - 1]].index();
      previous_cost = m[p_index][job_index];
      if (v.has_end()) {
        auto n_index = v.end.get().index();
        old_edge_cost = m[p_index][n_index];
        next_cost = m[job_index][n_index];
      }
    }
  } else {
    // Adding before one of the jobs.
    auto n_index = input._jobs[route[rank]].index();
    next_cost = m[job_index][n_index];

    if (rank == 0) {
      if (v.has_start()) {
        auto p_index = v.start.get().index();
        previous_cost = m[p_index][job_index];
        old_edge_cost = m[p_index][n_index];
      }
    } else {
      auto p_index = input._jobs[route[rank - 1]].index();
      previous_cost = m[p_index][job_index];
      old_edge_cost = m[p_index][n_index];
    }
  }

  return previous_cost + next_cost - old_edge_cost;
}

inline solution format_solution(const input& input,
                                const raw_solution& raw_routes) {
  const auto& m = input.get_matrix();

  std::vector<route_t> routes;
  cost_t total_cost = 0;
  duration_t total_service = 0;
  amount_t total_amount(input.amount_size());

  // All job ranks start with unassigned status.
  std::unordered_set<index_t> unassigned_ranks;
  for (unsigned i = 0; i < input._jobs.size(); ++i) {
    unassigned_ranks.insert(i);
  }

  for (std::size_t i = 0; i < raw_routes.size(); ++i) {
    const auto& route = raw_routes[i];
    if (route.empty()) {
      continue;
    }
    const auto& v = input._vehicles[i];

    cost_t cost = 0;
    duration_t service = 0;
    amount_t amount(input.amount_size());

    // Steps for current route.
    std::vector<step> steps;

    // Handle start.
    if (v.has_start()) {
      steps.emplace_back(TYPE::START, v.start.get());
      cost += m[v.start.get().index()][input._jobs[route.front()].index()];
    }

    // Handle jobs.
    index_t previous = route.front();
    assert(input.vehicle_ok_with_job(i, previous));
    steps.emplace_back(input._jobs[previous]);
    service += steps.back().service;
    amount += steps.back().amount;
    unassigned_ranks.erase(previous);

    for (auto it = ++route.cbegin(); it != route.cend(); ++it) {
      cost += m[input._jobs[previous].index()][input._jobs[*it].index()];
      assert(input.vehicle_ok_with_job(i, *it));
      steps.emplace_back(input._jobs[*it]);
      service += steps.back().service;
      amount += steps.back().amount;
      unassigned_ranks.erase(*it);
      previous = *it;
    }

    // Handle end.
    if (v.has_end()) {
      steps.emplace_back(TYPE::END, v.end.get());
      cost += m[input._jobs[route.back()].index()][v.end.get().index()];
    }

    assert(amount <= v.capacity);
    routes.emplace_back(v.id, std::move(steps), cost, service, amount);

    total_cost += cost;
    total_service += service;
    total_amount += amount;
  }

  // Handle unassigned jobs.
  std::vector<job_t> unassigned_jobs;
  std::transform(unassigned_ranks.begin(),
                 unassigned_ranks.end(),
                 std::back_inserter(unassigned_jobs),
                 [&](auto j) { return input._jobs[j]; });

  return solution(0,
                  total_cost,
                  std::move(routes),
                  std::move(unassigned_jobs),
                  total_service,
                  std::move(total_amount));
}

#endif
