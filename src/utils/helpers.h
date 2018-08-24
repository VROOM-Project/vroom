#ifndef HELPERS_H
#define HELPERS_H

/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/typedefs.h"
#include "structures/vroom/tw_route.h"
#include "utils/exceptions.h"

using tw_solution = std::vector<tw_route>;

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

    duration_t ETA = 0;
    // Handle start.
    if (v.has_start()) {
      steps.emplace_back(TYPE::START, v.start.get());
      steps.back().duration = 0;
      steps.back().arrival = 0;
      auto travel =
        m[v.start.get().index()][input._jobs[route.front()].index()];
      ETA += travel;
      cost += travel;
    }

    // Handle jobs.
    assert(input.vehicle_ok_with_job(i, route.front()));
    steps.emplace_back(input._jobs[route.front()]);

    auto& first = steps.back();
    service += first.service;
    amount += first.amount;

    first.duration = ETA;
    first.arrival = ETA;
    ETA += first.service;
    unassigned_ranks.erase(route.front());

    for (std::size_t r = 0; r < route.size() - 1; ++r) {
      assert(input.vehicle_ok_with_job(i, route[r + 1]));
      duration_t travel =
        m[input._jobs[route[r]].index()][input._jobs[route[r + 1]].index()];
      ETA += travel;
      cost += travel;
      steps.emplace_back(input._jobs[route[r + 1]]);
      auto& current = steps.back();
      service += current.service;
      amount += current.amount;
      current.duration = cost;
      current.arrival = ETA;
      ETA += current.service;
      unassigned_ranks.erase(route[r + 1]);
    }

    // Handle end.
    if (v.has_end()) {
      steps.emplace_back(TYPE::END, v.end.get());
      duration_t travel =
        m[input._jobs[route.back()].index()][v.end.get().index()];
      ETA += travel;
      cost += travel;
      steps.back().duration = cost;
      steps.back().arrival = ETA;
    }

    assert(amount <= v.capacity);
    routes.emplace_back(v.id, std::move(steps), cost, service, amount);
    routes.back().duration = cost;

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

inline solution format_solution(const input& input,
                                const tw_solution& tw_routes) {
  raw_solution raw_sol;
  raw_sol.reserve(tw_routes.size());

  std::transform(tw_routes.begin(),
                 tw_routes.end(),
                 std::back_inserter(raw_sol),
                 [](const auto& tw_r) { return tw_r.route; });

  auto sol = format_solution(input, raw_sol);

  const auto& m = input.get_matrix();

  duration_t total_waiting_time = 0;

  for (std::size_t i = 0; i < sol.routes.size(); ++i) {
    // TW ETA logic: use earliest possible arrival for last job then
    // "push" all previous steps forward to pack the route and
    // minimize waiting times.
    auto& route = sol.routes[i];
    const auto& tw_r = tw_routes[i];
    const auto& v = tw_r.v;

    duration_t waiting_time = 0;

    duration_t ETA = tw_r.earliest.back();
    // s and r respectively hold current index in route.steps and
    // tw_r.route.
    std::size_t s = route.steps.size() - 1;
    std::size_t r = tw_r.route.size() - 1;

    if (v.has_end()) {
      assert(route.steps[s].type == TYPE::END);

      const auto& last_job = input._jobs[tw_r.route[r]];
      duration_t end_ETA =
        ETA + last_job.service + m[last_job.index()][v.end.get().index()];
      assert(v.tw.contains(end_ETA));
      route.steps[s].arrival = end_ETA;
      --s;
    }
    route.steps[s].arrival = ETA;

    assert(r <= s);
    for (; r > 0; --r, --s) {
      assert(route.steps[s - 1].type == TYPE::JOB);

      const auto& current_job = input._jobs[tw_r.route[r]];
      const auto& previous_job = input._jobs[tw_r.route[r - 1]];

      duration_t diff =
        previous_job.service + m[previous_job.index()][current_job.index()];
      assert(diff <= ETA);
      duration_t candidate_ETA = ETA - diff;
      assert(tw_r.earliest[r - 1] <= candidate_ETA);

      ETA = std::min(candidate_ETA, tw_r.latest[r - 1]);
      if (ETA < candidate_ETA) {
        duration_t wt = candidate_ETA - ETA;
        route.steps[s].waiting_time = wt;
        waiting_time += wt;
      }
      assert(previous_job.is_valid_arrival(ETA));
      route.steps[s - 1].arrival = ETA;
    }

    if (v.has_start()) {
      assert(s == 1);
      assert(route.steps[0].type == TYPE::START);
      const auto& current_job = input._jobs[tw_r.route[0]];
      duration_t diff = m[v.start.get().index()][current_job.index()];
      assert(diff <= ETA);
      ETA -= diff;
      assert(v.tw.contains(ETA));
      route.steps[0].arrival = ETA;
    } else {
      assert(s == 0);
    }

    route.waiting_time = waiting_time;
    total_waiting_time += waiting_time;

    assert(route.steps.back().arrival - route.steps.front().arrival ==
           route.duration + route.service + route.waiting_time);
  }

  sol.summary.waiting_time = total_waiting_time;

  return sol;
}

#endif
