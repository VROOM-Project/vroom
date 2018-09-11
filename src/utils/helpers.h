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
    routes.emplace_back(v.id, std::move(steps), cost, service, cost, 0, amount);
  }

  // Handle unassigned jobs.
  std::vector<job_t> unassigned_jobs;
  std::transform(unassigned_ranks.begin(),
                 unassigned_ranks.end(),
                 std::back_inserter(unassigned_jobs),
                 [&](auto j) { return input._jobs[j]; });

  return solution(0,
                  input.amount_size(),
                  std::move(routes),
                  std::move(unassigned_jobs));
}

inline solution format_solution(const input& input,
                                const tw_solution& tw_routes) {
  const auto& m = input.get_matrix();

  std::vector<route_t> routes;

  // All job ranks start with unassigned status.
  std::unordered_set<index_t> unassigned_ranks;
  for (unsigned i = 0; i < input._jobs.size(); ++i) {
    unassigned_ranks.insert(i);
  }

  for (const auto& tw_r : tw_routes) {
    const auto& v = tw_r.v;
    if (tw_r.route.empty()) {
      continue;
    }

    // ETA logic: aim at earliest possible arrival for last job then
    // determine latest possible start time in order to minimize
    // waiting times.
    duration_t job_start = tw_r.earliest.back();
    duration_t backward_wt = 0;
    for (std::size_t r = tw_r.route.size() - 1; r > 0; --r) {
      const auto& current_job = input._jobs[tw_r.route[r]];
      const auto& previous_job = input._jobs[tw_r.route[r - 1]];

      duration_t diff =
        previous_job.service + m[previous_job.index()][current_job.index()];

      assert(diff <= job_start);
      duration_t candidate_start = job_start - diff;
      assert(tw_r.earliest[r - 1] <= candidate_start);

      job_start = std::min(candidate_start, tw_r.latest[r - 1]);
      if (job_start < candidate_start) {
        backward_wt += (candidate_start - job_start);
      }
      assert(previous_job.is_valid_start(job_start));
    }

    cost_t cost = 0;
    duration_t service = 0;
    amount_t amount(input.amount_size());
    // Steps for current route.
    std::vector<step> steps;

    // Now pack everything ASAP based on first job start date.
    if (v.has_start()) {
      steps.emplace_back(TYPE::START, v.start.get());
      steps.back().duration = 0;

      const auto& first_job = input._jobs[tw_r.route[0]];
      duration_t diff = m[v.start.get().index()][first_job.index()];
      cost += diff;

      assert(diff <= job_start);
      auto v_start = job_start - diff;
      assert(v.tw.contains(v_start));
      steps.back().arrival = v_start;
    }

    steps.emplace_back(input._jobs[tw_r.route.front()]);
    auto& first = steps.back();
    service += first.service;
    amount += first.amount;

    first.duration = cost;
    first.arrival = job_start;
    unassigned_ranks.erase(tw_r.route.front());

    duration_t forward_wt = 0;
    for (std::size_t r = 0; r < tw_r.route.size() - 1; ++r) {
      const auto& previous_job = input._jobs[tw_r.route[r]];
      const auto& next_job = input._jobs[tw_r.route[r + 1]];

      duration_t travel = m[previous_job.index()][next_job.index()];
      cost += travel;

      steps.emplace_back(input._jobs[tw_r.route[r + 1]]);
      auto& current = steps.back();
      service += current.service;
      amount += current.amount;
      current.duration = cost;

      duration_t start_candidate = job_start + previous_job.service + travel;
      assert(start_candidate <= tw_r.latest[r + 1]);

      current.arrival = start_candidate;
      job_start = std::max(start_candidate, tw_r.earliest[r + 1]);

      if (start_candidate < tw_r.earliest[r + 1]) {
        duration_t wt = tw_r.earliest[r + 1] - start_candidate;
        current.waiting_time = wt;
        forward_wt += wt;
      }
      assert(next_job.is_valid_start(current.arrival + current.waiting_time));

      unassigned_ranks.erase(tw_r.route[r + 1]);
    }

    if (v.has_end()) {
      const auto& last_job = input._jobs[tw_r.route.back()];
      duration_t travel = m[last_job.index()][v.end.get().index()];
      cost += travel;

      steps.emplace_back(TYPE::END, v.end.get());
      steps.back().duration = cost;

      duration_t v_end = job_start + last_job.service + travel;
      assert(v.tw.contains(v_end));
      steps.back().arrival = v_end;
    }

    assert(amount <= v.capacity);
    assert(forward_wt == backward_wt);
    assert(steps.back().arrival + steps.back().waiting_time +
             steps.back().service - steps.front().arrival ==
           cost + service + forward_wt);

    routes.emplace_back(v.id,
                        std::move(steps),
                        cost,
                        service,
                        cost,
                        forward_wt,
                        amount);
  }

  // Handle unassigned jobs.
  std::vector<job_t> unassigned_jobs;
  std::transform(unassigned_ranks.begin(),
                 unassigned_ranks.end(),
                 std::back_inserter(unassigned_jobs),
                 [&](auto j) { return input._jobs[j]; });

  return solution(0,
                  input.amount_size(),
                  std::move(routes),
                  std::move(unassigned_jobs));
}

inline raw_solution to_raw_solution(const tw_solution& tw_sol) {
  raw_solution result;
  std::transform(tw_sol.begin(),
                 tw_sol.end(),
                 std::back_inserter(result),
                 [](const auto& tw_r) { return tw_r.route; });

  return result;
}

#endif
