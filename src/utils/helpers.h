#ifndef HELPERS_H
#define HELPERS_H

/*

This file is part of VROOM.

Copyright (c) 2015-2019, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <sstream>

#include "structures/typedefs.h"
#include "structures/vroom/raw_route.h"
#include "structures/vroom/tw_route.h"
#include "utils/exception.h"

namespace vroom {
namespace utils {

using RawSolution = std::vector<RawRoute>;
using TWSolution = std::vector<TWRoute>;

inline Cost add_without_overflow(Cost a, Cost b) {
  if (a > std::numeric_limits<Cost>::max() - b) {
    throw Exception(ERROR::INPUT,
                    "Too high cost values, stopping to avoid overflowing.");
  }
  return a + b;
}

inline INIT get_init(const std::string& s) {
  if (s == "NONE") {
    return INIT::NONE;
  } else if (s == "HIGHER_AMOUNT") {
    return INIT::HIGHER_AMOUNT;
  } else if (s == "NEAREST") {
    return INIT::NEAREST;
  } else if (s == "FURTHEST") {
    return INIT::FURTHEST;
  } else if (s == "EARLIEST_DEADLINE") {
    return INIT::EARLIEST_DEADLINE;
  } else {
    throw Exception(ERROR::INPUT,
                    "Invalid heuristic parameter in command-line.");
  }
}

inline HeuristicParameters str_to_heuristic_param(const std::string& s) {
  // Split command-line string describing parameters.
  constexpr char delimiter = ',';
  std::vector<std::string> tokens;
  std::string token;
  std::istringstream tokenStream(s);
  while (std::getline(tokenStream, token, delimiter)) {
    tokens.push_back(token);
  }

  if (tokens.size() != 3 or tokens[0].size() != 1) {
    throw Exception(ERROR::INPUT,
                    "Invalid heuristic parameter in command-line.");
  }

  auto init = get_init(tokens[1]);
  try {
    auto h = std::stoul(tokens[0]);

    if (h != 0 and h != 1) {
      throw Exception(ERROR::INPUT,
                      "Invalid heuristic parameter in command-line.");
    }

    auto regret_coeff = std::stof(tokens[2]);
    if (regret_coeff < 0) {
      throw Exception(ERROR::INPUT,
                      "Invalid heuristic parameter in command-line.");
    }

    return HeuristicParameters(static_cast<HEURISTIC>(h), init, regret_coeff);
  } catch (const std::exception& e) {
    throw Exception(ERROR::INPUT,
                    "Invalid heuristic parameter in command-line.");
  }
}

// Compute cost of adding job with rank job_rank in given route at
// given rank for vehicle v.
inline Gain addition_cost(const Input& input,
                          const Matrix<Cost>& m,
                          Index job_rank,
                          const Vehicle& v,
                          const std::vector<Index>& route,
                          Index rank) {
  Index job_index = input.jobs[job_rank].index();
  Gain previous_cost = 0;
  Gain next_cost = 0;
  Gain old_edge_cost = 0;

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
      auto p_index = input.jobs[route[rank - 1]].index();
      previous_cost = m[p_index][job_index];
      if (v.has_end()) {
        auto n_index = v.end.get().index();
        old_edge_cost = m[p_index][n_index];
        next_cost = m[job_index][n_index];
      }
    }
  } else {
    // Adding before one of the jobs.
    auto n_index = input.jobs[route[rank]].index();
    next_cost = m[job_index][n_index];

    if (rank == 0) {
      if (v.has_start()) {
        auto p_index = v.start.get().index();
        previous_cost = m[p_index][job_index];
        old_edge_cost = m[p_index][n_index];
      }
    } else {
      auto p_index = input.jobs[route[rank - 1]].index();
      previous_cost = m[p_index][job_index];
      old_edge_cost = m[p_index][n_index];
    }
  }

  return previous_cost + next_cost - old_edge_cost;
}

inline Cost route_cost_for_vehicle(const Input& input,
                                   Index vehicle_rank,
                                   const std::vector<Index>& route) {
  const auto& v = input.vehicles[vehicle_rank];
  const auto& m = input.get_matrix();
  auto cost = 0;

  if (route.size() > 0) {
    if (v.has_start()) {
      cost += m[v.start.get().index()][input.jobs[route.front()].index()];
    }

    Index previous = route.front();
    for (auto it = ++route.cbegin(); it != route.cend(); ++it) {
      cost += m[input.jobs[previous].index()][input.jobs[*it].index()];
      previous = *it;
    }

    if (v.has_end()) {
      cost += m[input.jobs[route.back()].index()][v.end.get().index()];
    }
  }

  return cost;
}

inline Solution format_solution(const Input& input,
                                const RawSolution& raw_routes) {
  const auto& m = input.get_matrix();

  std::vector<Route> routes;

  // All job ranks start with unassigned status.
  std::unordered_set<Index> unassigned_ranks;
  for (unsigned i = 0; i < input.jobs.size(); ++i) {
    unassigned_ranks.insert(i);
  }

  for (std::size_t i = 0; i < raw_routes.size(); ++i) {
    const auto& route = raw_routes[i].route;
    if (route.empty()) {
      continue;
    }
    const auto& v = input.vehicles[i];

    Cost cost = 0;
    Duration service = 0;
    Amount amount(input.amount_size());

    // Steps for current route.
    std::vector<Step> steps;

    Duration ETA = 0;
    // Handle start.
    if (v.has_start()) {
      steps.emplace_back(STEP_TYPE::START, v.start.get());
      steps.back().duration = 0;
      steps.back().arrival = 0;
      auto travel = m[v.start.get().index()][input.jobs[route.front()].index()];
      ETA += travel;
      cost += travel;
    }

    // Handle jobs.
    assert(input.vehicle_ok_with_job(i, route.front()));
    steps.emplace_back(input.jobs[route.front()]);

    auto& first = steps.back();
    service += first.service;
    amount += first.amount;

    first.duration = ETA;
    first.arrival = ETA;
    ETA += first.service;
    unassigned_ranks.erase(route.front());

    for (std::size_t r = 0; r < route.size() - 1; ++r) {
      assert(input.vehicle_ok_with_job(i, route[r + 1]));
      Duration travel =
        m[input.jobs[route[r]].index()][input.jobs[route[r + 1]].index()];
      ETA += travel;
      cost += travel;

      steps.emplace_back(input.jobs[route[r + 1]]);
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
      steps.emplace_back(STEP_TYPE::END, v.end.get());
      Duration travel =
        m[input.jobs[route.back()].index()][v.end.get().index()];
      ETA += travel;
      cost += travel;
      steps.back().duration = cost;
      steps.back().arrival = ETA;
    }

    assert(amount <= v.capacity);
    routes.emplace_back(v.id, std::move(steps), cost, service, cost, 0, amount);
  }

  // Handle unassigned jobs.
  std::vector<Job> unassigned_jobs;
  std::transform(unassigned_ranks.begin(),
                 unassigned_ranks.end(),
                 std::back_inserter(unassigned_jobs),
                 [&](auto j) { return input.jobs[j]; });

  return Solution(0,
                  input.amount_size(),
                  std::move(routes),
                  std::move(unassigned_jobs));
}

inline Route format_route(const Input& input,
                          const TWRoute& tw_r,
                          std::unordered_set<Index>& unassigned_ranks) {
  const auto& m = input.get_matrix();
  const auto& v = input.vehicles[tw_r.vehicle_rank];

  // ETA logic: aim at earliest possible arrival for last job then
  // determine latest possible start time in order to minimize waiting
  // times.
  Duration job_start = tw_r.earliest.back();
  Duration backward_wt = 0;
  for (std::size_t r = tw_r.route.size() - 1; r > 0; --r) {
    const auto& current_job = input.jobs[tw_r.route[r]];
    const auto& previous_job = input.jobs[tw_r.route[r - 1]];

    Duration diff =
      previous_job.service + m[previous_job.index()][current_job.index()];

    assert(diff <= job_start);
    Duration candidate_start = job_start - diff;
    assert(tw_r.earliest[r - 1] <= candidate_start);

    job_start = std::min(candidate_start, tw_r.latest[r - 1]);
    if (job_start < candidate_start) {
      backward_wt += (candidate_start - job_start);
    }
    assert(previous_job.is_valid_start(job_start));
  }

  Cost cost = 0;
  Duration service = 0;
  Amount amount(input.amount_size());
  // Steps for current route.
  std::vector<Step> steps;

  // Now pack everything ASAP based on first job start date.
  if (v.has_start()) {
    steps.emplace_back(STEP_TYPE::START, v.start.get());
    steps.back().duration = 0;

    const auto& first_job = input.jobs[tw_r.route[0]];
    Duration diff = m[v.start.get().index()][first_job.index()];
    cost += diff;

    assert(diff <= job_start);
    auto v_start = job_start - diff;
    assert(v.tw.contains(v_start));
    steps.back().arrival = v_start;
  }

  assert(input.vehicle_ok_with_job(tw_r.vehicle_rank, tw_r.route.front()));
  steps.emplace_back(input.jobs[tw_r.route.front()]);
  auto& first = steps.back();
  service += first.service;
  amount += first.amount;

  first.duration = cost;
  first.arrival = job_start;
  unassigned_ranks.erase(tw_r.route.front());

  Duration forward_wt = 0;
  for (std::size_t r = 0; r < tw_r.route.size() - 1; ++r) {
    assert(input.vehicle_ok_with_job(tw_r.vehicle_rank, tw_r.route[r + 1]));
    const auto& previous_job = input.jobs[tw_r.route[r]];
    const auto& next_job = input.jobs[tw_r.route[r + 1]];

    Duration travel = m[previous_job.index()][next_job.index()];
    cost += travel;

    steps.emplace_back(input.jobs[tw_r.route[r + 1]]);
    auto& current = steps.back();
    service += current.service;
    amount += current.amount;
    current.duration = cost;

    Duration start_candidate = job_start + previous_job.service + travel;
    assert(start_candidate <= tw_r.latest[r + 1]);

    current.arrival = start_candidate;
    job_start = std::max(start_candidate, tw_r.earliest[r + 1]);

    if (start_candidate < tw_r.earliest[r + 1]) {
      Duration wt = tw_r.earliest[r + 1] - start_candidate;
      current.waiting_time = wt;
      forward_wt += wt;
    }
    assert(next_job.is_valid_start(current.arrival + current.waiting_time));

    unassigned_ranks.erase(tw_r.route[r + 1]);
  }

  if (v.has_end()) {
    const auto& last_job = input.jobs[tw_r.route.back()];
    Duration travel = m[last_job.index()][v.end.get().index()];
    cost += travel;

    steps.emplace_back(STEP_TYPE::END, v.end.get());
    steps.back().duration = cost;

    Duration v_end = job_start + last_job.service + travel;
    assert(v.tw.contains(v_end));
    steps.back().arrival = v_end;
  }

  assert(amount <= v.capacity);
  assert(forward_wt == backward_wt);
  assert(steps.back().arrival + steps.back().waiting_time +
           steps.back().service - steps.front().arrival ==
         cost + service + forward_wt);

  return Route(v.id, std::move(steps), cost, service, cost, forward_wt, amount);
}

inline Solution format_solution(const Input& input,
                                const TWSolution& tw_routes) {
  std::vector<Route> routes;

  // All job ranks start with unassigned status.
  std::unordered_set<Index> unassigned_ranks;
  for (unsigned i = 0; i < input.jobs.size(); ++i) {
    unassigned_ranks.insert(i);
  }

  for (const auto& tw_route : tw_routes) {
    if (tw_route.route.empty()) {
      continue;
    }

    routes.push_back(format_route(input, tw_route, unassigned_ranks));
  }

  // Handle unassigned jobs.
  std::vector<Job> unassigned_jobs;
  std::transform(unassigned_ranks.begin(),
                 unassigned_ranks.end(),
                 std::back_inserter(unassigned_jobs),
                 [&](auto j) { return input.jobs[j]; });

  return Solution(0,
                  input.amount_size(),
                  std::move(routes),
                  std::move(unassigned_jobs));
}

inline unsigned get_code(ERROR e) {
  unsigned code = 0;
  switch (e) {
  case ERROR::INTERNAL:
    code = 1;
    break;
  case ERROR::INPUT:
    code = 2;
    break;
  case ERROR::ROUTING:
    code = 3;
    break;
  }

  return code;
}

} // namespace utils
} // namespace vroom

#endif
