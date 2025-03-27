#ifndef HELPERS_H
#define HELPERS_H

/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <optional>
#include <string>
#include <vector>

#include "structures/typedefs.h"
#include "structures/vroom/raw_route.h"
#include "structures/vroom/tw_route.h"
#include "utils/exception.h"

namespace vroom::utils {

template <typename T> T round(double value) {
  constexpr double round_increment = 0.5;
  return static_cast<T>(value + round_increment);
}

TimePoint now();

Amount max_amount(std::size_t size);

inline UserCost add_without_overflow(UserCost a, UserCost b) {
  if (a > std::numeric_limits<UserCost>::max() - b) {
    throw InputException(
      "Too high cost values, stopping to avoid overflowing.");
  }
  return a + b;
}

// Taken from https://stackoverflow.com/a/72073933.
inline uint32_t get_vector_hash(const std::vector<uint32_t>& vec) {
  uint32_t seed = vec.size();
  for (auto x : vec) {
    x = ((x >> 16) ^ x) * 0x45d9f3b;
    x = ((x >> 16) ^ x) * 0x45d9f3b;
    x = (x >> 16) ^ x;
    seed ^= x + 0x9e3779b9 + (seed << 6) + (seed >> 2);
  }
  return seed;
}

inline unsigned get_depth(unsigned exploration_level) {
  return exploration_level;
}

inline unsigned get_nb_searches(unsigned exploration_level) {
  assert(exploration_level <= MAX_EXPLORATION_LEVEL);

  unsigned nb_searches = 4 * (exploration_level + 1);
  if (exploration_level >= 4) {
    nb_searches += 4;
  }
  if (exploration_level == MAX_EXPLORATION_LEVEL) {
    nb_searches += 4;
  }

  return nb_searches;
}

INIT get_init(std::string_view s);

SORT get_sort(std::string_view s);

#ifdef LOG_LS_OPERATORS
void log_LS_operators(
  const std::vector<std::array<ls::OperatorStats, OperatorName::MAX>>&
    ls_stats);
#endif

HeuristicParameters str_to_heuristic_param(const std::string& s);

// Evaluate adding job with rank job_rank in given route at given rank
// for vehicle v.
inline Eval addition_cost(const Input& input,
                          Index job_rank,
                          const Vehicle& v,
                          const std::vector<Index>& route,
                          Index rank) {
  assert(rank <= route.size());

  const Index job_index = input.jobs[job_rank].index();
  Eval previous_eval;
  Eval next_eval;
  Eval old_edge_eval;

  if (rank == route.size()) {
    if (route.empty()) {
      if (v.has_start()) {
        previous_eval = v.eval(v.start.value().index(), job_index);
      }
      if (v.has_end()) {
        next_eval = v.eval(job_index, v.end.value().index());
      }
    } else {
      // Adding job past the end after a real job.
      auto p_index = input.jobs[route[rank - 1]].index();
      previous_eval = v.eval(p_index, job_index);
      if (v.has_end()) {
        auto n_index = v.end.value().index();
        old_edge_eval = v.eval(p_index, n_index);
        next_eval = v.eval(job_index, n_index);
      }
    }
  } else {
    // Adding before one of the jobs.
    auto n_index = input.jobs[route[rank]].index();
    next_eval = v.eval(job_index, n_index);

    if (rank == 0) {
      if (v.has_start()) {
        auto p_index = v.start.value().index();
        previous_eval = v.eval(p_index, job_index);
        old_edge_eval = v.eval(p_index, n_index);
      }
    } else {
      auto p_index = input.jobs[route[rank - 1]].index();
      previous_eval = v.eval(p_index, job_index);
      old_edge_eval = v.eval(p_index, n_index);
    }
  }

  return previous_eval + next_eval - old_edge_eval;
}

// Evaluate adding pickup with rank job_rank and associated delivery
// (with rank job_rank + 1) in given route for vehicle v. Pickup is
// inserted at pickup_rank in route and delivery is inserted at
// delivery_rank in route **with pickup**.
inline Eval addition_cost(const Input& input,
                          Index job_rank,
                          const Vehicle& v,
                          const std::vector<Index>& route,
                          Index pickup_rank,
                          Index delivery_rank) {
  assert(pickup_rank < delivery_rank && delivery_rank <= route.size() + 1);

  // Start with pickup eval.
  auto eval = addition_cost(input, job_rank, v, route, pickup_rank);

  if (delivery_rank == pickup_rank + 1) {
    // Delivery is inserted just after pickup.
    const Index p_index = input.jobs[job_rank].index();
    const Index d_index = input.jobs[job_rank + 1].index();
    eval += v.eval(p_index, d_index);

    Eval after_delivery;
    Eval remove_after_pickup;

    if (pickup_rank == route.size()) {
      // Addition at the end of a route.
      if (v.has_end()) {
        after_delivery = v.eval(d_index, v.end.value().index());
        remove_after_pickup = v.eval(p_index, v.end.value().index());
      }
    } else {
      // There is a job after insertion.
      const Index next_index = input.jobs[route[pickup_rank]].index();
      after_delivery = v.eval(d_index, next_index);
      remove_after_pickup = v.eval(p_index, next_index);
    }

    eval += after_delivery;
    eval -= remove_after_pickup;
  } else {
    // Delivery is further away so edges sets for pickup and delivery
    // addition are disjoint.
    eval += addition_cost(input, job_rank + 1, v, route, delivery_rank - 1);
  }

  return eval;
}

inline Eval max_edge_eval(const Input& input,
                          const Vehicle& v,
                          const std::vector<Index>& route) {
  Eval max_eval;

  if (!route.empty()) {
    if (v.has_start()) {
      const auto start_to_first =
        v.eval(v.start.value().index(), input.jobs[route.front()].index());
      max_eval = std::max(max_eval, start_to_first);
    }

    for (std::size_t i = 0; i < route.size() - 1; ++i) {
      const auto job_to_next =
        v.eval(input.jobs[route[i]].index(), input.jobs[route[i + 1]].index());
      max_eval = std::max(max_eval, job_to_next);
    }

    if (v.has_end()) {
      const auto last_to_end =
        v.eval(input.jobs[route.back()].index(), v.end.value().index());
      max_eval = std::max(max_eval, last_to_end);
    }
  }

  return max_eval;
}

// Helper function for SwapStar operator, computing part of the eval
// for in-place replacing of job at rank in route with job at
// job_rank.
inline Eval in_place_delta_cost(const Input& input,
                                Index job_rank,
                                const Vehicle& v,
                                const std::vector<Index>& route,
                                Index rank) {
  assert(!route.empty());
  const Index new_index = input.jobs[job_rank].index();

  Eval new_previous_eval;
  Eval new_next_eval;
  std::optional<Index> p_index;
  std::optional<Index> n_index;

  if (rank == 0) {
    if (v.has_start()) {
      p_index = v.start.value().index();
      new_previous_eval = v.eval(p_index.value(), new_index);
    }
  } else {
    p_index = input.jobs[route[rank - 1]].index();
    new_previous_eval = v.eval(p_index.value(), new_index);
  }

  if (rank == route.size() - 1) {
    if (v.has_end()) {
      n_index = v.end.value().index();
      new_next_eval = v.eval(new_index, n_index.value());
    }
  } else {
    n_index = input.jobs[route[rank + 1]].index();
    new_next_eval = v.eval(new_index, n_index.value());
  }

  Eval old_virtual_eval;
  if (p_index && n_index) {
    old_virtual_eval = v.eval(p_index.value(), n_index.value());
  }

  return new_previous_eval + new_next_eval - old_virtual_eval;
}

Priority priority_sum_for_route(const Input& input,
                                const std::vector<Index>& route);

Eval route_eval_for_vehicle(const Input& input,
                            Index vehicle_rank,
                            std::vector<Index>::const_iterator first_job,
                            std::vector<Index>::const_iterator last_job);

Eval route_eval_for_vehicle(const Input& input,
                            Index vehicle_rank,
                            const std::vector<Index>& route);

void check_tws(const std::vector<TimeWindow>& tws,
               Id id,
               const std::string& type);

void check_priority(Priority priority, Id id, const std::string& type);

void check_no_empty_keys(const TypeToDurationMap& type_to_duration,
                         const Id id,
                         const std::string& type,
                         const std::string& key_name);

using RawSolution = std::vector<RawRoute>;
using TWSolution = std::vector<TWRoute>;

Solution format_solution(const Input& input, const RawSolution& raw_routes);

Route format_route(const Input& input,
                   const TWRoute& tw_r,
                   std::unordered_set<Index>& unassigned_ranks);

Solution format_solution(const Input& input, const TWSolution& tw_routes);

} // namespace vroom::utils

#endif
