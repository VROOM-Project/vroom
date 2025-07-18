#ifndef HELPERS_H
#define HELPERS_H

/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <optional>
#include <string>
#include <tuple>
#include <vector>

#include "structures/typedefs.h"
#include "structures/vroom/raw_route.h"
#include "structures/vroom/solution_state.h"
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

inline auto get_indices(const Input& input,
                        const RawRoute& route,
                        Index first_rank,
                        Index last_rank) {
  const auto& r = route.route;
  const auto& v = input.vehicles[route.v_rank];

  std::array<std::optional<Index>, 3> indices;

  auto& before_first = indices[0];
  if (first_rank > 0) {
    before_first = input.jobs[r[first_rank - 1]].index();
  } else {
    if (v.has_start()) {
      before_first = v.start.value().index();
    }
  }

  auto& first_index = indices[1];
  if (first_rank < r.size()) {
    first_index = input.jobs[r[first_rank]].index();
  } else {
    if (v.has_end()) {
      first_index = v.end.value().index();
    }
  }

  auto& last_index = indices[2];
  if (last_rank < r.size()) {
    last_index = input.jobs[r[last_rank]].index();
  } else {
    if (v.has_end()) {
      last_index = v.end.value().index();
    }
  }

  return indices;
}

inline Eval get_range_removal_gain(const SolutionState& sol_state,
                                   Index v,
                                   Index first_rank,
                                   Index last_rank) {
  assert(first_rank <= last_rank);

  Eval removal_gain;

  if (last_rank > first_rank) {
    // Gain related to removed portion.
    removal_gain += sol_state.fwd_costs[v][v][last_rank - 1];
    removal_gain -= sol_state.fwd_costs[v][v][first_rank];
  }

  return removal_gain;
}

// Compute cost variation when replacing the [first_rank, last_rank)
// portion for route1 with the range [insertion_start; insertion_end)
// from route_2. Returns a tuple to evaluate at once both options
// where new range is inserted as is, or reversed.
inline std::tuple<Eval, Eval>
addition_cost_delta(const Input& input,
                    const SolutionState& sol_state,
                    const RawRoute& route_1,
                    const Index first_rank,
                    const Index last_rank,
                    const RawRoute& route_2,
                    const Index insertion_start,
                    const Index insertion_end) {
  assert(first_rank <= last_rank);
  assert(last_rank <= route_1.route.size());
  assert(insertion_start <= insertion_end);

  const bool empty_insertion = (insertion_start == insertion_end);

  const auto& r1 = route_1.route;
  const auto v1_rank = route_1.v_rank;
  const auto& r2 = route_2.route;
  const auto v2_rank = route_2.v_rank;
  const auto& v1 = input.vehicles[v1_rank];

  // Common part of the cost.
  Eval cost_delta =
    get_range_removal_gain(sol_state, v1_rank, first_rank, last_rank);

  // Part of the cost that depends on insertion orientation.
  Eval straight_delta;
  Eval reversed_delta;
  if (insertion_start != insertion_end) {
    straight_delta += sol_state.fwd_costs[v2_rank][v1_rank][insertion_start];
    straight_delta -= sol_state.fwd_costs[v2_rank][v1_rank][insertion_end - 1];

    reversed_delta += sol_state.bwd_costs[v2_rank][v1_rank][insertion_start];
    reversed_delta -= sol_state.bwd_costs[v2_rank][v1_rank][insertion_end - 1];
  }

  // Determine useful values if present.
  const auto [before_first, first_index, last_index] =
    get_indices(input, route_1, first_rank, last_rank);

  // Gain of removed edge before replaced range. If route is empty,
  // before_first and first_index are respectively the start and end
  // of vehicle if defined.
  if (before_first && first_index && !r1.empty()) {
    cost_delta += v1.eval(before_first.value(), first_index.value());
  }

  if (empty_insertion) {
    if (before_first && last_index &&
        !(first_rank == 0 && last_rank == r1.size())) {
      // Add cost of new edge replacing removed range, except if
      // resulting route is empty.
      cost_delta -= v1.eval(before_first.value(), last_index.value());
    }
  } else {
    if (before_first) {
      // Cost of new edge to inserted range.
      straight_delta -=
        v1.eval(before_first.value(), input.jobs[r2[insertion_start]].index());
      reversed_delta -= v1.eval(before_first.value(),
                                input.jobs[r2[insertion_end - 1]].index());
    }

    if (last_index) {
      // Cost of new edge after inserted range.
      straight_delta -=
        v1.eval(input.jobs[r2[insertion_end - 1]].index(), last_index.value());
      reversed_delta -=
        v1.eval(input.jobs[r2[insertion_start]].index(), last_index.value());
    }
  }

  // Gain of removed edge after replaced range, if any.
  if (last_index && last_rank > first_rank) {
    const Index before_last = input.jobs[r1[last_rank - 1]].index();
    cost_delta += v1.eval(before_last, last_index.value());
  }

  // Handle fixed cost addition.
  if (r1.empty() && !empty_insertion) {
    cost_delta.cost -= v1.fixed_cost();
  }

  if (empty_insertion && first_rank == 0 && last_rank == r1.size()) {
    cost_delta.cost += v1.fixed_cost();
  }

  return std::make_tuple(cost_delta + straight_delta,
                         cost_delta + reversed_delta);
}

// Compute cost variation when replacing the *non-empty* [first_rank,
// last_rank) portion for route raw_route with the job at
// job_rank. The case where the replaced range is empty is already
// covered by addition_cost.
inline Eval addition_cost_delta(const Input& input,
                                const SolutionState& sol_state,
                                const RawRoute& raw_route,
                                Index first_rank,
                                Index last_rank,
                                Index job_rank) {
  assert(first_rank < last_rank && !raw_route.empty());
  assert(last_rank <= raw_route.route.size());

  const auto& r = raw_route.route;
  const auto v_rank = raw_route.v_rank;
  const auto& v = input.vehicles[v_rank];
  const auto job_index = input.jobs[job_rank].index();

  Eval cost_delta =
    get_range_removal_gain(sol_state, v_rank, first_rank, last_rank);

  // Determine useful values if present.
  const auto [before_first, first_index, last_index] =
    get_indices(input, raw_route, first_rank, last_rank);

  // Gain of removed edge before replaced range.
  if (before_first && first_index) {
    cost_delta += v.eval(before_first.value(), first_index.value());
  }

  if (before_first) {
    // Cost of new edge to inserted job.
    cost_delta -= v.eval(before_first.value(), job_index);
  }

  if (last_index) {
    // Cost of new edge after inserted job.
    cost_delta -= v.eval(job_index, last_index.value());
  }

  // Gain of removed edge after replaced range, if any.
  if (last_index) {
    const Index before_last = input.jobs[r[last_rank - 1]].index();
    cost_delta += v.eval(before_last, last_index.value());
  }

  return cost_delta;
}

// Compute cost variation when removing the "count" elements starting
// from rank in route.
inline Eval removal_cost_delta(const Input& input,
                               const SolutionState& sol_state,
                               const RawRoute& route,
                               Index rank,
                               unsigned count) {
  assert(!route.empty());
  assert(rank + count <= route.size());

  return std::get<0>(addition_cost_delta(input,
                                         sol_state,
                                         route,
                                         rank,
                                         rank + count,
                                         // dummy values for empty insertion
                                         route,
                                         0,
                                         0));
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
