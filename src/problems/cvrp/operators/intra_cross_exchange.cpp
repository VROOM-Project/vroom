/*

This file is part of VROOM.

Copyright (c) 2015-2019, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/cvrp/operators/intra_cross_exchange.h"
#include "utils/helpers.h"

namespace vroom {
namespace cvrp {

IntraCrossExchange::IntraCrossExchange(const Input& input,
                                       const utils::SolutionState& sol_state,
                                       RawRoute& s_route,
                                       Index s_vehicle,
                                       Index s_rank,
                                       Index t_rank)
  : Operator(input,
             sol_state,
             s_route,
             s_vehicle,
             s_rank,
             s_route,
             s_vehicle,
             t_rank),
    reverse_s_edge(false),
    reverse_t_edge(false) {
  // Use s_rank as smallest rank for symmetry reasons.
  assert(s_rank + 2 < t_rank); // Avoid common edge.
  assert(s_route.size() >= 5);
  assert(t_rank < s_route.size() - 1);
}

void IntraCrossExchange::compute_gain() {
  const auto& m = _input.get_matrix();
  const auto& v = _input.vehicles[s_vehicle];

  // Consider the cost of replacing edge starting at rank s_rank with
  // target edge. Part of that cost (for adjacent edges) is stored in
  // _sol_state.edge_costs_around_edge.  reverse_* checks whether we
  // should change the target edge order.
  Index s_index = _input.jobs[s_route[s_rank]].index();
  Index s_after_index = _input.jobs[s_route[s_rank + 1]].index();
  Index t_index = _input.jobs[s_route[t_rank]].index();
  Index t_after_index = _input.jobs[s_route[t_rank + 1]].index();

  // Determine costs added with target edge.
  Gain previous_cost = 0;
  Gain next_cost = 0;
  Gain reverse_previous_cost = 0;
  Gain reverse_next_cost = 0;

  if (s_rank == 0) {
    if (v.has_start()) {
      auto p_index = v.start.get().index();
      previous_cost = m[p_index][t_index];
      reverse_previous_cost = m[p_index][t_after_index];
    }
  } else {
    auto p_index = _input.jobs[s_route[s_rank - 1]].index();
    previous_cost = m[p_index][t_index];
    reverse_previous_cost = m[p_index][t_after_index];
  }

  auto n_index = _input.jobs[s_route[s_rank + 2]].index();
  next_cost = m[t_after_index][n_index];
  reverse_next_cost = m[t_index][n_index];

  normal_s_gain = _sol_state.edge_costs_around_edge[s_vehicle][s_rank] -
                  previous_cost - next_cost;

  Gain reverse_edge_cost = static_cast<Gain>(m[t_index][t_after_index]) -
                           static_cast<Gain>(m[t_after_index][t_index]);
  reversed_s_gain = _sol_state.edge_costs_around_edge[s_vehicle][s_rank] +
                    reverse_edge_cost - reverse_previous_cost -
                    reverse_next_cost;

  if (reversed_s_gain > normal_s_gain) {
    reverse_t_edge = true;
  }

  // Consider the cost of replacing edge starting at rank t_rank with
  // source edge. Part of that cost (for adjacent edges) is stored in
  // _sol_state.edge_costs_around_edge.  reverse_* checks whether we
  // should change the source edge order.
  next_cost = 0;
  reverse_previous_cost = 0;
  reverse_next_cost = 0;

  auto p_index = _input.jobs[s_route[t_rank - 1]].index();
  previous_cost = m[p_index][s_index];
  reverse_previous_cost = m[p_index][s_after_index];

  if (t_rank == s_route.size() - 2) {
    if (v.has_end()) {
      auto n_index = v.end.get().index();
      next_cost = m[s_after_index][n_index];
      reverse_next_cost = m[s_index][n_index];
    }
  } else {
    auto n_index = _input.jobs[s_route[t_rank + 2]].index();
    next_cost = m[s_after_index][n_index];
    reverse_next_cost = m[s_index][n_index];
  }

  normal_t_gain = _sol_state.edge_costs_around_edge[t_vehicle][t_rank] -
                  previous_cost - next_cost;

  reverse_edge_cost = static_cast<Gain>(m[s_index][s_after_index]) -
                      static_cast<Gain>(m[s_after_index][s_index]);
  reversed_t_gain = _sol_state.edge_costs_around_edge[t_vehicle][t_rank] +
                    reverse_edge_cost - reverse_previous_cost -
                    reverse_next_cost;

  if (reversed_t_gain > normal_t_gain) {
    reverse_s_edge = true;
  }

  stored_gain = std::max(normal_s_gain, reversed_s_gain) +
                std::max(normal_t_gain, reversed_t_gain);

  gain_computed = true;
}

bool IntraCrossExchange::is_valid() {
  return true;
}

void IntraCrossExchange::apply() {
  std::swap(s_route[s_rank], s_route[t_rank]);
  std::swap(s_route[s_rank + 1], s_route[t_rank + 1]);

  if (reverse_s_edge) {
    std::swap(s_route[t_rank], s_route[t_rank + 1]);
  }
  if (reverse_t_edge) {
    std::swap(s_route[s_rank], s_route[s_rank + 1]);
  }
}

std::vector<Index> IntraCrossExchange::addition_candidates() const {
  return {};
}

std::vector<Index> IntraCrossExchange::update_candidates() const {
  return {s_vehicle};
}

} // namespace cvrp
} // namespace vroom
