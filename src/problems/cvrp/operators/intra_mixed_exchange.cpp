/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/cvrp/operators/intra_mixed_exchange.h"
#include "utils/helpers.h"

namespace vroom {
namespace cvrp {

IntraMixedExchange::IntraMixedExchange(const Input& input,
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
    reverse_t_edge(false) {
  // If node at s_rank is right before/after edge at t_rank, then the
  // move is a relocate.
  assert(s_rank + 1 < t_rank or t_rank + 2 < s_rank);
  assert(s_route.size() >= 4);
  assert(s_rank < s_route.size());
  assert(t_rank < s_route.size() - 1);
}

void IntraMixedExchange::compute_gain() {
  const auto& m = _input.get_matrix();
  const auto& v = _input.vehicles[s_vehicle];

  // Consider the cost of replacing node at rank s_rank with target
  // edge. Part of that cost (for adjacent edges) is stored in
  // _sol_state.edge_costs_around_node.  reverse_* checks whether we
  // should change the target edge order.
  Index s_index = _input.jobs[s_route[s_rank]].index();
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

  if (s_rank == s_route.size() - 1) {
    if (v.has_end()) {
      auto n_index = v.end.get().index();
      next_cost = m[t_after_index][n_index];
      reverse_next_cost = m[t_index][n_index];
    }
  } else {
    auto n_index = _input.jobs[s_route[s_rank + 1]].index();
    next_cost = m[t_after_index][n_index];
    reverse_next_cost = m[t_index][n_index];
  }

  normal_s_gain = _sol_state.edge_costs_around_node[s_vehicle][s_rank] -
                  previous_cost - next_cost;

  Gain reverse_edge_cost = static_cast<Gain>(m[t_index][t_after_index]) -
                           static_cast<Gain>(m[t_after_index][t_index]);
  reversed_s_gain = _sol_state.edge_costs_around_node[s_vehicle][s_rank] +
                    reverse_edge_cost - reverse_previous_cost -
                    reverse_next_cost;

  if (reversed_s_gain > normal_s_gain) {
    reverse_t_edge = true;
  }

  // Consider the cost of replacing edge starting at rank t_rank with
  // source node. Part of that cost (for adjacent edges) is stored in
  // _sol_state.edge_costs_around_edge.  reverse_* checks whether we
  // should change the source edge order.
  previous_cost = 0;
  next_cost = 0;

  if (t_rank == 0) {
    if (v.has_start()) {
      auto p_index = v.start.get().index();
      previous_cost = m[p_index][s_index];
    }
  } else {
    auto p_index = _input.jobs[s_route[t_rank - 1]].index();
    previous_cost = m[p_index][s_index];
  }

  if (t_rank == s_route.size() - 2) {
    if (v.has_end()) {
      auto n_index = v.end.get().index();
      next_cost = m[s_index][n_index];
      reverse_next_cost = m[s_index][n_index];
    }
  } else {
    auto n_index = _input.jobs[s_route[t_rank + 2]].index();
    next_cost = m[s_index][n_index];
  }

  t_gain = _sol_state.edge_costs_around_edge[t_vehicle][t_rank] -
           previous_cost - next_cost;

  stored_gain = std::max(normal_s_gain, reversed_s_gain) + t_gain;

  gain_computed = true;
}

bool IntraMixedExchange::is_valid() {
  return true;
}

void IntraMixedExchange::apply() {
  if (reverse_t_edge) {
    std::swap(s_route[t_rank], s_route[t_rank + 1]);
  }

  std::swap(s_route[s_rank], s_route[t_rank]);

  auto t_after = s_route[t_rank + 1];
  s_route.erase(s_route.begin() + t_rank + 1);

  auto end_t_rank = s_rank + 1;
  if (t_rank < s_rank) {
    --end_t_rank;
  }

  s_route.insert(s_route.begin() + end_t_rank, t_after);
}

std::vector<Index> IntraMixedExchange::addition_candidates() const {
  return {};
}

std::vector<Index> IntraMixedExchange::update_candidates() const {
  return {s_vehicle};
}

} // namespace cvrp
} // namespace vroom
