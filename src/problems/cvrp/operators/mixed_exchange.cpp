/*

This file is part of VROOM.

Copyright (c) 2015-2019, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/cvrp/operators/mixed_exchange.h"

namespace vroom {
namespace cvrp {

MixedExchange::MixedExchange(const Input& input,
                             const utils::SolutionState& sol_state,
                             RawRoute& s_route,
                             Index s_vehicle,
                             Index s_rank,
                             RawRoute& t_route,
                             Index t_vehicle,
                             Index t_rank)
  : Operator(input,
             sol_state,
             s_route,
             s_vehicle,
             s_rank,
             t_route,
             t_vehicle,
             t_rank),
    reverse_t_edge(false) {
  assert(s_vehicle != t_vehicle);
  assert(s_route.size() >= 1);
  assert(t_route.size() >= 2);
  assert(s_rank < s_route.size());
  assert(t_rank < t_route.size() - 1);
}

void MixedExchange::compute_gain() {
  const auto& m = _input.get_matrix();
  const auto& v_source = _input.vehicles[s_vehicle];
  const auto& v_target = _input.vehicles[t_vehicle];

  // For source vehicle, we consider the cost of replacing job at rank
  // s_rank with target edge. Part of that cost (for adjacent edges)
  // is stored in _sol_state.edge_costs_around_node. reverse_t_edge
  // checks whether we should change the target edge order.
  Index s_index = _input.jobs[s_route[s_rank]].index();
  Index t_index = _input.jobs[t_route[t_rank]].index();
  Index t_after_index = _input.jobs[t_route[t_rank + 1]].index();

  // Determine costs added with target edge.
  Gain previous_cost = 0;
  Gain next_cost = 0;
  Gain reverse_previous_cost = 0;
  Gain reverse_next_cost = 0;

  if (s_rank == 0) {
    if (v_source.has_start()) {
      auto p_index = v_source.start.get().index();
      previous_cost = m[p_index][t_index];
      reverse_previous_cost = m[p_index][t_after_index];
    }
  } else {
    auto p_index = _input.jobs[s_route[s_rank - 1]].index();
    previous_cost = m[p_index][t_index];
    reverse_previous_cost = m[p_index][t_after_index];
  }

  if (s_rank == s_route.size() - 1) {
    if (v_source.has_end()) {
      auto n_index = v_source.end.get().index();
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

  // For target vehicle, we consider the cost of replacing edge at
  // rank t_rank with source job. Part of that cost (for adjacent
  // edges) is stored in _sol_state.edge_costs_around_edges.

  // Determine costs added with source job.
  previous_cost = 0;
  next_cost = 0;

  if (t_rank == 0) {
    if (v_target.has_start()) {
      auto p_index = v_target.start.get().index();
      previous_cost = m[p_index][s_index];
    }
  } else {
    auto p_index = _input.jobs[t_route[t_rank - 1]].index();
    previous_cost = m[p_index][s_index];
  }

  if (t_rank == t_route.size() - 2) {
    if (v_target.has_end()) {
      auto n_index = v_target.end.get().index();
      next_cost = m[s_index][n_index];
    }
  } else {
    auto n_index = _input.jobs[t_route[t_rank + 2]].index();
    next_cost = m[s_index][n_index];
  }

  t_gain = _sol_state.edge_costs_around_edge[t_vehicle][t_rank] -
           previous_cost - next_cost;

  stored_gain = std::max(normal_s_gain, reversed_s_gain) + t_gain;
  gain_computed = true;
}

bool MixedExchange::is_valid() {
  auto s_job_rank = s_route[s_rank];
  auto t_job_rank = t_route[t_rank];
  // Already asserted in compute_gain.
  auto t_after_job_rank = t_route[t_rank + 1];

  bool valid = _input.vehicle_ok_with_job(t_vehicle, s_job_rank);
  valid &= _input.vehicle_ok_with_job(s_vehicle, t_job_rank);
  valid &= _input.vehicle_ok_with_job(s_vehicle, t_after_job_rank);

  valid &=
    (_sol_state.fwd_amounts[s_vehicle].back() - _input.jobs[s_job_rank].amount +
       _input.jobs[t_job_rank].amount + _input.jobs[t_after_job_rank].amount <=
     _input.vehicles[s_vehicle].capacity);

  valid &=
    (_sol_state.fwd_amounts[t_vehicle].back() - _input.jobs[t_job_rank].amount -
       _input.jobs[t_after_job_rank].amount + _input.jobs[s_job_rank].amount <=
     _input.vehicles[t_vehicle].capacity);

  return valid;
}

void MixedExchange::apply() {
  std::swap(s_route[s_rank], t_route[t_rank]);
  s_route.insert(s_route.begin() + s_rank + 1,
                 t_route.begin() + t_rank + 1,
                 t_route.begin() + t_rank + 2);
  t_route.erase(t_route.begin() + t_rank + 1);

  if (reverse_t_edge) {
    std::swap(s_route[s_rank], s_route[s_rank + 1]);
  }
}

std::vector<Index> MixedExchange::addition_candidates() const {
  return {s_vehicle, t_vehicle};
}

std::vector<Index> MixedExchange::update_candidates() const {
  return {s_vehicle, t_vehicle};
}

} // namespace cvrp
} // namespace vroom
