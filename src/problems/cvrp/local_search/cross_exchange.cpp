/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/cvrp/local_search/cross_exchange.h"

cvrp_cross_exchange::cvrp_cross_exchange(const input& input,
                                         raw_solution& sol,
                                         const solution_state& sol_state,
                                         index_t source_vehicle,
                                         index_t source_rank,
                                         index_t target_vehicle,
                                         index_t target_rank)
  : cvrp_ls_operator(input,
                     sol,
                     sol_state,
                     source_vehicle,
                     source_rank,
                     target_vehicle,
                     target_rank),
    reverse_source_edge(false),
    reverse_target_edge(false) {
  assert(source_vehicle != target_vehicle);
  assert(_sol[source_vehicle].size() >= 2);
  assert(_sol[target_vehicle].size() >= 2);
  assert(source_rank < _sol[source_vehicle].size() - 1);
  assert(target_rank < _sol[target_vehicle].size() - 1);
}

void cvrp_cross_exchange::compute_gain() {
  const auto& m = _input.get_matrix();
  const auto& v_source = _input._vehicles[source_vehicle];
  const auto& v_target = _input._vehicles[target_vehicle];

  // For source vehicle, we consider the cost of replacing edge
  // starting at rank source_rank with target edge. Part of that cost
  // (for adjacent edges) is stored in
  // _sol_state.edge_costs_around_edge.  reverse_* checks whether we
  // should change the target edge order.
  index_t s_c_index = _input._jobs[_sol[source_vehicle][source_rank]].index();
  index_t s_after_c_index =
    _input._jobs[_sol[source_vehicle][source_rank + 1]].index();
  index_t t_c_index = _input._jobs[_sol[target_vehicle][target_rank]].index();
  index_t t_after_c_index =
    _input._jobs[_sol[target_vehicle][target_rank + 1]].index();

  // Determine costs added with target edge.
  gain_t previous_cost = 0;
  gain_t next_cost = 0;
  gain_t reverse_previous_cost = 0;
  gain_t reverse_next_cost = 0;

  if (source_rank == 0) {
    if (v_source.has_start()) {
      auto p_index = v_source.start.get().index();
      previous_cost = m[p_index][t_c_index];
      reverse_previous_cost = m[p_index][t_after_c_index];
    }
  } else {
    auto p_index = _input._jobs[_sol[source_vehicle][source_rank - 1]].index();
    previous_cost = m[p_index][t_c_index];
    reverse_previous_cost = m[p_index][t_after_c_index];
  }

  if (source_rank == _sol[source_vehicle].size() - 2) {
    if (v_source.has_end()) {
      auto n_index = v_source.end.get().index();
      next_cost = m[t_after_c_index][n_index];
      reverse_next_cost = m[t_c_index][n_index];
    }
  } else {
    auto n_index = _input._jobs[_sol[source_vehicle][source_rank + 2]].index();
    next_cost = m[t_after_c_index][n_index];
    reverse_next_cost = m[t_c_index][n_index];
  }

  gain_t source_gain =
    _sol_state.edge_costs_around_edge[source_vehicle][source_rank] -
    previous_cost - next_cost;

  gain_t reverse_edge_cost =
    static_cast<gain_t>(m[t_c_index][t_after_c_index]) -
    static_cast<gain_t>(m[t_after_c_index][t_c_index]);
  gain_t reverse_source_gain =
    _sol_state.edge_costs_around_edge[source_vehicle][source_rank] +
    reverse_edge_cost - reverse_previous_cost - reverse_next_cost;

  if (reverse_source_gain > source_gain) {
    reverse_target_edge = true;
    source_gain = reverse_source_gain;
  }

  // For target vehicle, we consider the cost of replacing edge
  // starting at rank target_rank with source edge. Part of that cost
  // (for adjacent edges) is stored in edge_costs_around_edge.
  // reverse_* checks whether we should change the source edge order.
  previous_cost = 0;
  next_cost = 0;
  reverse_previous_cost = 0;
  reverse_next_cost = 0;

  if (target_rank == 0) {
    if (v_target.has_start()) {
      auto p_index = v_target.start.get().index();
      previous_cost = m[p_index][s_c_index];
      reverse_previous_cost = m[p_index][s_after_c_index];
    }
  } else {
    auto p_index = _input._jobs[_sol[target_vehicle][target_rank - 1]].index();
    previous_cost = m[p_index][s_c_index];
    reverse_previous_cost = m[p_index][s_after_c_index];
  }

  if (target_rank == _sol[target_vehicle].size() - 2) {
    if (v_target.has_end()) {
      auto n_index = v_target.end.get().index();
      next_cost = m[s_after_c_index][n_index];
      reverse_next_cost = m[s_c_index][n_index];
    }
  } else {
    auto n_index = _input._jobs[_sol[target_vehicle][target_rank + 2]].index();
    next_cost = m[s_after_c_index][n_index];
    reverse_next_cost = m[s_c_index][n_index];
  }

  gain_t target_gain =
    _sol_state.edge_costs_around_edge[target_vehicle][target_rank] -
    previous_cost - next_cost;

  reverse_edge_cost = static_cast<gain_t>(m[s_c_index][s_after_c_index]) -
                      static_cast<gain_t>(m[s_after_c_index][s_c_index]);
  gain_t reverse_target_gain =
    _sol_state.edge_costs_around_edge[target_vehicle][target_rank] +
    reverse_edge_cost - reverse_previous_cost - reverse_next_cost;

  if (reverse_target_gain > target_gain) {
    reverse_source_edge = true;
    target_gain = reverse_target_gain;
  }

  stored_gain = source_gain + target_gain;

  gain_computed = true;
}

bool cvrp_cross_exchange::is_valid() const {
  auto s_current_job_rank = _sol[source_vehicle][source_rank];
  auto t_current_job_rank = _sol[target_vehicle][target_rank];
  // Already asserted in compute_gain.
  auto s_after_job_rank = _sol[source_vehicle][source_rank + 1];
  auto t_after_job_rank = _sol[target_vehicle][target_rank + 1];

  bool valid = _input.vehicle_ok_with_job(target_vehicle, s_current_job_rank);
  valid &= _input.vehicle_ok_with_job(target_vehicle, s_after_job_rank);
  valid &= _input.vehicle_ok_with_job(source_vehicle, t_current_job_rank);
  valid &= _input.vehicle_ok_with_job(source_vehicle, t_after_job_rank);

  valid &= (_sol_state.fwd_amounts[source_vehicle].back() -
              _input._jobs[s_current_job_rank].amount -
              _input._jobs[s_after_job_rank].amount +
              _input._jobs[t_current_job_rank].amount +
              _input._jobs[t_after_job_rank].amount <=
            _input._vehicles[source_vehicle].capacity);

  valid &= (_sol_state.fwd_amounts[target_vehicle].back() -
              _input._jobs[t_current_job_rank].amount -
              _input._jobs[t_after_job_rank].amount +
              _input._jobs[s_current_job_rank].amount +
              _input._jobs[s_after_job_rank].amount <=
            _input._vehicles[target_vehicle].capacity);

  return valid;
}

void cvrp_cross_exchange::apply() const {
  std::swap(_sol[source_vehicle][source_rank],
            _sol[target_vehicle][target_rank]);
  std::swap(_sol[source_vehicle][source_rank + 1],
            _sol[target_vehicle][target_rank + 1]);

  if (reverse_source_edge) {
    std::swap(_sol[target_vehicle][target_rank],
              _sol[target_vehicle][target_rank + 1]);
  }
  if (reverse_target_edge) {
    std::swap(_sol[source_vehicle][source_rank],
              _sol[source_vehicle][source_rank + 1]);
  }
}

std::vector<index_t> cvrp_cross_exchange::addition_candidates() const {
  return {source_vehicle, target_vehicle};
}
