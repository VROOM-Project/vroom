/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <iostream>

#include "cross_exchange.h"

cross_exchange::cross_exchange(const input& input,
                               raw_solution& sol,
                               std::vector<amount_t>& amounts,
                               index_t source_vehicle,
                               index_t source_rank,
                               index_t target_vehicle,
                               index_t target_rank)
  : ls_operator(input,
                sol,
                amounts,
                source_vehicle,
                source_rank,
                target_vehicle,
                target_rank) {
}

void cross_exchange::compute_gain() {
  assert(source_vehicle != target_vehicle);
  assert(_sol[source_vehicle].size() >= 2);
  assert(_sol[target_vehicle].size() >= 2);
  assert(source_rank < _sol[source_vehicle].size() - 1);
  assert(target_rank < _sol[target_vehicle].size() - 1);

  auto m = _input.get_matrix();
  const auto& v_source = _input._vehicles[source_vehicle];
  const auto& v_target = _input._vehicles[target_vehicle];

  // For source vehicle, we consider the cost of replacing edge
  // starting at rank source_rank with target edge. Part of that cost
  // (for adjacent edges) is stored in edge_costs_around_edge.
  index_t s_c_index = _input._jobs[_sol[source_vehicle][source_rank]].index();
  index_t s_after_c_index =
    _input._jobs[_sol[source_vehicle][source_rank + 1]].index();
  index_t t_c_index = _input._jobs[_sol[target_vehicle][target_rank]].index();
  index_t t_after_c_index =
    _input._jobs[_sol[target_vehicle][target_rank + 1]].index();

  // Determine costs added with target edge.
  gain_t new_previous_cost = 0;
  gain_t new_next_cost = 0;

  if (source_rank == 0) {
    if (v_source.has_start()) {
      auto p_index = v_source.start.get().index();
      new_previous_cost = m[p_index][t_c_index];
    }
  } else {
    auto p_index = _input._jobs[_sol[source_vehicle][source_rank - 1]].index();
    new_previous_cost = m[p_index][t_c_index];
  }

  if (source_rank == _sol[source_vehicle].size() - 2) {
    if (v_source.has_end()) {
      auto n_index = v_source.end.get().index();
      new_next_cost = m[t_after_c_index][n_index];
    }
  } else {
    auto n_index = _input._jobs[_sol[source_vehicle][source_rank + 2]].index();
    new_next_cost = m[t_after_c_index][n_index];
  }

  gain_t source_gain = edge_costs_around_edge[source_vehicle][source_rank] -
                       new_previous_cost - new_next_cost;

  // For target vehicle, we consider the cost of replacing edge
  // starting at rank target_rank with source edge. Part of that cost
  // (for adjacent edges) is stored in edge_costs_around_edge.

  // Determine costs added with source edge.
  new_previous_cost = 0;
  new_next_cost = 0;

  if (target_rank == 0) {
    if (v_target.has_start()) {
      auto p_index = v_target.start.get().index();
      new_previous_cost = m[p_index][s_c_index];
    }
  } else {
    auto p_index = _input._jobs[_sol[target_vehicle][target_rank - 1]].index();
    new_previous_cost = m[p_index][s_c_index];
  }

  if (target_rank == _sol[target_vehicle].size() - 2) {
    if (v_target.has_end()) {
      auto n_index = v_target.end.get().index();
      new_next_cost = m[s_after_c_index][n_index];
    }
  } else {
    auto n_index = _input._jobs[_sol[target_vehicle][target_rank + 2]].index();
    new_next_cost = m[s_after_c_index][n_index];
  }

  gain_t target_gain = edge_costs_around_edge[target_vehicle][target_rank] -
                       new_previous_cost - new_next_cost;

  stored_gain = source_gain + target_gain;

  gain_computed = true;
}

bool cross_exchange::is_valid() const {
  auto s_current_job_rank = _sol[source_vehicle][source_rank];
  auto t_current_job_rank = _sol[target_vehicle][target_rank];
  // Already asserted in compute_gain.
  auto s_after_job_rank = _sol[source_vehicle][source_rank + 1];
  auto t_after_job_rank = _sol[target_vehicle][target_rank + 1];

  bool valid = _input.vehicle_ok_with_job(target_vehicle, s_current_job_rank);
  valid &= _input.vehicle_ok_with_job(target_vehicle, s_after_job_rank);
  valid &= _input.vehicle_ok_with_job(source_vehicle, t_current_job_rank);
  valid &= _input.vehicle_ok_with_job(source_vehicle, t_after_job_rank);

  valid &= (_amounts[source_vehicle] - _input._jobs[s_current_job_rank].amount -
              _input._jobs[s_after_job_rank].amount +
              _input._jobs[t_current_job_rank].amount +
              _input._jobs[t_after_job_rank].amount <=
            _input._vehicles[source_vehicle].capacity);

  valid &= (_amounts[target_vehicle] - _input._jobs[t_current_job_rank].amount -
              _input._jobs[t_after_job_rank].amount +
              _input._jobs[s_current_job_rank].amount +
              _input._jobs[s_after_job_rank].amount <=
            _input._vehicles[target_vehicle].capacity);

  return valid;
}

void cross_exchange::apply() const {
  auto s_amount = _input._jobs[_sol[source_vehicle][source_rank]].amount +
                  _input._jobs[_sol[source_vehicle][source_rank + 1]].amount;
  auto t_amount = _input._jobs[_sol[target_vehicle][target_rank]].amount +
                  _input._jobs[_sol[target_vehicle][target_rank + 1]].amount;

  auto s_t_amount = t_amount - s_amount;

  _amounts[source_vehicle] += s_t_amount;
  _amounts[target_vehicle] -= s_t_amount;

  std::swap(_sol[source_vehicle][source_rank],
            _sol[target_vehicle][target_rank]);
  std::swap(_sol[source_vehicle][source_rank + 1],
            _sol[target_vehicle][target_rank + 1]);
}

void cross_exchange::log() const {
  const auto& v_source = _input._vehicles[source_vehicle];
  const auto& v_target = _input._vehicles[target_vehicle];

  std::cout << "Cross_Exchange gain: " << stored_gain << " - vehicle "
            << v_source.id << ", edge " << source_rank << " -> "
            << source_rank + 1 << " (job "
            << _input._jobs[_sol[source_vehicle][source_rank]].id << " -> "
            << _input._jobs[_sol[source_vehicle][source_rank + 1]].id
            << ") exchanged with vehicle " << v_target.id << ", edge "
            << target_rank << " -> " << target_rank + 1 << " (job "
            << _input._jobs[_sol[target_vehicle][target_rank]].id << " -> "
            << _input._jobs[_sol[target_vehicle][target_rank + 1]].id << ")"
            << std::endl;
}
