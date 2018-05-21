/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <iostream>

#include "exchange.h"

exchange::exchange(const input& input,
                   raw_solution& sol,
                   index_t source_vehicle,
                   index_t source_rank,
                   index_t target_vehicle,
                   index_t target_rank)
  : ls_operator(input,
                sol,
                source_vehicle,
                source_rank,
                target_vehicle,
                target_rank) {
  assert(source_vehicle != target_vehicle);
  assert(_sol[source_vehicle].size() >= 1);
  assert(_sol[target_vehicle].size() >= 1);
  assert(source_rank < _sol[source_vehicle].size());
  assert(target_rank < _sol[target_vehicle].size());
}

void exchange::compute_gain() {
  auto& m = _input.get_matrix();
  const auto& v_source = _input._vehicles[source_vehicle];
  const auto& v_target = _input._vehicles[target_vehicle];

  // For source vehicle, we consider the cost of replacing job at rank
  // source_rank with target job. Part of that cost (for adjacent
  // edges) is stored in edge_costs_around_node.
  index_t s_c_index = _input._jobs[_sol[source_vehicle][source_rank]].index();
  index_t t_c_index = _input._jobs[_sol[target_vehicle][target_rank]].index();

  // Determine costs added with target job.
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

  if (source_rank == _sol[source_vehicle].size() - 1) {
    if (v_source.has_end()) {
      auto n_index = v_source.end.get().index();
      new_next_cost = m[t_c_index][n_index];
    }
  } else {
    auto n_index = _input._jobs[_sol[source_vehicle][source_rank + 1]].index();
    new_next_cost = m[t_c_index][n_index];
  }

  gain_t source_gain = edge_costs_around_node[source_vehicle][source_rank] -
                       new_previous_cost - new_next_cost;

  // For target vehicle, we consider the cost of replacing job at rank
  // target_rank with source job. Part of that cost (for adjacent
  // edges) is stored in edge_costs_around_node.

  // Determine costs added with source job.
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

  if (target_rank == _sol[target_vehicle].size() - 1) {
    if (v_target.has_end()) {
      auto n_index = v_target.end.get().index();
      new_next_cost = m[s_c_index][n_index];
    }
  } else {
    auto n_index = _input._jobs[_sol[target_vehicle][target_rank + 1]].index();
    new_next_cost = m[s_c_index][n_index];
  }

  gain_t target_gain = edge_costs_around_node[target_vehicle][target_rank] -
                       new_previous_cost - new_next_cost;

  stored_gain = source_gain + target_gain;
  gain_computed = true;
}

bool exchange::is_valid() const {
  auto source_job_rank = _sol[source_vehicle][source_rank];
  auto target_job_rank = _sol[target_vehicle][target_rank];

  bool valid = _input.vehicle_ok_with_job(target_vehicle, source_job_rank);
  valid &= _input.vehicle_ok_with_job(source_vehicle, target_job_rank);

  valid &=
    (amounts[target_vehicle].back() - _input._jobs[target_job_rank].amount +
       _input._jobs[source_job_rank].amount <=
     _input._vehicles[target_vehicle].capacity);

  valid &=
    (amounts[source_vehicle].back() - _input._jobs[source_job_rank].amount +
       _input._jobs[target_job_rank].amount <=
     _input._vehicles[source_vehicle].capacity);

  return valid;
}

void exchange::apply() const {
  std::swap(_sol[source_vehicle][source_rank],
            _sol[target_vehicle][target_rank]);
}

void exchange::log() const {
  const auto& v_source = _input._vehicles[source_vehicle];
  const auto& v_target = _input._vehicles[target_vehicle];

  std::cout << "Exchange gain: " << stored_gain << " - vehicle " << v_source.id
            << ", step " << source_rank << " (job "
            << _input._jobs[_sol[source_vehicle][source_rank]].id
            << ") exchanged with vehicle " << v_target.id << ", step "
            << target_rank << " (job "
            << _input._jobs[_sol[target_vehicle][target_rank]].id << ")"
            << std::endl;
}

std::vector<index_t> exchange::addition_candidates() const {
  return {source_vehicle, target_vehicle};
}
