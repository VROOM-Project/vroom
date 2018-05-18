/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <iostream>

#include "relocate.h"

relocate::relocate(const input& input,
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

void relocate::compute_gain() {
  assert(source_vehicle != target_vehicle);
  assert(_sol[source_vehicle].size() >= 1);
  assert(source_rank < _sol[source_vehicle].size());
  assert(target_rank <= _sol[target_vehicle].size());

  auto& m = _input.get_matrix();
  const auto& v_target = _input._vehicles[target_vehicle];

  // For source vehicle, we consider the cost of removing job at rank
  // source_rank, already stored in
  // node_gains[source_vehicle][source_rank].

  // For target vehicle, we consider the cost of adding source job at
  // rank target_rank.
  index_t c_index = _input._jobs[_sol[source_vehicle][source_rank]].index();

  gain_t previous_cost = 0;
  gain_t next_cost = 0;
  gain_t old_edge_cost = 0;

  if (target_rank == _sol[target_vehicle].size()) {
    if (_sol[target_vehicle].size() == 0) {
      // Adding job to an empty route.
      if (v_target.has_start()) {
        previous_cost = m[v_target.start.get().index()][c_index];
      }
      if (v_target.has_end()) {
        next_cost = m[c_index][v_target.end.get().index()];
      }
    } else {
      // Adding job past the end after a real job.
      auto p_index =
        _input._jobs[_sol[target_vehicle][target_rank - 1]].index();
      previous_cost = m[p_index][c_index];
      if (v_target.has_end()) {
        auto n_index = v_target.end.get().index();
        old_edge_cost = m[p_index][n_index];
        next_cost = m[c_index][n_index];
      }
    }
  } else {
    // Adding before one of the jobs.
    auto n_index = _input._jobs[_sol[target_vehicle][target_rank]].index();
    next_cost = m[c_index][n_index];

    if (target_rank == 0) {
      if (v_target.has_start()) {
        auto p_index = v_target.start.get().index();
        previous_cost = m[p_index][c_index];
        old_edge_cost = m[p_index][n_index];
      }
    } else {
      auto p_index =
        _input._jobs[_sol[target_vehicle][target_rank - 1]].index();
      previous_cost = m[p_index][c_index];
      old_edge_cost = m[p_index][n_index];
    }
  }

  // Gain for target vehicle.
  gain_t target_gain = old_edge_cost - previous_cost - next_cost;

  stored_gain = node_gains[source_vehicle][source_rank] + target_gain;
  gain_computed = true;
}

bool relocate::is_valid() const {
  auto relocate_job_rank = _sol[source_vehicle][source_rank];

  bool valid = _input.vehicle_ok_with_job(target_vehicle, relocate_job_rank);

  valid &= (_amounts[target_vehicle] + _input._jobs[relocate_job_rank].amount <=
            _input._vehicles[target_vehicle].capacity);

  return valid;
}

void relocate::apply() const {
  auto relocate_job_rank = _sol[source_vehicle][source_rank];
  _sol[source_vehicle].erase(_sol[source_vehicle].begin() + source_rank);
  _sol[target_vehicle].insert(_sol[target_vehicle].begin() + target_rank,
                              relocate_job_rank);

  auto& relocate_amount = _input._jobs[relocate_job_rank].amount;
  _amounts[target_vehicle] += relocate_amount;
  _amounts[source_vehicle] -= relocate_amount;
}

void relocate::log() const {
  const auto& v_source = _input._vehicles[source_vehicle];
  const auto& v_target = _input._vehicles[target_vehicle];

  std::cout << "Relocate gain: " << stored_gain << " - vehicle " << v_source.id
            << ", step " << source_rank << " (job "
            << _input._jobs[_sol[source_vehicle][source_rank]].id
            << ") moved to rank " << target_rank << " in route for vehicle "
            << v_target.id << std::endl;
}
