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

  auto m = _input.get_matrix();
  const auto& v_target = _input._vehicles[target_vehicle];

  // For source vehicle, we consider replacing "previous --> current
  // --> next" with "previous --> next". This is already stored at
  // node_gains[source_vehicle][source_rank].

  // For target vehicle, we consider replacing "previous --> next"
  // with "previous --> current --> next".
  index_t current = _input._jobs[_sol[source_vehicle][source_rank]].index();
  index_t previous;
  index_t next;

  if (target_rank == 0) {
    assert(v_target.has_start());
    previous = v_target.start.get().index();
  } else {
    previous = _input._jobs[_sol[target_vehicle][target_rank - 1]].index();
  }
  if (target_rank == _sol[target_vehicle].size()) {
    assert(v_target.has_end());
    next = v_target.end.get().index();
  } else {
    next = _input._jobs[_sol[target_vehicle][target_rank]].index();
  }

  // Gain for target vehicle.
  gain_t old_edge_cost = m[previous][next];
  if (_sol[target_vehicle].size() == 0) {
    // Adding to empty target route, so cost of start --> end without
    // job is not taken into account.
    old_edge_cost = 0;
  }
  // Implicit cast to gain_t thanks to old_edge_cost.
  gain_t g2 = old_edge_cost - m[previous][current] - m[current][next];

  stored_gain = node_gains[source_vehicle][source_rank] + g2;
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
