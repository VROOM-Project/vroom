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
  this->compute_gain();
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

  // For source vehicle, we consider replacing "s_previous -->
  // s_current --> s_after --> s_next" with "s_previous --> t_current
  // --> t_after --> s_next".
  index_t s_current = _input._jobs[_sol[source_vehicle][source_rank]].index();
  index_t s_after = _input._jobs[_sol[source_vehicle][source_rank + 1]].index();
  index_t t_current = _input._jobs[_sol[target_vehicle][target_rank]].index();
  index_t t_after = _input._jobs[_sol[target_vehicle][target_rank + 1]].index();
  index_t s_previous;
  if (source_rank == 0) {
    assert(v_source.has_start());
    s_previous = v_source.start.get().index();
  } else {
    s_previous = _input._jobs[_sol[source_vehicle][source_rank - 1]].index();
  }
  index_t s_next;
  if (source_rank == _sol[source_vehicle].size() - 2) {
    assert(v_source.has_end());
    s_next = v_source.end.get().index();
  } else {
    s_next = _input._jobs[_sol[source_vehicle][source_rank + 2]].index();
  }

  // Gain for source vehicle.
  gain_t g1 = static_cast<gain_t>(m[s_previous][s_current]) +
              m[s_after][s_next] - m[s_previous][t_current] -
              m[t_after][s_next];

  // For target vehicle, we consider replacing "t_previous -->
  // t_current --> t_after --> t_next" with "t_previous --> s_current
  // --> s_after --> t_next".
  index_t t_previous;
  if (target_rank == 0) {
    assert(v_target.has_start());
    t_previous = v_target.start.get().index();
  } else {
    t_previous = _input._jobs[_sol[target_vehicle][target_rank - 1]].index();
  }
  index_t t_next;
  if (target_rank == _sol[target_vehicle].size() - 2) {
    assert(v_target.has_end());
    t_next = v_target.end.get().index();
  } else {
    t_next = _input._jobs[_sol[target_vehicle][target_rank + 2]].index();
  }

  // Gain for target vehicle.
  gain_t g2 = static_cast<gain_t>(m[t_previous][t_current]) +
              m[t_after][t_next] - m[t_previous][s_current] -
              m[s_after][t_next];

  stored_gain = g1 + g2;
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
