/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <iostream>

#include "exchange.h"

exchange::exchange(const input& input,
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

void exchange::compute_gain() {
  assert(source_vehicle != target_vehicle);
  assert(_sol[source_vehicle].size() >= 1);
  assert(_sol[target_vehicle].size() >= 1);
  assert(source_rank < _sol[source_vehicle].size());
  assert(target_rank < _sol[target_vehicle].size());

  auto m = _input.get_matrix();
  const auto& v_source = _input._vehicles[source_vehicle];
  const auto& v_target = _input._vehicles[target_vehicle];

  // For source vehicle, we consider replacing "s_previous -->
  // s_current --> s_next" with "s_previous --> t_current --> s_next".
  index_t s_current = _input._jobs[_sol[source_vehicle][source_rank]].index();
  index_t t_current = _input._jobs[_sol[target_vehicle][target_rank]].index();
  index_t s_previous;
  if (source_rank == 0) {
    assert(v_source.has_start());
    s_previous = v_source.start.get().index();
  } else {
    s_previous = _input._jobs[_sol[source_vehicle][source_rank - 1]].index();
  }
  index_t s_next;
  if (source_rank == _sol[source_vehicle].size() - 1) {
    assert(v_source.has_end());
    s_next = v_source.end.get().index();
  } else {
    s_next = _input._jobs[_sol[source_vehicle][source_rank + 1]].index();
  }

  // Gain for source vehicle.
  gain_t g1 = static_cast<gain_t>(m[s_previous][s_current]) +
              m[s_current][s_next] - m[s_previous][t_current] -
              m[t_current][s_next];
  BOOST_LOG_TRIVIAL(info) << m[s_previous][s_current] << " + "
                          << m[s_current][s_next] << " - "
                          << m[s_previous][t_current] << " - "
                          << m[t_current][s_next] << " = " << g1;

  // For target vehicle, we consider replacing "t_previous -->
  // t_current --> t_next" with "t_previous --> s_current --> t_next".
  index_t t_previous;
  if (target_rank == 0) {
    assert(v_target.has_start());
    t_previous = v_target.start.get().index();
  } else {
    t_previous = _input._jobs[_sol[target_vehicle][target_rank - 1]].index();
  }
  index_t t_next;
  if (target_rank == _sol[target_vehicle].size() - 1) {
    assert(v_target.has_end());
    t_next = v_target.end.get().index();
  } else {
    t_next = _input._jobs[_sol[target_vehicle][target_rank + 1]].index();
  }

  // Gain for target vehicle.
  gain_t g2 = static_cast<gain_t>(m[t_previous][t_current]) +
              m[t_current][t_next] - m[t_previous][s_current] -
              m[s_current][t_next];
  BOOST_LOG_TRIVIAL(info) << m[t_previous][t_current] << " + "
                          << m[t_current][t_next] << " - "
                          << m[t_previous][s_current] << " - "
                          << m[s_current][t_next] << " = " << g2;

  stored_gain = g1 + g2;
  gain_computed = true;
}

bool exchange::is_valid() const {
  auto source_job_rank = _sol[source_vehicle][source_rank];
  auto target_job_rank = _sol[target_vehicle][target_rank];

  bool valid = _input.vehicle_ok_with_job(target_vehicle, source_job_rank);
  valid &= _input.vehicle_ok_with_job(source_vehicle, target_job_rank);

  valid &= (_amounts[target_vehicle] - _input._jobs[target_job_rank].amount +
              _input._jobs[source_job_rank].amount <=
            _input._vehicles[target_vehicle].capacity);

  valid &= (_amounts[source_vehicle] - _input._jobs[source_job_rank].amount +
              _input._jobs[target_job_rank].amount <=
            _input._vehicles[source_vehicle].capacity);

  return valid;
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
