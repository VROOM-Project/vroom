/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <iostream>

#include "or_opt.h"

gain_t or_opt::compute_gain(const input& input,
                            const raw_solution& sol,
                            index_t source_vehicle,
                            index_t source_rank,
                            index_t target_vehicle,
                            index_t target_rank) {
  assert(source_vehicle != target_vehicle);
  assert(sol[source_vehicle].size() >= 2);
  assert(source_rank < sol[source_vehicle].size() - 1);
  assert(target_rank <= sol[target_vehicle].size());

  auto m = input.get_matrix();
  const auto& v_source = input._vehicles[source_vehicle];
  const auto& v_target = input._vehicles[target_vehicle];

  // For source vehicle, we consider replacing "previous --> current
  // --> after_current --> next" with "previous --> next".
  index_t current = input._jobs[sol[source_vehicle][source_rank]].index();
  index_t after_current =
    input._jobs[sol[source_vehicle][source_rank + 1]].index();
  index_t previous;
  if (source_rank == 0) {
    assert(v_source.has_start());
    previous = v_source.start.get().index();
  } else {
    previous = input._jobs[sol[source_vehicle][source_rank - 1]].index();
  }
  index_t next;
  if (source_rank == sol[source_vehicle].size() - 2) {
    assert(v_source.has_end());
    next = v_source.end.get().index();
  } else {
    next = input._jobs[sol[source_vehicle][source_rank + 2]].index();
  }

  // Gain for source vehicle.
  gain_t new_edge_cost = m[previous][next];
  if (sol[source_vehicle].size() == 2) {
    // Trying to empty a route, so cost of start --> end without job
    // is not taken into account.
    new_edge_cost = 0;
  }
  // Implicit cast to gain_t thanks to new_edge_cost.
  gain_t g1 = m[previous][current] + m[after_current][next] - new_edge_cost;
  BOOST_LOG_TRIVIAL(info) << m[previous][current] << " + "
                          << m[after_current][next] << " - " << new_edge_cost
                          << " = " << g1;

  // For target vehicle, we consider replacing "previous --> next"
  // with "previous --> current --> after_current --> next".
  if (target_rank == 0) {
    assert(v_target.has_start());
    previous = v_target.start.get().index();
  } else {
    previous = input._jobs[sol[target_vehicle][target_rank - 1]].index();
  }
  if (target_rank == sol[target_vehicle].size()) {
    assert(v_target.has_end());
    next = v_target.end.get().index();
  } else {
    next = input._jobs[sol[target_vehicle][target_rank]].index();
  }

  // Gain for target vehicle.
  gain_t old_edge_cost = m[previous][next];
  if (sol[target_vehicle].size() == 0) {
    // Adding to empty target route, so cost of start --> end without
    // job is not taken into account.
    old_edge_cost = 0;
  }
  // Implicit cast to gain_t thanks to old_edge_cost.
  gain_t g2 = old_edge_cost - m[previous][current] - m[after_current][next];
  BOOST_LOG_TRIVIAL(info) << old_edge_cost << " - " << m[previous][current]
                          << " - " << m[after_current][next] << " = " << g2;

  return g1 + g2;
}

or_opt::or_opt(const input& input,
               raw_solution& sol,
               std::vector<amount_t>& amounts,
               index_t source_vehicle,
               index_t source_rank,
               index_t target_vehicle,
               index_t target_rank)
  : _input(input),
    _sol(sol),
    _amounts(amounts),
    source_vehicle(source_vehicle),
    source_rank(source_rank),
    target_vehicle(target_vehicle),
    target_rank(target_rank),
    gain(compute_gain(input,
                      sol,
                      source_vehicle,
                      source_rank,
                      target_vehicle,
                      target_rank)) {
}

bool or_opt::is_valid() const {
  auto current_job_rank = _sol[source_vehicle][source_rank];
  // Already asserted in compute_gain.
  auto after_job_rank = _sol[source_vehicle][source_rank + 1];

  bool valid = _input.vehicle_ok_with_job(target_vehicle, current_job_rank);
  valid &= _input.vehicle_ok_with_job(target_vehicle, after_job_rank);

  valid &= (_amounts[target_vehicle] + _input._jobs[current_job_rank].amount +
              _input._jobs[after_job_rank].amount <=
            _input._vehicles[target_vehicle].capacity);

  return valid;
}

void or_opt::log() const {
  const auto& v_source = _input._vehicles[source_vehicle];
  const auto& v_target = _input._vehicles[target_vehicle];

  std::cout << "Or_Opt gain: " << gain << " - vehicle " << v_source.id
            << ", edge " << source_rank << " -> " << source_rank + 1 << " (job "
            << _input._jobs[_sol[source_vehicle][source_rank]].id << " -> "
            << _input._jobs[_sol[source_vehicle][source_rank + 1]].id
            << ") moved to rank " << target_rank << " in route for vehicle "
            << v_target.id << std::endl;
}
