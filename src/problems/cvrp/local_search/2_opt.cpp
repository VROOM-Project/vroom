/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <iostream>

#include "2_opt.h"

two_opt::two_opt(const input& input,
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

void two_opt::compute_gain() {
  auto& m = _input.get_matrix();
  const auto& v_source = _input._vehicles[source_vehicle];
  const auto& v_target = _input._vehicles[target_vehicle];

  index_t s_index = _input._jobs[_sol[source_vehicle][source_rank]].index();
  index_t t_index = _input._jobs[_sol[target_vehicle][target_rank]].index();
  index_t last_s = _input._jobs[_sol[source_vehicle].back()].index();
  index_t last_t = _input._jobs[_sol[target_vehicle].back()].index();
  stored_gain = 0;
  index_t new_last_s = last_t;
  index_t new_last_t = last_s;

  // Cost of swapping route for vehicle source_vehicle after step
  // source_rank with route for vehicle target_vehicle after step
  // target_rank.

  // Basic costs in case we really swap jobs and not only the end of
  // the route. Otherwise remember that last job does not change.
  if (source_rank < _sol[source_vehicle].size() - 1) {
    index_t next_index =
      _input._jobs[_sol[source_vehicle][source_rank + 1]].index();
    stored_gain += m[s_index][next_index];
    stored_gain -= m[t_index][next_index];
  } else {
    new_last_t = t_index;
  }
  if (target_rank < _sol[target_vehicle].size() - 1) {
    index_t next_index =
      _input._jobs[_sol[target_vehicle][target_rank + 1]].index();
    stored_gain += m[t_index][next_index];
    stored_gain -= m[s_index][next_index];
  } else {
    new_last_s = s_index;
  }

  // Handling end route cost change because vehicle ends can be
  // different or none.
  if (v_source.has_end()) {
    auto end_s = v_source.end.get().index();
    stored_gain += m[last_s][end_s];
    stored_gain -= m[new_last_s][end_s];
  }
  if (v_target.has_end()) {
    auto end_t = v_target.end.get().index();
    stored_gain += m[last_t][end_t];
    stored_gain -= m[new_last_t][end_t];
  }

  gain_computed = true;
}

bool two_opt::is_valid() const {
  bool valid = true;
  for (auto it = _sol[source_vehicle].begin() + source_rank + 1;
       it < _sol[source_vehicle].end() and valid;
       ++it) {
    valid &= _input.vehicle_ok_with_job(target_vehicle, *it);
  }
  for (auto it = _sol[target_vehicle].begin() + target_rank + 1;
       it < _sol[target_vehicle].end() and valid;
       ++it) {
    valid &= _input.vehicle_ok_with_job(source_vehicle, *it);
  }

  valid &= (fwd_amounts[source_vehicle][source_rank] +
              bwd_amounts[target_vehicle][target_rank] <=
            _input._vehicles[source_vehicle].capacity);
  valid &= (fwd_amounts[target_vehicle][target_rank] +
              bwd_amounts[source_vehicle][source_rank] <=
            _input._vehicles[target_vehicle].capacity);

  return valid;
}

void two_opt::apply() const {
  auto nb_source = _sol[source_vehicle].size() - 1 - source_rank;

  _sol[target_vehicle].insert(_sol[target_vehicle].begin() + target_rank + 1,
                              _sol[source_vehicle].begin() + source_rank + 1,
                              _sol[source_vehicle].end());
  _sol[source_vehicle].erase(_sol[source_vehicle].begin() + source_rank + 1,
                             _sol[source_vehicle].end());
  _sol[source_vehicle].insert(_sol[source_vehicle].end(),
                              _sol[target_vehicle].begin() + target_rank + 1 +
                                nb_source,
                              _sol[target_vehicle].end());
  _sol[target_vehicle].erase(_sol[target_vehicle].begin() + target_rank + 1 +
                               nb_source,
                             _sol[target_vehicle].end());
}

void two_opt::log() const {
  const auto& v_source = _input._vehicles[source_vehicle];
  const auto& v_target = _input._vehicles[target_vehicle];

  std::cout << "2-opt* gain: " << stored_gain << " - swap route for vehicle "
            << v_source.id << " after step " << source_rank << " (job "
            << _input._jobs[_sol[source_vehicle][source_rank]].id
            << ") with route for vehicle " << v_target.id << " after step "
            << target_rank << " (job "
            << _input._jobs[_sol[target_vehicle][target_rank]].id << ")"
            << std::endl;
}

std::vector<index_t> two_opt::addition_candidates() const {
  return {source_vehicle, target_vehicle};
}
