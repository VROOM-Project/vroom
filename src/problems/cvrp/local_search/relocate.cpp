/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/cvrp/local_search/relocate.h"
#include "utils/helpers.h"

cvrp_relocate::cvrp_relocate(const input& input,
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
                     target_rank) {
  assert(source_vehicle != target_vehicle);
  assert(_sol[source_vehicle].size() >= 1);
  assert(source_rank < _sol[source_vehicle].size());
  assert(target_rank <= _sol[target_vehicle].size());
}

void cvrp_relocate::compute_gain() {
  const auto& m = _input.get_matrix();
  const auto& v_target = _input._vehicles[target_vehicle];

  // For source vehicle, we consider the cost of removing job at rank
  // source_rank, already stored in
  // _sol_state.node_gains[source_vehicle][source_rank].

  // For target vehicle, we consider the cost of adding source job at
  // rank target_rank.
  gain_t target_gain = -addition_cost(_input,
                                      m,
                                      _sol[source_vehicle][source_rank],
                                      v_target,
                                      _sol[target_vehicle],
                                      target_rank);

  stored_gain =
    _sol_state.node_gains[source_vehicle][source_rank] + target_gain;
  gain_computed = true;
}

bool cvrp_relocate::is_valid() const {
  auto relocate_job_rank = _sol[source_vehicle][source_rank];

  bool valid = _input.vehicle_ok_with_job(target_vehicle, relocate_job_rank);

  if (_sol_state.fwd_amounts[target_vehicle].empty()) {
    valid &= (_input._jobs[relocate_job_rank].amount <=
              _input._vehicles[target_vehicle].capacity);
  } else {
    valid &= (_sol_state.fwd_amounts[target_vehicle].back() +
                _input._jobs[relocate_job_rank].amount <=
              _input._vehicles[target_vehicle].capacity);
  }

  return valid;
}

void cvrp_relocate::apply() const {
  auto relocate_job_rank = _sol[source_vehicle][source_rank];
  _sol[source_vehicle].erase(_sol[source_vehicle].begin() + source_rank);
  _sol[target_vehicle].insert(_sol[target_vehicle].begin() + target_rank,
                              relocate_job_rank);
}

std::vector<index_t> cvrp_relocate::addition_candidates() const {
  return {source_vehicle};
}
