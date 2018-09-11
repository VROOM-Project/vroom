/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/cvrp/local_search/2_opt.h"

cvrp_two_opt::cvrp_two_opt(const input& input,
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
  assert(_sol[target_vehicle].size() >= 1);
  assert(source_rank < _sol[source_vehicle].size());
  assert(target_rank < _sol[target_vehicle].size());
}

void cvrp_two_opt::compute_gain() {
  const auto& m = _input.get_matrix();
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

bool cvrp_two_opt::is_valid() const {
  bool valid = (_sol_state.bwd_skill_rank[source_vehicle][target_vehicle] <=
                source_rank + 1);

  valid &= (_sol_state.bwd_skill_rank[target_vehicle][source_vehicle] <=
            target_rank + 1);

  valid &= (_sol_state.fwd_amounts[source_vehicle][source_rank] +
              _sol_state.bwd_amounts[target_vehicle][target_rank] <=
            _input._vehicles[source_vehicle].capacity);
  valid &= (_sol_state.fwd_amounts[target_vehicle][target_rank] +
              _sol_state.bwd_amounts[source_vehicle][source_rank] <=
            _input._vehicles[target_vehicle].capacity);

  return valid;
}

void cvrp_two_opt::apply() const {
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

std::vector<index_t> cvrp_two_opt::addition_candidates() const {
  return {source_vehicle, target_vehicle};
}
