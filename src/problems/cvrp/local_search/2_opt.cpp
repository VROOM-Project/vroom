/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/cvrp/local_search/2_opt.h"

cvrp_two_opt::cvrp_two_opt(const input& input,
                           const solution_state& sol_state,
                           std::vector<index_t>& s_route,
                           index_t s_vehicle,
                           index_t s_rank,
                           std::vector<index_t>& t_route,
                           index_t t_vehicle,
                           index_t t_rank)
  : ls_operator(input,
                sol_state,
                s_route,
                s_vehicle,
                s_rank,
                t_route,
                t_vehicle,
                t_rank) {
  assert(s_vehicle != t_vehicle);
  assert(s_route.size() >= 1);
  assert(t_route.size() >= 1);
  assert(s_rank < s_route.size());
  assert(t_rank < t_route.size());
}

void cvrp_two_opt::compute_gain() {
  const auto& m = _input.get_matrix();
  const auto& v_source = _input._vehicles[s_vehicle];
  const auto& v_target = _input._vehicles[t_vehicle];

  index_t s_index = _input._jobs[s_route[s_rank]].index();
  index_t t_index = _input._jobs[t_route[t_rank]].index();
  index_t last_s = _input._jobs[s_route.back()].index();
  index_t last_t = _input._jobs[t_route.back()].index();
  stored_gain = 0;
  index_t new_last_s = last_t;
  index_t new_last_t = last_s;

  // Cost of swapping route for vehicle s_vehicle after step
  // s_rank with route for vehicle t_vehicle after step
  // t_rank.

  // Basic costs in case we really swap jobs and not only the end of
  // the route. Otherwise remember that last job does not change.
  if (s_rank < s_route.size() - 1) {
    index_t next_index = _input._jobs[s_route[s_rank + 1]].index();
    stored_gain += m[s_index][next_index];
    stored_gain -= m[t_index][next_index];
  } else {
    new_last_t = t_index;
  }
  if (t_rank < t_route.size() - 1) {
    index_t next_index = _input._jobs[t_route[t_rank + 1]].index();
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
  bool valid = (_sol_state.bwd_skill_rank[s_vehicle][t_vehicle] <= s_rank + 1);

  valid &= (_sol_state.bwd_skill_rank[t_vehicle][s_vehicle] <= t_rank + 1);

  valid &= (_sol_state.fwd_amounts[s_vehicle][s_rank] +
              _sol_state.bwd_amounts[t_vehicle][t_rank] <=
            _input._vehicles[s_vehicle].capacity);
  valid &= (_sol_state.fwd_amounts[t_vehicle][t_rank] +
              _sol_state.bwd_amounts[s_vehicle][s_rank] <=
            _input._vehicles[t_vehicle].capacity);

  return valid;
}

void cvrp_two_opt::apply() const {
  auto nb_source = s_route.size() - 1 - s_rank;

  t_route.insert(t_route.begin() + t_rank + 1,
                 s_route.begin() + s_rank + 1,
                 s_route.end());
  s_route.erase(s_route.begin() + s_rank + 1, s_route.end());
  s_route.insert(s_route.end(),
                 t_route.begin() + t_rank + 1 + nb_source,
                 t_route.end());
  t_route.erase(t_route.begin() + t_rank + 1 + nb_source, t_route.end());
}

std::vector<index_t> cvrp_two_opt::addition_candidates() const {
  return {s_vehicle, t_vehicle};
}

std::vector<index_t> cvrp_two_opt::update_candidates() const {
  return {s_vehicle, t_vehicle};
}
