/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/cvrp/local_search/relocate.h"
#include "utils/helpers.h"

cvrp_relocate::cvrp_relocate(const input& input,
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
  assert(s_rank < s_route.size());
  assert(t_rank <= t_route.size());
}

void cvrp_relocate::compute_gain() {
  const auto& m = _input.get_matrix();
  const auto& v_target = _input._vehicles[t_vehicle];

  // For source vehicle, we consider the cost of removing job at rank
  // s_rank, already stored in
  // _sol_state.node_gains[s_vehicle][s_rank].

  // For target vehicle, we consider the cost of adding source job at
  // rank t_rank.
  gain_t t_gain =
    -addition_cost(_input, m, s_route[s_rank], v_target, t_route, t_rank);

  stored_gain = _sol_state.node_gains[s_vehicle][s_rank] + t_gain;
  gain_computed = true;
}

bool cvrp_relocate::is_valid() const {
  auto relocate_job_rank = s_route[s_rank];

  bool valid = _input.vehicle_ok_with_job(t_vehicle, relocate_job_rank);

  if (_sol_state.fwd_amounts[t_vehicle].empty()) {
    valid &= (_input._jobs[relocate_job_rank].amount <=
              _input._vehicles[t_vehicle].capacity);
  } else {
    valid &= (_sol_state.fwd_amounts[t_vehicle].back() +
                _input._jobs[relocate_job_rank].amount <=
              _input._vehicles[t_vehicle].capacity);
  }

  return valid;
}

void cvrp_relocate::apply() const {
  auto relocate_job_rank = s_route[s_rank];
  s_route.erase(s_route.begin() + s_rank);
  t_route.insert(t_route.begin() + t_rank, relocate_job_rank);
}

std::vector<index_t> cvrp_relocate::addition_candidates() const {
  return {s_vehicle};
}

std::vector<index_t> cvrp_relocate::update_candidates() const {
  return {s_vehicle, t_vehicle};
}
