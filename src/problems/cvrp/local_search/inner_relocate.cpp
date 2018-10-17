/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/cvrp/local_search/inner_relocate.h"
#include "utils/helpers.h"

cvrp_inner_relocate::cvrp_inner_relocate(const input& input,
                                         const solution_state& sol_state,
                                         std::vector<index_t>& s_route,
                                         index_t s_vehicle,
                                         index_t s_rank,
                                         index_t t_rank)
  : ls_operator(input,
                sol_state,
                s_route,
                s_vehicle,
                s_rank,
                s_route,
                s_vehicle,
                t_rank) {
  assert(s_route.size() >= 2);
  assert(s_rank < s_route.size());
  assert(t_rank <= s_route.size() - 1);
  assert(s_rank != t_rank);
}

void cvrp_inner_relocate::compute_gain() {
  const auto& m = _input.get_matrix();
  const auto& v_target = _input._vehicles[s_vehicle];

  // For removal, we consider the cost of removing job at rank s_rank,
  // already stored in _sol_state.node_gains[s_vehicle][s_rank].

  // For addition, consider the cost of adding source job at new rank
  // *after* removal.
  auto new_rank = t_rank;
  if (s_rank < t_rank) {
    ++new_rank;
  }
  gain_t t_gain =
    -addition_cost(_input, m, s_route[s_rank], v_target, t_route, new_rank);

  stored_gain = _sol_state.node_gains[s_vehicle][s_rank] + t_gain;
  gain_computed = true;
}

bool cvrp_inner_relocate::is_valid() {
  return true;
}

void cvrp_inner_relocate::apply() {
  auto relocate_job_rank = s_route[s_rank];
  s_route.erase(s_route.begin() + s_rank);
  t_route.insert(t_route.begin() + t_rank, relocate_job_rank);
}

std::vector<index_t> cvrp_inner_relocate::addition_candidates() const {
  return {};
}

std::vector<index_t> cvrp_inner_relocate::update_candidates() const {
  return {s_vehicle};
}
