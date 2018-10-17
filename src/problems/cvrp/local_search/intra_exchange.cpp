/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/cvrp/local_search/intra_exchange.h"
#include "utils/helpers.h"

cvrp_intra_exchange::cvrp_intra_exchange(const input& input,
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
  // Assume s_rank < t_rank for symmetry reasons. Set aside cases
  // where t_rank = s_rank + 1, as the move is also an intra_relocate.
  assert(0 < t_rank);
  assert(s_rank < t_rank - 1);
  assert(s_route.size() >= 3);
  assert(t_rank < s_route.size());
}

void cvrp_intra_exchange::compute_gain() {
  const auto& m = _input.get_matrix();
  const auto& v = _input._vehicles[s_vehicle];

  // Consider the cost of replacing job at rank s_rank with target
  // job. Part of that cost (for adjacent edges) is stored in
  // _sol_state.edge_costs_around_node.
  index_t s_index = _input._jobs[s_route[s_rank]].index();
  index_t t_index = _input._jobs[t_route[t_rank]].index();

  // Determine costs added with target job.
  gain_t new_previous_cost = 0;

  if (s_rank == 0) {
    if (v.has_start()) {
      auto p_index = v.start.get().index();
      new_previous_cost = m[p_index][t_index];
    }
  } else {
    auto p_index = _input._jobs[s_route[s_rank - 1]].index();
    new_previous_cost = m[p_index][t_index];
  }

  auto n_index = _input._jobs[s_route[s_rank + 1]].index();
  gain_t new_next_cost = m[t_index][n_index];

  gain_t s_gain = _sol_state.edge_costs_around_node[s_vehicle][s_rank] -
                  new_previous_cost - new_next_cost;

  // Consider the cost of replacing job at rank t_rank with source
  // job. Part of that cost (for adjacent edges) is stored in
  // _sol_state.edge_costs_around_node.

  // Determine costs added with source job.
  new_next_cost = 0;

  auto p_index = _input._jobs[t_route[t_rank - 1]].index();
  new_previous_cost = m[p_index][s_index];

  if (t_rank == t_route.size() - 1) {
    if (v.has_end()) {
      auto n_index = v.end.get().index();
      new_next_cost = m[s_index][n_index];
    }
  } else {
    auto n_index = _input._jobs[t_route[t_rank + 1]].index();
    new_next_cost = m[s_index][n_index];
  }

  gain_t t_gain = _sol_state.edge_costs_around_node[s_vehicle][t_rank] -
                  new_previous_cost - new_next_cost;

  stored_gain = s_gain + t_gain;
  gain_computed = true;
}

bool cvrp_intra_exchange::is_valid() {
  return true;
}

void cvrp_intra_exchange::apply() {
  std::swap(s_route[s_rank], t_route[t_rank]);
}

std::vector<index_t> cvrp_intra_exchange::addition_candidates() const {
  return {};
}

std::vector<index_t> cvrp_intra_exchange::update_candidates() const {
  return {s_vehicle};
}
