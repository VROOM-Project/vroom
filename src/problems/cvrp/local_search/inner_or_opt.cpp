/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/cvrp/local_search/inner_or_opt.h"
#include "utils/helpers.h"

cvrp_inner_or_opt::cvrp_inner_or_opt(const input& input,
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
                t_rank),
    reverse_s_edge(false) {
  assert(s_route.size() >= 4);
  assert(s_rank < s_route.size() - 1);
  assert(t_rank <= s_route.size() - 2);
  assert(s_rank != t_rank);
}

void cvrp_inner_or_opt::compute_gain() {
  const auto& m = _input.get_matrix();
  const auto& v = _input._vehicles[s_vehicle];

  // The cost of removing edge starting at rank s_rank is already
  // stored in _sol_state.edge_gains[s_vehicle][s_rank].

  // For addition, consider the cost of adding source edge at new rank
  // *after* removal.
  auto new_rank = t_rank;
  if (s_rank < t_rank) {
    new_rank += 2;
  }

  index_t s_index = _input._jobs[s_route[s_rank]].index();
  index_t after_s_index = _input._jobs[s_route[s_rank + 1]].index();

  gain_t previous_cost = 0;
  gain_t next_cost = 0;
  gain_t reverse_previous_cost = 0;
  gain_t reverse_next_cost = 0;
  gain_t old_edge_cost = 0;

  if (new_rank == s_route.size()) {
    // Adding edge past the end after a real job that was unmoved.
    auto p_index = _input._jobs[s_route[new_rank - 1]].index();
    previous_cost = m[p_index][s_index];
    reverse_previous_cost = m[p_index][after_s_index];
    if (v.has_end()) {
      auto n_index = v.end.get().index();
      old_edge_cost = m[p_index][n_index];
      next_cost = m[after_s_index][n_index];
      reverse_next_cost = m[s_index][n_index];
    }
  } else {
    // Adding before one of the jobs.
    auto n_index = _input._jobs[s_route[new_rank]].index();
    next_cost = m[after_s_index][n_index];
    reverse_next_cost = m[s_index][n_index];

    if (new_rank == 0) {
      if (v.has_start()) {
        auto p_index = v.start.get().index();
        previous_cost = m[p_index][s_index];
        reverse_previous_cost = m[p_index][after_s_index];
        old_edge_cost = m[p_index][n_index];
      }
    } else {
      auto p_index = _input._jobs[s_route[new_rank - 1]].index();
      previous_cost = m[p_index][s_index];
      reverse_previous_cost = m[p_index][after_s_index];
      old_edge_cost = m[p_index][n_index];
    }
  }

  // Gain for addition.
  gain_t add_gain = old_edge_cost - previous_cost - next_cost;

  gain_t reverse_edge_cost = static_cast<gain_t>(m[s_index][after_s_index]) -
                             static_cast<gain_t>(m[after_s_index][s_index]);
  gain_t reverse_add_gain = old_edge_cost + reverse_edge_cost -
                            reverse_previous_cost - reverse_next_cost;

  normal_stored_gain = _sol_state.edge_gains[s_vehicle][s_rank] + add_gain;
  reversed_stored_gain =
    _sol_state.edge_gains[s_vehicle][s_rank] + reverse_add_gain;

  stored_gain = normal_stored_gain;

  if (reverse_add_gain > add_gain) {
    reverse_s_edge = true;
    stored_gain = reversed_stored_gain;
  }

  gain_computed = true;
}

bool cvrp_inner_or_opt::is_valid() const {
  return true;
}

void cvrp_inner_or_opt::apply() const {
  auto first_job_rank = s_route[s_rank];
  auto second_job_rank = s_route[s_rank + 1];
  s_route.erase(s_route.begin() + s_rank, s_route.begin() + s_rank + 2);
  s_route.insert(s_route.begin() + t_rank, {first_job_rank, second_job_rank});
  if (reverse_s_edge) {
    std::swap(t_route[t_rank], t_route[t_rank + 1]);
  }
}

std::vector<index_t> cvrp_inner_or_opt::addition_candidates() const {
  return {};
}

std::vector<index_t> cvrp_inner_or_opt::update_candidates() const {
  return {s_vehicle};
}
