/*

This file is part of VROOM.

Copyright (c) 2015-2019, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/cvrp/operators/intra_or_opt.h"
#include "utils/helpers.h"

namespace vroom {
namespace cvrp {

IntraOrOpt::IntraOrOpt(const Input& input,
                       const utils::SolutionState& sol_state,
                       RawRoute& s_route,
                       Index s_vehicle,
                       Index s_rank,
                       Index t_rank)
  : Operator(input,
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

void IntraOrOpt::compute_gain() {
  const auto& m = _input.get_matrix();
  const auto& v = _input.vehicles[s_vehicle];

  // The cost of removing edge starting at rank s_rank is already
  // stored in _sol_state.edge_gains[s_vehicle][s_rank].

  // For addition, consider the cost of adding source edge at new rank
  // *after* removal.
  auto new_rank = t_rank;
  if (s_rank < t_rank) {
    new_rank += 2;
  }

  Index s_index = _input.jobs[s_route[s_rank]].index();
  Index after_s_index = _input.jobs[s_route[s_rank + 1]].index();

  Gain previous_cost = 0;
  Gain next_cost = 0;
  Gain reverse_previous_cost = 0;
  Gain reverse_next_cost = 0;
  Gain old_edge_cost = 0;

  if (new_rank == s_route.size()) {
    // Adding edge past the end after a real job that was unmoved.
    auto p_index = _input.jobs[s_route[new_rank - 1]].index();
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
    auto n_index = _input.jobs[s_route[new_rank]].index();
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
      auto p_index = _input.jobs[s_route[new_rank - 1]].index();
      previous_cost = m[p_index][s_index];
      reverse_previous_cost = m[p_index][after_s_index];
      old_edge_cost = m[p_index][n_index];
    }
  }

  // Gain for addition.
  Gain add_gain = old_edge_cost - previous_cost - next_cost;

  Gain reverse_edge_cost = static_cast<Gain>(m[s_index][after_s_index]) -
                           static_cast<Gain>(m[after_s_index][s_index]);
  Gain reverse_add_gain = old_edge_cost + reverse_edge_cost -
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

bool IntraOrOpt::is_valid() {
  return true;
}

void IntraOrOpt::apply() {
  auto first_job_rank = s_route[s_rank];
  auto second_job_rank = s_route[s_rank + 1];
  s_route.erase(s_route.begin() + s_rank, s_route.begin() + s_rank + 2);
  s_route.insert(s_route.begin() + t_rank, {first_job_rank, second_job_rank});
  if (reverse_s_edge) {
    std::swap(t_route[t_rank], t_route[t_rank + 1]);
  }
}

std::vector<Index> IntraOrOpt::addition_candidates() const {
  return {};
}

std::vector<Index> IntraOrOpt::update_candidates() const {
  return {s_vehicle};
}

} // namespace cvrp
} // namespace vroom
