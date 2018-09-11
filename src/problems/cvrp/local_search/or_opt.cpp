/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/cvrp/local_search/or_opt.h"

cvrp_or_opt::cvrp_or_opt(const input& input,
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
                     target_rank),
    reverse_source_edge(false) {
  assert(source_vehicle != target_vehicle);
  assert(_sol[source_vehicle].size() >= 2);
  assert(source_rank < _sol[source_vehicle].size() - 1);
  assert(target_rank <= _sol[target_vehicle].size());
}

void cvrp_or_opt::compute_gain() {
  const auto& m = _input.get_matrix();
  const auto& v_target = _input._vehicles[target_vehicle];

  // For source vehicle, we consider the cost of removing edge
  // starting at rank source_rank, already stored in
  // _sol_state.edge_gains[source_vehicle][source_rank].

  // For target vehicle, we consider the cost of adding source edge at
  // rank target_rank. reverse_* checks whether we should change the
  // source edge order.
  index_t c_index = _input._jobs[_sol[source_vehicle][source_rank]].index();
  index_t after_c_index =
    _input._jobs[_sol[source_vehicle][source_rank + 1]].index();

  gain_t previous_cost = 0;
  gain_t next_cost = 0;
  gain_t reverse_previous_cost = 0;
  gain_t reverse_next_cost = 0;
  gain_t old_edge_cost = 0;

  if (target_rank == _sol[target_vehicle].size()) {
    if (_sol[target_vehicle].size() == 0) {
      // Adding edge to an empty route.
      if (v_target.has_start()) {
        previous_cost = m[v_target.start.get().index()][c_index];
        reverse_previous_cost = m[v_target.start.get().index()][after_c_index];
      }
      if (v_target.has_end()) {
        next_cost = m[after_c_index][v_target.end.get().index()];
        reverse_next_cost = m[c_index][v_target.end.get().index()];
      }
    } else {
      // Adding edge past the end after a real job.
      auto p_index =
        _input._jobs[_sol[target_vehicle][target_rank - 1]].index();
      previous_cost = m[p_index][c_index];
      reverse_previous_cost = m[p_index][after_c_index];
      if (v_target.has_end()) {
        auto n_index = v_target.end.get().index();
        old_edge_cost = m[p_index][n_index];
        next_cost = m[after_c_index][n_index];
        reverse_next_cost = m[c_index][n_index];
      }
    }
  } else {
    // Adding before one of the jobs.
    auto n_index = _input._jobs[_sol[target_vehicle][target_rank]].index();
    next_cost = m[after_c_index][n_index];
    reverse_next_cost = m[c_index][n_index];

    if (target_rank == 0) {
      if (v_target.has_start()) {
        auto p_index = v_target.start.get().index();
        previous_cost = m[p_index][c_index];
        reverse_previous_cost = m[p_index][after_c_index];
        old_edge_cost = m[p_index][n_index];
      }
    } else {
      auto p_index =
        _input._jobs[_sol[target_vehicle][target_rank - 1]].index();
      previous_cost = m[p_index][c_index];
      reverse_previous_cost = m[p_index][after_c_index];
      old_edge_cost = m[p_index][n_index];
    }
  }

  // Gain for target vehicle.
  gain_t target_gain = old_edge_cost - previous_cost - next_cost;

  gain_t reverse_edge_cost = static_cast<gain_t>(m[c_index][after_c_index]) -
                             static_cast<gain_t>(m[after_c_index][c_index]);
  gain_t reverse_target_gain = old_edge_cost + reverse_edge_cost -
                               reverse_previous_cost - reverse_next_cost;

  if (reverse_target_gain > target_gain) {
    reverse_source_edge = true;
    target_gain = reverse_target_gain;
  }

  stored_gain =
    _sol_state.edge_gains[source_vehicle][source_rank] + target_gain;

  gain_computed = true;
}

bool cvrp_or_opt::is_valid() const {
  auto current_job_rank = _sol[source_vehicle][source_rank];
  // Already asserted in compute_gain.
  auto after_job_rank = _sol[source_vehicle][source_rank + 1];

  bool valid = _input.vehicle_ok_with_job(target_vehicle, current_job_rank);
  valid &= _input.vehicle_ok_with_job(target_vehicle, after_job_rank);

  if (_sol_state.fwd_amounts[target_vehicle].empty()) {
    valid &= (_input._jobs[current_job_rank].amount +
                _input._jobs[after_job_rank].amount <=
              _input._vehicles[target_vehicle].capacity);
  } else {
    valid &= (_sol_state.fwd_amounts[target_vehicle].back() +
                _input._jobs[current_job_rank].amount +
                _input._jobs[after_job_rank].amount <=
              _input._vehicles[target_vehicle].capacity);
  }

  return valid;
}

void cvrp_or_opt::apply() const {
  _sol[target_vehicle].insert(_sol[target_vehicle].begin() + target_rank,
                              _sol[source_vehicle].begin() + source_rank,
                              _sol[source_vehicle].begin() + source_rank + 2);
  if (reverse_source_edge) {
    std::swap(_sol[target_vehicle][target_rank],
              _sol[target_vehicle][target_rank + 1]);
  }

  _sol[source_vehicle].erase(_sol[source_vehicle].begin() + source_rank,
                             _sol[source_vehicle].begin() + source_rank + 2);
}

std::vector<index_t> cvrp_or_opt::addition_candidates() const {
  return {source_vehicle};
}
