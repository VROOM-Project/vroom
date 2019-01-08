/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/cvrp/operators/or_opt.h"

namespace vroom {
namespace cvrp {

OrOpt::OrOpt(const Input& input,
             const utils::SolutionState& sol_state,
             RawRoute& s_route,
             Index s_vehicle,
             Index s_rank,
             RawRoute& t_route,
             Index t_vehicle,
             Index t_rank)
  : Operator(input,
             sol_state,
             s_route,
             s_vehicle,
             s_rank,
             t_route,
             t_vehicle,
             t_rank),
    reverse_s_edge(false) {
  assert(s_vehicle != t_vehicle);
  assert(s_route.size() >= 2);
  assert(s_rank < s_route.size() - 1);
  assert(t_rank <= t_route.size());
}

void OrOpt::compute_gain() {
  const auto& m = _input.get_matrix();
  const auto& v_target = _input.vehicles[t_vehicle];

  // For source vehicle, we consider the cost of removing edge
  // starting at rank s_rank, already stored in
  // _sol_state.edge_gains[s_vehicle][s_rank].

  // For target vehicle, we consider the cost of adding source edge at
  // rank t_rank. reverse_* checks whether we should change the
  // source edge order.
  Index s_index = _input.jobs[s_route[s_rank]].index();
  Index after_s_index = _input.jobs[s_route[s_rank + 1]].index();

  Gain previous_cost = 0;
  Gain next_cost = 0;
  Gain reverse_previous_cost = 0;
  Gain reverse_next_cost = 0;
  Gain old_edge_cost = 0;

  if (t_rank == t_route.size()) {
    if (t_route.size() == 0) {
      // Adding edge to an empty route.
      if (v_target.has_start()) {
        previous_cost = m[v_target.start.get().index()][s_index];
        reverse_previous_cost = m[v_target.start.get().index()][after_s_index];
      }
      if (v_target.has_end()) {
        next_cost = m[after_s_index][v_target.end.get().index()];
        reverse_next_cost = m[s_index][v_target.end.get().index()];
      }
    } else {
      // Adding edge past the end after a real job.
      auto p_index = _input.jobs[t_route[t_rank - 1]].index();
      previous_cost = m[p_index][s_index];
      reverse_previous_cost = m[p_index][after_s_index];
      if (v_target.has_end()) {
        auto n_index = v_target.end.get().index();
        old_edge_cost = m[p_index][n_index];
        next_cost = m[after_s_index][n_index];
        reverse_next_cost = m[s_index][n_index];
      }
    }
  } else {
    // Adding before one of the jobs.
    auto n_index = _input.jobs[t_route[t_rank]].index();
    next_cost = m[after_s_index][n_index];
    reverse_next_cost = m[s_index][n_index];

    if (t_rank == 0) {
      if (v_target.has_start()) {
        auto p_index = v_target.start.get().index();
        previous_cost = m[p_index][s_index];
        reverse_previous_cost = m[p_index][after_s_index];
        old_edge_cost = m[p_index][n_index];
      }
    } else {
      auto p_index = _input.jobs[t_route[t_rank - 1]].index();
      previous_cost = m[p_index][s_index];
      reverse_previous_cost = m[p_index][after_s_index];
      old_edge_cost = m[p_index][n_index];
    }
  }

  // Gain for target vehicle.
  Gain t_gain = old_edge_cost - previous_cost - next_cost;

  Gain reverse_edge_cost = static_cast<Gain>(m[s_index][after_s_index]) -
                           static_cast<Gain>(m[after_s_index][s_index]);
  Gain reverse_t_gain = old_edge_cost + reverse_edge_cost -
                        reverse_previous_cost - reverse_next_cost;

  normal_stored_gain = _sol_state.edge_gains[s_vehicle][s_rank] + t_gain;
  reversed_stored_gain =
    _sol_state.edge_gains[s_vehicle][s_rank] + reverse_t_gain;

  stored_gain = normal_stored_gain;

  if (reverse_t_gain > t_gain) {
    reverse_s_edge = true;
    stored_gain = reversed_stored_gain;
  }

  gain_computed = true;
}

bool OrOpt::is_valid() {
  auto current_job_rank = s_route[s_rank];
  // Already asserted in compute_gain.
  auto after_job_rank = s_route[s_rank + 1];

  bool valid = _input.vehicle_ok_with_job(t_vehicle, current_job_rank);
  valid &= _input.vehicle_ok_with_job(t_vehicle, after_job_rank);

  if (_sol_state.fwd_amounts[t_vehicle].empty()) {
    valid &= (_input.jobs[current_job_rank].amount +
                _input.jobs[after_job_rank].amount <=
              _input.vehicles[t_vehicle].capacity);
  } else {
    valid &= (_sol_state.fwd_amounts[t_vehicle].back() +
                _input.jobs[current_job_rank].amount +
                _input.jobs[after_job_rank].amount <=
              _input.vehicles[t_vehicle].capacity);
  }

  return valid;
}

void OrOpt::apply() {
  t_route.insert(t_route.begin() + t_rank,
                 s_route.begin() + s_rank,
                 s_route.begin() + s_rank + 2);
  if (reverse_s_edge) {
    std::swap(t_route[t_rank], t_route[t_rank + 1]);
  }

  s_route.erase(s_route.begin() + s_rank, s_route.begin() + s_rank + 2);
}

std::vector<Index> OrOpt::addition_candidates() const {
  return {s_vehicle};
}

std::vector<Index> OrOpt::update_candidates() const {
  return {s_vehicle, t_vehicle};
}

} // namespace cvrp
} // namespace vroom
