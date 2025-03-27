/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/cvrp/operators/intra_cross_exchange.h"

namespace vroom::cvrp {

IntraCrossExchange::IntraCrossExchange(const Input& input,
                                       const utils::SolutionState& sol_state,
                                       RawRoute& s_raw_route,
                                       Index s_vehicle,
                                       Index s_rank,
                                       Index t_rank,
                                       bool check_s_reverse,
                                       bool check_t_reverse)
  : Operator(OperatorName::IntraCrossExchange,
             input,
             sol_state,
             s_raw_route,
             s_vehicle,
             s_rank,
             s_raw_route,
             s_vehicle,
             t_rank),
    // Required for consistency in compute_gain if check_s_reverse or
    // check_t_reverse are false.
    check_s_reverse(check_s_reverse),
    check_t_reverse(check_t_reverse),
    _moved_jobs(t_rank - s_rank + 2),
    _first_rank(s_rank),
    _last_rank(t_rank + 2),
    _delivery(source.delivery_in_range(_first_rank, _last_rank)) {
  // Use s_rank as smallest rank for symmetry reasons.
  assert(s_rank + 2 < t_rank); // Avoid common edge.
  assert(s_route.size() >= 5);
  assert(t_rank < s_route.size() - 1);

  // Either moving edges of single jobs or whole shipments.
  assert((_input.jobs[this->s_route[s_rank]].type == JOB_TYPE::SINGLE &&
          _input.jobs[this->s_route[s_rank + 1]].type == JOB_TYPE::SINGLE &&
          check_s_reverse) ||
         (_input.jobs[this->s_route[s_rank]].type == JOB_TYPE::PICKUP &&
          _input.jobs[this->s_route[s_rank + 1]].type == JOB_TYPE::DELIVERY &&
          !check_s_reverse &&
          _sol_state.matching_delivery_rank[s_vehicle][s_rank] == s_rank + 1));
  assert((_input.jobs[this->t_route[t_rank]].type == JOB_TYPE::SINGLE &&
          _input.jobs[this->t_route[t_rank + 1]].type == JOB_TYPE::SINGLE &&
          check_t_reverse) ||
         (_input.jobs[this->t_route[t_rank]].type == JOB_TYPE::PICKUP &&
          _input.jobs[this->t_route[t_rank + 1]].type == JOB_TYPE::DELIVERY &&
          !check_t_reverse &&
          _sol_state.matching_delivery_rank[t_vehicle][t_rank] == t_rank + 1));

  _moved_jobs[0] = s_route[t_rank];
  _moved_jobs[1] = s_route[t_rank + 1];
  std::copy(s_route.begin() + s_rank + 2,
            s_route.begin() + t_rank,
            _moved_jobs.begin() + 2);
  _moved_jobs[_moved_jobs.size() - 2] = s_route[s_rank];
  _moved_jobs[_moved_jobs.size() - 1] = s_route[s_rank + 1];
}

Eval IntraCrossExchange::gain_upper_bound() {
  const auto& v = _input.vehicles[s_vehicle];

  // Consider the cost of replacing edge starting at rank s_rank with
  // target edge. Part of that cost (for adjacent edges) is stored in
  // _sol_state.edge_evals_around_edge.  reverse_* checks whether we
  // should change the target edge order.
  const Index s_index = _input.jobs[s_route[s_rank]].index();
  const Index s_after_index = _input.jobs[s_route[s_rank + 1]].index();
  const Index t_index = _input.jobs[s_route[t_rank]].index();
  const Index t_after_index = _input.jobs[s_route[t_rank + 1]].index();

  // Determine costs added with target edge.
  Eval previous_cost;
  Eval next_cost;
  Eval reverse_previous_cost;
  Eval reverse_next_cost;

  if (s_rank == 0) {
    if (v.has_start()) {
      auto p_index = v.start.value().index();
      previous_cost = v.eval(p_index, t_index);
      reverse_previous_cost = v.eval(p_index, t_after_index);
    }
  } else {
    auto p_index = _input.jobs[s_route[s_rank - 1]].index();
    previous_cost = v.eval(p_index, t_index);
    reverse_previous_cost = v.eval(p_index, t_after_index);
  }

  auto n_index = _input.jobs[s_route[s_rank + 2]].index();
  next_cost = v.eval(t_after_index, n_index);
  reverse_next_cost = v.eval(t_index, n_index);

  _normal_s_gain = _sol_state.edge_evals_around_edge[s_vehicle][s_rank] -
                   previous_cost - next_cost;

  auto s_gain_upper_bound = _normal_s_gain;

  if (check_t_reverse) {
    const auto reverse_edge_cost =
      v.eval(t_index, t_after_index) - v.eval(t_after_index, t_index);
    _reversed_s_gain = _sol_state.edge_evals_around_edge[s_vehicle][s_rank] +
                       reverse_edge_cost - reverse_previous_cost -
                       reverse_next_cost;

    s_gain_upper_bound = std::max(_normal_s_gain, _reversed_s_gain);
  }

  // Consider the cost of replacing edge starting at rank t_rank with
  // source edge. Part of that cost (for adjacent edges) is stored in
  // _sol_state.edge_evals_around_edge.  reverse_* checks whether we
  // should change the source edge order.
  next_cost = Eval();
  reverse_previous_cost = Eval();
  reverse_next_cost = Eval();

  auto p_index = _input.jobs[s_route[t_rank - 1]].index();
  previous_cost = v.eval(p_index, s_index);
  reverse_previous_cost = v.eval(p_index, s_after_index);

  if (t_rank == s_route.size() - 2) {
    if (v.has_end()) {
      n_index = v.end.value().index();
      next_cost = v.eval(s_after_index, n_index);
      reverse_next_cost = v.eval(s_index, n_index);
    }
  } else {
    n_index = _input.jobs[s_route[t_rank + 2]].index();
    next_cost = v.eval(s_after_index, n_index);
    reverse_next_cost = v.eval(s_index, n_index);
  }

  _normal_t_gain = _sol_state.edge_evals_around_edge[t_vehicle][t_rank] -
                   previous_cost - next_cost;

  auto t_gain_upper_bound = _normal_t_gain;

  if (check_s_reverse) {
    const auto reverse_edge_cost =
      v.eval(s_index, s_after_index) - v.eval(s_after_index, s_index);
    _reversed_t_gain = _sol_state.edge_evals_around_edge[t_vehicle][t_rank] +
                       reverse_edge_cost - reverse_previous_cost -
                       reverse_next_cost;

    t_gain_upper_bound = std::max(_normal_t_gain, _reversed_t_gain);
  }

  _gain_upper_bound_computed = true;

  return s_gain_upper_bound + t_gain_upper_bound;
}

void IntraCrossExchange::compute_gain() {
  assert(_gain_upper_bound_computed);
  assert(s_normal_t_normal_is_valid || s_normal_t_reverse_is_valid ||
         s_reverse_t_reverse_is_valid || s_reverse_t_normal_is_valid);

  stored_gain = NO_GAIN;

  if (s_normal_t_normal_is_valid) {
    const auto current_gain = _normal_s_gain + _normal_t_gain;
    if (stored_gain < current_gain) {
      stored_gain = current_gain;
      reverse_s_edge = false;
      reverse_t_edge = false;
    }
  }

  if (s_normal_t_reverse_is_valid) {
    const auto current_gain = _reversed_s_gain + _normal_t_gain;
    if (stored_gain < current_gain) {
      stored_gain = current_gain;
      reverse_s_edge = false;
      reverse_t_edge = true;
    }
  }

  if (s_reverse_t_reverse_is_valid) {
    const auto current_gain = _reversed_s_gain + _reversed_t_gain;
    if (stored_gain < current_gain) {
      stored_gain = current_gain;
      reverse_s_edge = true;
      reverse_t_edge = true;
    }
  }

  if (s_reverse_t_normal_is_valid) {
    const auto current_gain = _normal_s_gain + _reversed_t_gain;
    if (stored_gain < current_gain) {
      stored_gain = current_gain;
      reverse_s_edge = true;
      reverse_t_edge = false;
    }
  }

  gain_computed = true;
}

bool IntraCrossExchange::is_valid() {
  assert(_gain_upper_bound_computed);

  const auto& s_v = _input.vehicles[s_vehicle];
  const auto& s_eval = _sol_state.route_evals[s_vehicle];
  const auto s_normal_t_normal_eval = _normal_s_gain + _normal_t_gain;

  s_normal_t_normal_is_valid =
    s_v.ok_for_range_bounds(s_eval - s_normal_t_normal_eval) &&
    source.is_valid_addition_for_capacity_inclusion(_input,
                                                    _delivery,
                                                    _moved_jobs.begin(),
                                                    _moved_jobs.end(),
                                                    _first_rank,
                                                    _last_rank);

  std::swap(_moved_jobs[0], _moved_jobs[1]);

  if (check_t_reverse) {
    const auto s_normal_t_reverse_eval = _reversed_s_gain + _normal_t_gain;

    s_normal_t_reverse_is_valid =
      s_v.ok_for_range_bounds(s_eval - s_normal_t_reverse_eval) &&
      source.is_valid_addition_for_capacity_inclusion(_input,
                                                      _delivery,
                                                      _moved_jobs.begin(),
                                                      _moved_jobs.end(),
                                                      _first_rank,
                                                      _last_rank);
  }

  std::swap(_moved_jobs[_moved_jobs.size() - 2],
            _moved_jobs[_moved_jobs.size() - 1]);

  if (check_s_reverse && check_t_reverse) {
    const auto s_reversed_t_reversed_eval = _reversed_s_gain + _reversed_t_gain;
    s_reverse_t_reverse_is_valid =
      s_v.ok_for_range_bounds(s_eval - s_reversed_t_reversed_eval) &&
      source.is_valid_addition_for_capacity_inclusion(_input,
                                                      _delivery,
                                                      _moved_jobs.begin(),
                                                      _moved_jobs.end(),
                                                      _first_rank,
                                                      _last_rank);
  }

  std::swap(_moved_jobs[0], _moved_jobs[1]);

  if (check_s_reverse) {
    const auto s_reverse_t_normal_eval = _normal_s_gain + _reversed_t_gain;

    s_reverse_t_normal_is_valid =
      s_v.ok_for_range_bounds(s_eval - s_reverse_t_normal_eval) &&
      source.is_valid_addition_for_capacity_inclusion(_input,
                                                      _delivery,
                                                      _moved_jobs.begin(),
                                                      _moved_jobs.end(),
                                                      _first_rank,
                                                      _last_rank);
  }

  // Reset to initial situation before potential application and TW
  // checks.
  std::swap(_moved_jobs[_moved_jobs.size() - 2],
            _moved_jobs[_moved_jobs.size() - 1]);

  return s_normal_t_normal_is_valid || s_normal_t_reverse_is_valid ||
         s_reverse_t_reverse_is_valid || s_reverse_t_normal_is_valid;
}

void IntraCrossExchange::apply() {
  assert(!reverse_s_edge ||
         (_input.jobs[s_route[s_rank]].type == JOB_TYPE::SINGLE &&
          _input.jobs[s_route[s_rank + 1]].type == JOB_TYPE::SINGLE));
  assert(!reverse_t_edge ||
         (_input.jobs[t_route[t_rank]].type == JOB_TYPE::SINGLE &&
          _input.jobs[t_route[t_rank + 1]].type == JOB_TYPE::SINGLE));

  std::swap(s_route[s_rank], s_route[t_rank]);
  std::swap(s_route[s_rank + 1], s_route[t_rank + 1]);

  if (reverse_s_edge) {
    std::swap(s_route[t_rank], s_route[t_rank + 1]);
  }
  if (reverse_t_edge) {
    std::swap(s_route[s_rank], s_route[s_rank + 1]);
  }

  source.update_amounts(_input);
}

std::vector<Index> IntraCrossExchange::addition_candidates() const {
  return {};
}

std::vector<Index> IntraCrossExchange::update_candidates() const {
  return {s_vehicle};
}

} // namespace vroom::cvrp
