/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/cvrp/operators/intra_mixed_exchange.h"
#include "utils/helpers.h"

namespace vroom::cvrp {

IntraMixedExchange::IntraMixedExchange(const Input& input,
                                       const utils::SolutionState& sol_state,
                                       RawRoute& s_raw_route,
                                       Index s_vehicle,
                                       Index s_rank,
                                       Index t_rank,
                                       bool check_t_reverse)
  : Operator(OperatorName::IntraMixedExchange,
             input,
             sol_state,
             s_raw_route,
             s_vehicle,
             s_rank,
             s_raw_route,
             s_vehicle,
             t_rank),
    // Required for consistency in compute_gain if check_t_reverse is
    // false.
    check_t_reverse(check_t_reverse),
    _moved_jobs((s_rank < t_rank) ? t_rank - s_rank + 2 : s_rank - t_rank + 1),
    _first_rank(std::min(s_rank, t_rank)),
    _last_rank((t_rank < s_rank) ? s_rank + 1 : t_rank + 2),
    _delivery(source.delivery_in_range(_first_rank, _last_rank)) {
  // If node at s_rank is right before/after edge at t_rank, then the
  // move is a relocate.
  assert(s_rank + 1 < t_rank || t_rank + 2 < s_rank);
  assert(s_route.size() >= 4);
  assert(s_rank < s_route.size());
  assert(t_rank < s_route.size() - 1);

  // Either moving edge with single jobs or a whole shipment.
  assert((_input.jobs[this->t_route[t_rank]].type == JOB_TYPE::SINGLE &&
          _input.jobs[this->t_route[t_rank + 1]].type == JOB_TYPE::SINGLE &&
          check_t_reverse) ||
         (_input.jobs[this->t_route[t_rank]].type == JOB_TYPE::PICKUP &&
          _input.jobs[this->t_route[t_rank + 1]].type == JOB_TYPE::DELIVERY &&
          !check_t_reverse &&
          _sol_state.matching_delivery_rank[t_vehicle][t_rank] == t_rank + 1));

  Index s_node;
  if (t_rank < s_rank) {
    s_node = 0;
    _t_edge_first = _moved_jobs.size() - 2;
    _t_edge_last = _moved_jobs.size() - 1;

    std::copy(s_route.begin() + t_rank + 2,
              s_route.begin() + s_rank,
              _moved_jobs.begin() + 1);
  } else {
    _t_edge_first = 0;
    _t_edge_last = 1;
    s_node = _moved_jobs.size() - 1;

    std::copy(s_route.begin() + s_rank + 1,
              s_route.begin() + t_rank,
              _moved_jobs.begin() + 2);
  }

  _moved_jobs[s_node] = s_route[s_rank];
  _moved_jobs[_t_edge_first] = s_route[t_rank];
  _moved_jobs[_t_edge_last] = s_route[t_rank + 1];
}

Eval IntraMixedExchange::gain_upper_bound() {
  std::tie(_normal_s_gain, _reversed_s_gain) =
    utils::addition_eval_delta(_input,
                               _sol_state,
                               source,
                               s_rank,
                               s_rank + 1,
                               target,
                               t_rank,
                               t_rank + 2);

  auto s_gain_upper_bound = _normal_s_gain;

  if (check_t_reverse) {
    s_gain_upper_bound = std::max(s_gain_upper_bound, _reversed_s_gain);
  }

  t_gain = std::get<0>(utils::addition_eval_delta(_input,
                                                  _sol_state,
                                                  source,
                                                  t_rank,
                                                  t_rank + 2,
                                                  source,
                                                  s_rank,
                                                  s_rank + 1));

  _gain_upper_bound_computed = true;

  return s_gain_upper_bound + t_gain;
}

void IntraMixedExchange::compute_gain() {
  assert(_gain_upper_bound_computed);
  assert(s_is_normal_valid || s_is_reverse_valid);
  if (_normal_s_gain < _reversed_s_gain) {
    // Biggest potential gain is obtained when reversing edge.
    if (s_is_reverse_valid) {
      stored_gain += _reversed_s_gain;
      reverse_t_edge = true;
    } else {
      stored_gain += _normal_s_gain;
    }
  } else {
    // Biggest potential gain is obtained when not reversing edge.
    if (s_is_normal_valid) {
      stored_gain += _normal_s_gain;
    } else {
      stored_gain += _reversed_s_gain;
      reverse_t_edge = true;
    }
  }

  stored_gain += t_gain;

  gain_computed = true;
}

bool IntraMixedExchange::is_valid() {
  assert(_gain_upper_bound_computed);

  const auto& s_v = _input.vehicles[s_vehicle];
  const auto& s_eval = _sol_state.route_evals[s_vehicle];
  const auto normal_eval = _normal_s_gain + t_gain;

  s_is_normal_valid =
    s_v.ok_for_range_bounds(s_eval - normal_eval) &&
    source.is_valid_addition_for_capacity_inclusion(_input,
                                                    _delivery,
                                                    _moved_jobs.begin(),
                                                    _moved_jobs.end(),
                                                    _first_rank,
                                                    _last_rank);

  if (check_t_reverse) {
    const auto reversed_eval = _reversed_s_gain + t_gain;

    if (s_v.ok_for_range_bounds(s_eval - reversed_eval)) {
      std::swap(_moved_jobs[_t_edge_first], _moved_jobs[_t_edge_last]);

      s_is_reverse_valid =
        source.is_valid_addition_for_capacity_inclusion(_input,
                                                        _delivery,
                                                        _moved_jobs.begin(),
                                                        _moved_jobs.end(),
                                                        _first_rank,
                                                        _last_rank);

      // Reset to initial situation before potential application or TW
      // checks.
      std::swap(_moved_jobs[_t_edge_first], _moved_jobs[_t_edge_last]);
    }
  }

  return s_is_normal_valid || s_is_reverse_valid;
}

void IntraMixedExchange::apply() {
  assert(!reverse_t_edge ||
         (_input.jobs[t_route[t_rank]].type == JOB_TYPE::SINGLE &&
          _input.jobs[t_route[t_rank + 1]].type == JOB_TYPE::SINGLE));

  if (reverse_t_edge) {
    std::swap(s_route[t_rank], s_route[t_rank + 1]);
  }

  std::swap(s_route[s_rank], s_route[t_rank]);

  auto t_after = s_route[t_rank + 1];
  s_route.erase(s_route.begin() + t_rank + 1);

  auto end_t_rank = s_rank + 1;
  if (t_rank < s_rank) {
    --end_t_rank;
  }

  s_route.insert(s_route.begin() + end_t_rank, t_after);

  source.update_amounts(_input);
}

std::vector<Index> IntraMixedExchange::addition_candidates() const {
  return {};
}

std::vector<Index> IntraMixedExchange::update_candidates() const {
  return {s_vehicle};
}

} // namespace vroom::cvrp
