/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/cvrp/operators/cross_exchange.h"
#include "utils/helpers.h"

namespace vroom::cvrp {

CrossExchange::CrossExchange(const Input& input,
                             const utils::SolutionState& sol_state,
                             RawRoute& s_route,
                             Index s_vehicle,
                             Index s_rank,
                             RawRoute& t_route,
                             Index t_vehicle,
                             Index t_rank,
                             bool check_s_reverse,
                             bool check_t_reverse)
  : Operator(OperatorName::CrossExchange,
             input,
             sol_state,
             s_route,
             s_vehicle,
             s_rank,
             t_route,
             t_vehicle,
             t_rank),
    // Required for consistency in compute_gain if check_s_reverse or
    // check_t_reverse are false.
    check_s_reverse(check_s_reverse),
    check_t_reverse(check_t_reverse),
    source_delivery(_input.jobs[this->s_route[s_rank]].delivery +
                    _input.jobs[this->s_route[s_rank + 1]].delivery),
    target_delivery(_input.jobs[this->t_route[t_rank]].delivery +
                    _input.jobs[this->t_route[t_rank + 1]].delivery) {
  assert(s_vehicle != t_vehicle);
  assert(s_route.size() >= 2);
  assert(t_route.size() >= 2);
  assert(s_rank < s_route.size() - 1);
  assert(t_rank < t_route.size() - 1);

  assert(_input.vehicle_ok_with_job(t_vehicle, this->s_route[s_rank]));
  assert(_input.vehicle_ok_with_job(t_vehicle, this->s_route[s_rank + 1]));
  assert(_input.vehicle_ok_with_job(s_vehicle, this->t_route[t_rank]));
  assert(_input.vehicle_ok_with_job(s_vehicle, this->t_route[t_rank + 1]));

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
}

Eval CrossExchange::gain_upper_bound() {
  std::tie(_normal_s_gain, _reversed_s_gain) =
    utils::addition_eval_delta(_input,
                               _sol_state,
                               source,
                               s_rank,
                               s_rank + 2,
                               target,
                               t_rank,
                               t_rank + 2);

  auto s_gain_upper_bound = _normal_s_gain;

  if (check_t_reverse) {
    s_gain_upper_bound = std::max(s_gain_upper_bound, _reversed_s_gain);
  }

  std::tie(_normal_t_gain, _reversed_t_gain) =
    utils::addition_eval_delta(_input,
                               _sol_state,
                               target,
                               t_rank,
                               t_rank + 2,
                               source,
                               s_rank,
                               s_rank + 2);

  auto t_gain_upper_bound = _normal_t_gain;

  if (check_s_reverse) {
    t_gain_upper_bound = std::max(t_gain_upper_bound, _reversed_t_gain);
  }

  _gain_upper_bound_computed = true;

  return s_gain_upper_bound + t_gain_upper_bound;
}

void CrossExchange::compute_gain() {
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

  assert(t_is_normal_valid || t_is_reverse_valid);
  if (_normal_t_gain < _reversed_t_gain) {
    // Biggest potential gain is obtained when reversing edge.
    if (t_is_reverse_valid) {
      stored_gain += _reversed_t_gain;
      reverse_s_edge = true;
    } else {
      stored_gain += _normal_t_gain;
    }
  } else {
    // Biggest potential gain is obtained when not reversing edge.
    if (t_is_normal_valid) {
      stored_gain += _normal_t_gain;
    } else {
      stored_gain += _reversed_t_gain;
      reverse_s_edge = true;
    }
  }

  gain_computed = true;
}

bool CrossExchange::is_valid() {
  assert(_gain_upper_bound_computed);

  auto target_pickup = _input.jobs[t_route[t_rank]].pickup +
                       _input.jobs[t_route[t_rank + 1]].pickup;

  bool valid = source.is_valid_addition_for_capacity_margins(_input,
                                                             target_pickup,
                                                             target_delivery,
                                                             s_rank,
                                                             s_rank + 2);

  if (valid) {
    const auto& s_v = _input.vehicles[s_vehicle];
    const auto& s_eval = _sol_state.route_evals[s_vehicle];

    // Keep target edge direction when inserting in source route.
    auto t_start = t_route.begin() + t_rank;
    s_is_normal_valid =
      s_v.ok_for_range_bounds(s_eval - _normal_s_gain) &&
      source.is_valid_addition_for_capacity_inclusion(_input,
                                                      target_delivery,
                                                      t_start,
                                                      t_start + 2,
                                                      s_rank,
                                                      s_rank + 2);

    if (check_t_reverse) {
      // Reverse target edge direction when inserting in source route.
      auto t_reverse_start = t_route.rbegin() + t_route.size() - 2 - t_rank;
      s_is_reverse_valid =
        s_v.ok_for_range_bounds(s_eval - _reversed_s_gain) &&
        source.is_valid_addition_for_capacity_inclusion(_input,
                                                        target_delivery,
                                                        t_reverse_start,
                                                        t_reverse_start + 2,
                                                        s_rank,
                                                        s_rank + 2);
    }

    valid = s_is_normal_valid || s_is_reverse_valid;
  }

  auto source_pickup = _input.jobs[s_route[s_rank]].pickup +
                       _input.jobs[s_route[s_rank + 1]].pickup;

  valid =
    valid && target.is_valid_addition_for_capacity_margins(_input,
                                                           source_pickup,
                                                           source_delivery,
                                                           t_rank,
                                                           t_rank + 2);

  if (valid) {
    const auto& t_v = _input.vehicles[t_vehicle];
    const auto& t_eval = _sol_state.route_evals[t_vehicle];

    // Keep source edge direction when inserting in target route.
    auto s_start = s_route.begin() + s_rank;
    t_is_normal_valid =
      t_v.ok_for_range_bounds(t_eval - _normal_t_gain) &&
      target.is_valid_addition_for_capacity_inclusion(_input,
                                                      source_delivery,
                                                      s_start,
                                                      s_start + 2,
                                                      t_rank,
                                                      t_rank + 2);

    if (check_s_reverse) {
      // Reverse source edge direction when inserting in target route.
      auto s_reverse_start = s_route.rbegin() + s_route.size() - 2 - s_rank;
      t_is_reverse_valid =
        t_v.ok_for_range_bounds(t_eval - _reversed_t_gain) &&
        target.is_valid_addition_for_capacity_inclusion(_input,
                                                        source_delivery,
                                                        s_reverse_start,
                                                        s_reverse_start + 2,
                                                        t_rank,
                                                        t_rank + 2);
    }

    valid = t_is_normal_valid || t_is_reverse_valid;
  }

  return valid;
}

void CrossExchange::apply() {
  assert(!reverse_s_edge ||
         (_input.jobs[s_route[s_rank]].type == JOB_TYPE::SINGLE &&
          _input.jobs[s_route[s_rank + 1]].type == JOB_TYPE::SINGLE));
  assert(!reverse_t_edge ||
         (_input.jobs[t_route[t_rank]].type == JOB_TYPE::SINGLE &&
          _input.jobs[t_route[t_rank + 1]].type == JOB_TYPE::SINGLE));

  std::swap(s_route[s_rank], t_route[t_rank]);
  std::swap(s_route[s_rank + 1], t_route[t_rank + 1]);

  if (reverse_s_edge) {
    std::swap(t_route[t_rank], t_route[t_rank + 1]);
  }
  if (reverse_t_edge) {
    std::swap(s_route[s_rank], s_route[s_rank + 1]);
  }

  source.update_amounts(_input);
  target.update_amounts(_input);
}

std::vector<Index> CrossExchange::addition_candidates() const {
  return {s_vehicle, t_vehicle};
}

std::vector<Index> CrossExchange::update_candidates() const {
  return {s_vehicle, t_vehicle};
}

} // namespace vroom::cvrp
