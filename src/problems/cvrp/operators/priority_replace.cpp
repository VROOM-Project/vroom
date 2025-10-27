/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <algorithm>

#include "utils/helpers.h"

#include "problems/cvrp/operators/priority_replace.h"

namespace vroom::cvrp {

PriorityReplace::PriorityReplace(const Input& input,
                                 const utils::SolutionState& sol_state,
                                 std::unordered_set<Index>& unassigned,
                                 RawRoute& s_raw_route,
                                 Index s_vehicle,
                                 Index s_rank,
                                 Index t_rank,
                                 Index u,
                                 Priority best_known_priority_gain)
  : Operator(OperatorName::PriorityReplace,
             input,
             sol_state,
             s_raw_route,
             s_vehicle,
             s_rank,
             s_raw_route,
             s_vehicle,
             t_rank),
    _start_priority_gain(_input.jobs[u].priority -
                         _sol_state.fwd_priority[s_vehicle][s_rank]),
    _end_priority_gain(_input.jobs[u].priority -
                       _sol_state.bwd_priority[s_vehicle][t_rank]),
    _start_assigned_number(s_route.size() - s_rank),
    _end_assigned_number(t_rank + 1),
    _u(u),
    _best_known_priority_gain(best_known_priority_gain),
    _unassigned(unassigned) {
  assert(!s_route.empty());
  assert(t_rank != 0);
  assert(_start_priority_gain > 0 or _end_priority_gain > 0);
}

void PriorityReplace::compute_start_gain() {
  s_gain =
    utils::addition_eval_delta(_input, _sol_state, source, 0, s_rank + 1, _u);

  _start_gain_computed = true;
}

void PriorityReplace::compute_end_gain() {
  t_gain = utils::addition_eval_delta(_input,
                                      _sol_state,
                                      source,
                                      t_rank,
                                      source.size(),
                                      _u);

  _end_gain_computed = true;
}

void PriorityReplace::compute_gain() {
  assert(replace_start_valid || replace_end_valid);
  assert(_start_gain_computed || !replace_start_valid);
  assert(_end_gain_computed || !replace_end_valid);

  if (replace_start_valid && replace_end_valid) {
    // Decide based on priority and cost.
    if (std::tie(_end_priority_gain, _end_assigned_number, t_gain) <
        std::tie(_start_priority_gain, _start_assigned_number, s_gain)) {
      replace_end_valid = false;
    } else {
      replace_start_valid = false;
    }
  }

  if (replace_start_valid) {
    stored_gain = s_gain;
  } else {
    assert(replace_end_valid);
    stored_gain = t_gain;
  }

  gain_computed = true;
}

bool PriorityReplace::is_valid() {
  const auto& j = _input.jobs[_u];

  // Early abort if priority gain is not interesting anyway or the
  // move is not interesting: s_rank is zero if the candidate start
  // portion is empty or with a single job (that would be an
  // UnassignedExchange move).
  replace_start_valid =
    (0 < _start_priority_gain) &&
    (_best_known_priority_gain <= _start_priority_gain) && (s_rank > 0) &&
    source.is_valid_addition_for_capacity_margins(_input,
                                                  j.pickup,
                                                  j.delivery,
                                                  0,
                                                  s_rank + 1);
  assert(!replace_start_valid ||
         !source.has_pending_delivery_after_rank(s_rank));

  // Don't bother if the candidate end portion is empty or with a
  // single job (that would be an UnassignedExchange move).
  replace_end_valid =
    (0 < _end_priority_gain) &&
    (_best_known_priority_gain <= _end_priority_gain) &&
    (t_rank < s_route.size() - 1) &&
    source.is_valid_addition_for_capacity_margins(_input,
                                                  j.pickup,
                                                  j.delivery,
                                                  t_rank,
                                                  s_route.size());
  assert(!replace_end_valid ||
         !source.has_pending_delivery_after_rank(t_rank - 1));

  // Check validity with regard to vehicle range bounds, requires
  // valid gain values for both options.
  if (replace_start_valid) {
    this->compute_start_gain();

    replace_start_valid = is_valid_for_source_range_bounds();
  }

  if (replace_end_valid) {
    this->compute_end_gain();

    replace_end_valid = is_valid_for_target_range_bounds();
  }

  return replace_start_valid || replace_end_valid;
}

void PriorityReplace::apply() {
  assert(_unassigned.contains(_u));
  _unassigned.erase(_u);

  const std::vector<Index> addition({_u});

  assert(replace_start_valid xor replace_end_valid);

  if (replace_start_valid) {
    assert(
      std::all_of(s_route.cbegin(),
                  s_route.cbegin() + s_rank + 1,
                  [this](const auto j) { return !_unassigned.contains(j); }));
    _unassigned.insert(s_route.cbegin(), s_route.cbegin() + s_rank + 1);

    source.replace(_input, addition.begin(), addition.end(), 0, s_rank + 1);
  } else {
    assert(
      std::all_of(s_route.cbegin() + t_rank,
                  s_route.cend(),
                  [this](const auto j) { return !_unassigned.contains(j); }));
    _unassigned.insert(s_route.cbegin() + t_rank, s_route.cend());

    source.replace(_input,
                   addition.begin(),
                   addition.end(),
                   t_rank,
                   s_route.size());
  }
}

Priority PriorityReplace::priority_gain() {
  if (!gain_computed) {
    // Priority gain depends on actual option, decided in compute_gain.
    this->compute_gain();
  }

  assert(replace_start_valid xor replace_end_valid);

  return replace_start_valid ? _start_priority_gain : _end_priority_gain;
}

unsigned PriorityReplace::assigned() const {
  assert(gain_computed);
  assert(replace_start_valid xor replace_end_valid);

  return replace_start_valid ? _start_assigned_number : _end_assigned_number;
}

std::vector<Index> PriorityReplace::addition_candidates() const {
  return {s_vehicle};
}

std::vector<Index> PriorityReplace::update_candidates() const {
  return {s_vehicle};
}

std::vector<Index> PriorityReplace::required_unassigned() const {
  return {_u};
}

} // namespace vroom::cvrp
