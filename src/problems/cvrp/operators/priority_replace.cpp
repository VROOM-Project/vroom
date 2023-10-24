/*

This file is part of VROOM.

Copyright (c) 2015-2022, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <algorithm>

#include "problems/cvrp/operators/priority_replace.h"
#include "utils/helpers.h"

namespace vroom::cvrp {

PriorityReplace::PriorityReplace(const Input& input,
                                 const utils::SolutionState& sol_state,
                                 std::unordered_set<Index>& unassigned,
                                 RawRoute& s_raw_route,
                                 Index s_vehicle,
                                 Index s_rank,
                                 Index u)
  : Operator(OperatorName::PriorityReplace,
             input,
             sol_state,
             s_raw_route,
             s_vehicle,
             s_rank,
             s_raw_route,
             s_vehicle,
             0),
    _u(u),
    _unassigned(unassigned) {
  assert(!s_route.empty());
}

void PriorityReplace::compute_gain() {
  const auto& v = _input.vehicles[s_vehicle];

  const Index u_index = _input.jobs[_u].index();

  // Replace full beginning or route up to s_rank with _u.
  s_gain = _sol_state.fwd_costs[s_vehicle][s_vehicle][s_rank];

  if (v.has_start()) {
    s_gain += v.eval(v.start.value().index(), _input.jobs[s_route[0]].index());
    s_gain -= v.eval(v.start.value().index(), u_index);
  }

  const Index s_index = _input.jobs[s_route[s_rank]].index();

  if (s_rank == s_route.size() - 1) {
    if (v.has_end()) {
      s_gain += v.eval(s_index, v.end.value().index());
      s_gain -= v.eval(u_index, v.end.value().index());
    }
  } else {
    const auto s_next_index = _input.jobs[s_route[s_rank + 1]].index();
    s_gain += v.eval(s_index, s_next_index);
    s_gain -= v.eval(u_index, s_next_index);
  }

  stored_gain = s_gain;
  gain_computed = true;
}

bool PriorityReplace::is_valid() {
  const auto& j = _input.jobs[_u];

  bool valid = source.is_valid_addition_for_capacity_margins(_input,
                                                             j.pickup,
                                                             j.delivery,
                                                             0,
                                                             s_rank + 1);

  if (valid) {
    // Check validity with regard to vehicle range bounds, requires
    // valid gain value.
    if (!gain_computed) {
      // We don't check gain before validity if priority is strictly
      // improved.
      this->compute_gain();
    }

    valid = is_valid_for_source_range_bounds();
  }

  return valid;
}

void PriorityReplace::apply() {
  assert(_unassigned.contains(_u));
  _unassigned.erase(_u);

  assert(
    std::all_of(s_route.cbegin(),
                s_route.cbegin() + s_rank + 1,
                [this](const auto j) { return !_unassigned.contains(j); }));
  _unassigned.insert(s_route.cbegin(), s_route.cbegin() + s_rank + 1);

  const std::vector<Index> addition({_u});
  source.replace(_input, addition.begin(), addition.end(), 0, s_rank + 1);
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
