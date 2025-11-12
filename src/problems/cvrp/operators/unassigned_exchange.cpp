/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/cvrp/operators/unassigned_exchange.h"
#include "utils/helpers.h"

namespace vroom::cvrp {

UnassignedExchange::UnassignedExchange(const Input& input,
                                       const utils::SolutionState& sol_state,
                                       std::unordered_set<Index>& unassigned,
                                       RawRoute& s_raw_route,
                                       Index s_vehicle,
                                       Index s_rank,
                                       Index t_rank,
                                       Index u)
  : Operator(OperatorName::UnassignedExchange,
             input,
             sol_state,
             s_raw_route,
             s_vehicle,
             s_rank,
             s_raw_route,
             s_vehicle,
             t_rank),
    _u(u),
    _unassigned(unassigned),
    _first_rank(std::min(s_rank, t_rank)),
    _last_rank((s_rank < t_rank) ? t_rank : s_rank + 1),
    _moved_jobs(_last_rank - _first_rank),
    _removed(s_route[s_rank]),
    _delivery(source.delivery_in_range(_first_rank, _last_rank)) {
  assert(t_rank != s_rank + 1);
  assert(!s_route.empty());
  assert(s_rank < s_route.size());
  assert(t_rank <= s_route.size());

  assert(_input.jobs[_removed].delivery <= _delivery);
  _delivery -= _input.jobs[_removed].delivery;
  _delivery += _input.jobs[_u].delivery;

  if (s_rank < t_rank) {
    std::copy(s_route.begin() + s_rank + 1,
              s_route.begin() + t_rank,
              _moved_jobs.begin());
    _moved_jobs.back() = u;
  } else {
    std::copy(s_route.begin() + t_rank,
              s_route.begin() + s_rank,
              _moved_jobs.begin() + 1);
    _moved_jobs.front() = u;
  }
}

void UnassignedExchange::compute_gain() {
  if (t_rank == s_rank) {
    s_gain = utils::addition_eval_delta(_input,
                                        _sol_state,
                                        source,
                                        s_rank,
                                        s_rank + 1,
                                        _u);
  } else {
    // No common edge so both gains can be computed independently.
    const auto& v = _input.vehicles[s_vehicle];

    s_gain = _sol_state.node_gains[s_vehicle][s_rank] -
             utils::addition_eval(_input, _u, v, s_route, t_rank);
  }

  stored_gain = s_gain;
  gain_computed = true;
}

bool UnassignedExchange::is_valid() {
  auto pickup = source.pickup_in_range(_first_rank, _last_rank);
  assert(_input.jobs[_removed].pickup <= pickup);
  pickup -= _input.jobs[_removed].pickup;
  pickup += _input.jobs[_u].pickup;

  bool valid = source.is_valid_addition_for_capacity_margins(_input,
                                                             pickup,
                                                             _delivery,
                                                             _first_rank,
                                                             _last_rank);

  valid = valid &&
          source.is_valid_addition_for_capacity_inclusion(_input,
                                                          _delivery,
                                                          _moved_jobs.begin(),
                                                          _moved_jobs.end(),
                                                          _first_rank,
                                                          _last_rank);

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

void UnassignedExchange::apply() {
  std::ranges::copy(_moved_jobs, s_route.begin() + _first_rank);

  assert(_unassigned.contains(_u));
  _unassigned.erase(_u);
  assert(!_unassigned.contains(_removed));
  _unassigned.insert(_removed);

  source.update_amounts(_input);
}

std::vector<Index> UnassignedExchange::addition_candidates() const {
  return _input.compatible_vehicles_for_job[_removed];
}

std::vector<Index> UnassignedExchange::update_candidates() const {
  return {s_vehicle};
}

std::vector<Index> UnassignedExchange::required_unassigned() const {
  return {_u};
}

} // namespace vroom::cvrp
