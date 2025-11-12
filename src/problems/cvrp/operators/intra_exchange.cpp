/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/cvrp/operators/intra_exchange.h"
#include "utils/helpers.h"

namespace vroom::cvrp {

IntraExchange::IntraExchange(const Input& input,
                             const utils::SolutionState& sol_state,
                             RawRoute& s_raw_route,
                             Index s_vehicle,
                             Index s_rank,
                             Index t_rank)
  : Operator(OperatorName::IntraExchange,
             input,
             sol_state,
             s_raw_route,
             s_vehicle,
             s_rank,
             s_raw_route,
             s_vehicle,
             t_rank),
    _moved_jobs(t_rank - s_rank + 1),
    _first_rank(s_rank),
    _last_rank(t_rank + 1),
    _delivery(source.delivery_in_range(_first_rank, _last_rank)) {
  // Assume s_rank < t_rank for symmetry reasons. Set aside cases
  // where t_rank = s_rank + 1, as the move is also an intra_relocate.
  assert(0 < t_rank);
  assert(s_rank < t_rank - 1);
  assert(s_route.size() >= 3);
  assert(t_rank < s_route.size());

  std::copy(s_route.begin() + _first_rank,
            s_route.begin() + _last_rank,
            _moved_jobs.begin());
  std::swap(_moved_jobs[0], _moved_jobs.back());
}

void IntraExchange::compute_gain() {
  s_gain = _sol_state.node_gains[s_vehicle][s_rank] -
           utils::in_place_delta_eval(_input,
                                      s_route[t_rank],
                                      _input.vehicles[s_vehicle],
                                      s_route,
                                      s_rank);

  t_gain = _sol_state.node_gains[s_vehicle][t_rank] -
           utils::in_place_delta_eval(_input,
                                      s_route[s_rank],
                                      _input.vehicles[s_vehicle],
                                      s_route,
                                      t_rank);

  stored_gain = s_gain + t_gain;
  gain_computed = true;
}

bool IntraExchange::is_valid() {
  return is_valid_for_range_bounds() &&
         source.is_valid_addition_for_capacity_inclusion(_input,
                                                         _delivery,
                                                         _moved_jobs.begin(),
                                                         _moved_jobs.end(),
                                                         _first_rank,
                                                         _last_rank);
}

void IntraExchange::apply() {
  std::swap(s_route[s_rank], t_route[t_rank]);

  source.update_amounts(_input);
}

std::vector<Index> IntraExchange::addition_candidates() const {
  return {};
}

std::vector<Index> IntraExchange::update_candidates() const {
  return {s_vehicle};
}

} // namespace vroom::cvrp
