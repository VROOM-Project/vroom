/*

This file is part of VROOM.

Copyright (c) 2015-2022, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/cvrp/operators/route_fix.h"
#include "algorithms/heuristics/heuristics.h"

namespace vroom::cvrp {

RouteFix::RouteFix(const Input& input,
                   const utils::SolutionState& sol_state,
                   RawRoute& s_route,
                   Index s_vehicle)
  // Use dummy 0 values for unused ranks.
  : Operator(OperatorName::RouteFix,
             input,
             sol_state,
             s_route,
             s_vehicle,
             0,
             s_route,
             s_vehicle,
             0) {
  assert(s_route.size() >= 2);
}

void RouteFix::compute_gain() {
  std::vector<RawRoute> fix_sol;
  fix_sol.reserve(_input.vehicles.size());

  for (Index v = 0; v < _input.vehicles.size(); ++v) {
    fix_sol.emplace_back(_input, v, _input.zero_amount().size());
  }

  std::vector<Index> vehicles_ranks({s_vehicle});

  const auto fix_eval = heuristics::basic<RawRoute>(_input,
                                                    fix_sol,
                                                    s_route.cbegin(),
                                                    s_route.cend(),
                                                    vehicles_ranks.cbegin(),
                                                    vehicles_ranks.cend(),
                                                    INIT::NONE);

  const auto assigned_jobs = fix_sol[s_vehicle].size();

  stored_gain = (assigned_jobs < s_route.size())
                  ? NO_GAIN
                  : _sol_state.route_evals[s_vehicle] - fix_eval;
  heuristic_route = std::move(fix_sol[s_vehicle].route);
  gain_computed = true;
}

bool RouteFix::is_valid() {
  // Not supposed to be used.
  assert(false);
  return true;
}

void RouteFix::apply() {
  s_route = std::move(heuristic_route);

  source.update_amounts(_input);
}

std::vector<Index> RouteFix::addition_candidates() const {
  return {s_vehicle};
}

std::vector<Index> RouteFix::update_candidates() const {
  return {s_vehicle};
}

} // namespace vroom::cvrp
