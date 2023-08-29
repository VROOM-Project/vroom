/*

This file is part of VROOM.

Copyright (c) 2015-2022, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/vrptw/operators/route_fix.h"
#include "algorithms/heuristics/heuristics.h"

namespace vroom::vrptw {

RouteFix::RouteFix(const Input& input,
                   const utils::SolutionState& sol_state,
                   TWRoute& tw_s_route,
                   Index s_vehicle)
  : cvrp::RouteFix(input,
                   sol_state,
                   static_cast<RawRoute&>(tw_s_route),
                   s_vehicle),
    _tw_s_route(tw_s_route) {
}

void RouteFix::compute_gain() {
  // Similar to cvrp::RouteFix::compute_gain but makes sure to
  // trigger heuristics::basic<TWRoute>.
  std::vector<TWRoute> fix_sol;
  fix_sol.reserve(_input.vehicles.size());

  for (Index v = 0; v < _input.vehicles.size(); ++v) {
    fix_sol.emplace_back(_input, v, _input.zero_amount().size());
  }

  std::vector<Index> vehicles_ranks({s_vehicle});

  const auto fix_eval = heuristics::basic<TWRoute>(_input,
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
  _tw_s_route.replace(_input,
                      source.job_deliveries_sum(),
                      heuristic_route.begin(),
                      heuristic_route.end(),
                      0,
                      s_route.size());
}

} // namespace vroom::vrptw
