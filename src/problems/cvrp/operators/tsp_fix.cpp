/*

This file is part of VROOM.

Copyright (c) 2015-2022, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/cvrp/operators/tsp_fix.h"
#include "problems/tsp/tsp.h"

namespace vroom::cvrp {

TSPFix::TSPFix(const Input& input,
               const utils::SolutionState& sol_state,
               RawRoute& s_route,
               Index s_vehicle)
  // Use dummy 0 values for unused ranks.
  : Operator(OperatorName::TSPFix,
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

void TSPFix::compute_gain() {
  std::vector<Index> jobs = s_route;
  TSP tsp(_input, std::move(jobs), s_vehicle);
  tsp_route = tsp.raw_solve(1, Timeout());

  const auto tsp_eval =
    utils::route_eval_for_vehicle(_input, s_vehicle, tsp_route);

  stored_gain = _sol_state.route_evals[s_vehicle] - tsp_eval;
  gain_computed = true;
}

bool TSPFix::is_valid() {
  // TODO check!
  return true;
}

void TSPFix::apply() {
  s_route = std::move(tsp_route);

  source.update_amounts(_input);
}

std::vector<Index> TSPFix::addition_candidates() const {
  return {s_vehicle};
}

std::vector<Index> TSPFix::update_candidates() const {
  return {s_vehicle};
}

} // namespace vroom::cvrp
