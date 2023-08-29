/*

This file is part of VROOM.

Copyright (c) 2015-2022, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/vrptw/operators/route_fix.h"

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

bool RouteFix::is_valid() {
  // TODO check!
  return true;
}

void RouteFix::apply() {
  _tw_s_route.replace(_input,
                      source.job_deliveries_sum(),
                      tsp_route.begin(),
                      tsp_route.end(),
                      0,
                      s_route.size());
}

} // namespace vroom::vrptw
