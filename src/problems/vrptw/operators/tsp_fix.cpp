/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/vrptw/operators/tsp_fix.h"

namespace vroom::vrptw {

TSPFix::TSPFix(const Input& input,
               const utils::SolutionState& sol_state,
               TWRoute& tw_s_route,
               Index s_vehicle)
  : cvrp::TSPFix(input,
                 sol_state,
                 static_cast<RawRoute&>(tw_s_route),
                 s_vehicle),
    _tw_s_route(tw_s_route) {
}

bool TSPFix::is_valid() {
  return cvrp::TSPFix::is_valid() &&
         _tw_s_route.is_valid_addition_for_tw(_input,
                                              _s_delivery,
                                              tsp_route.begin(),
                                              tsp_route.end(),
                                              0,
                                              s_route.size());
}

void TSPFix::apply() {
  _tw_s_route.replace(_input,
                      source.job_deliveries_sum(),
                      tsp_route.begin(),
                      tsp_route.end(),
                      0,
                      s_route.size());
}

} // namespace vroom::vrptw
