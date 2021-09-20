/*

This file is part of VROOM.

Copyright (c) 2015-2021, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/vrptw/operators/swap_star.h"

namespace vroom {
namespace vrptw {

SwapStar::SwapStar(const Input& input,
                   const utils::SolutionState& sol_state,
                   TWRoute& tw_s_route,
                   Index s_vehicle,
                   TWRoute& tw_t_route,
                   Index t_vehicle)
  : cvrp::SwapStar(input,
                   sol_state,
                   static_cast<RawRoute&>(tw_s_route),
                   s_vehicle,
                   static_cast<RawRoute&>(tw_t_route),
                   t_vehicle),
    _tw_s_route(tw_s_route),
    _tw_t_route(tw_t_route) {
}

void SwapStar::apply() {
  // TODO
}

} // namespace vrptw
} // namespace vroom
