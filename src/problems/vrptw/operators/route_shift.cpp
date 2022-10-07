/*

This file is part of VROOM.

Copyright (c) 2015-2022, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/vrptw/operators/route_shift.h"

namespace vroom {
namespace vrptw {

RouteShift::RouteShift(const Input& input,
                       const utils::SolutionState& sol_state,
                       TWRoute& tw_s_route,
                       Index s_vehicle,
                       TWRoute& tw_t_route,
                       Index t_vehicle)
  : cvrp::RouteShift(input,
                     sol_state,
                     static_cast<RawRoute&>(tw_s_route),
                     s_vehicle,
                     static_cast<RawRoute&>(tw_t_route),
                     t_vehicle),
    _tw_s_route(tw_s_route),
    _tw_t_route(tw_t_route) {
}

bool RouteShift::is_valid() {
  bool valid = cvrp::RouteShift::is_valid();

  if (valid) {
    is_start_valid =
      is_start_valid && _tw_t_route.is_valid_addition_for_tw(_input,
                                                             s_route.begin(),
                                                             s_route.end(),
                                                             0,
                                                             0);

    is_end_valid =
      is_end_valid && _tw_t_route.is_valid_addition_for_tw(_input,
                                                           s_route.begin(),
                                                           s_route.end(),
                                                           t_route.size(),
                                                           t_route.size());

    valid = is_start_valid or is_end_valid;
  }

  return valid;
}

void RouteShift::apply() {
  if (shift_to_start) {
    _tw_t_route.replace(_input, s_route.begin(), s_route.end(), 0, 0);
  } else {
    assert(shift_to_end);

    _tw_t_route.replace(_input,
                        s_route.begin(),
                        s_route.end(),
                        t_route.size(),
                        t_route.size());
  }

  _tw_s_route.remove(_input, 0, s_route.size());

  source.update_amounts(_input);
  target.update_amounts(_input);
}

} // namespace vrptw
} // namespace vroom
