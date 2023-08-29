#ifndef VRPTW_ROUTE_FIX_H
#define VRPTW_ROUTE_FIX_H

/*

This file is part of VROOM.

Copyright (c) 2015-2022, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/cvrp/operators/route_fix.h"

namespace vroom::vrptw {

class RouteFix : public cvrp::RouteFix {
private:
  TWRoute& _tw_s_route;

public:
  RouteFix(const Input& input,
           const utils::SolutionState& sol_state,
           TWRoute& tw_s_route,
           Index s_vehicle);

  bool is_valid() override;

  void apply() override;
};

} // namespace vroom::vrptw

#endif
