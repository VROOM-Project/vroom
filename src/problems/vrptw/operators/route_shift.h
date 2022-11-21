#ifndef VRPTW_ROUTE_SHIFT_H
#define VRPTW_ROUTE_SHIFT_H

/*

This file is part of VROOM.

Copyright (c) 2015-2022, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/cvrp/operators/route_shift.h"

namespace vroom {
namespace vrptw {

class RouteShift : public cvrp::RouteShift {
private:
  TWRoute& _tw_s_route;
  TWRoute& _tw_t_route;

public:
  RouteShift(const Input& input,
             const utils::SolutionState& sol_state,
             TWRoute& tw_s_route,
             Index s_vehicle,
             TWRoute& tw_t_route,
             Index t_vehicle);

  virtual bool is_valid() override;

  virtual void apply() override;
};

} // namespace vrptw
} // namespace vroom

#endif
