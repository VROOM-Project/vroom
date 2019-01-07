#ifndef VRPTW_RELOCATE_H
#define VRPTW_RELOCATE_H

/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/cvrp/operators/relocate.h"
#include "structures/vroom/tw_route.h"

namespace vroom {

class vrptwRelocate : public CVRPRelocate {
private:
  TWRoute& _tw_s_route;
  TWRoute& _tw_t_route;

public:
  vrptwRelocate(const Input& input,
                const SolutionState& sol_state,
                TWRoute& tw_s_route,
                Index s_vehicle,
                Index s_rank,
                TWRoute& tw_t_route,
                Index t_vehicle,
                Index t_rank);

  virtual bool is_valid() override;

  virtual void apply() override;
};

} // namespace vroom

#endif
