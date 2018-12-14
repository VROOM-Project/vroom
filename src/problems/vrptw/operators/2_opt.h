#ifndef VRPTW_TWO_OPT_H
#define VRPTW_TWO_OPT_H

/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/cvrp/operators/2_opt.h"
#include "structures/vroom/tw_route.h"

class vrptwTwoOpt : public CVRPTwoOpt {
private:
  TWRoute& _tw_s_route;
  TWRoute& _tw_t_route;

public:
  vrptwTwoOpt(const Input& input,
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

#endif
