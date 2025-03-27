#ifndef VRPTW_TSP_FIX_H
#define VRPTW_TSP_FIX_H

/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/cvrp/operators/tsp_fix.h"

namespace vroom::vrptw {

class TSPFix : public cvrp::TSPFix {
private:
  TWRoute& _tw_s_route;

public:
  TSPFix(const Input& input,
         const utils::SolutionState& sol_state,
         TWRoute& tw_s_route,
         Index s_vehicle);

  bool is_valid() override;

  void apply() override;
};

} // namespace vroom::vrptw

#endif
