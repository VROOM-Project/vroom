#ifndef VRPTW_ROUTE_SPLIT_H
#define VRPTW_ROUTE_SPLIT_H

/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/cvrp/operators/route_split.h"

namespace vroom::vrptw {

class RouteSplit : public cvrp::RouteSplit {
private:
  TWRoute& _tw_s_route;
  std::vector<TWRoute>& _tw_sol;

  void compute_gain() override;

public:
  RouteSplit(const Input& input,
             const utils::SolutionState& sol_state,
             TWRoute& tw_s_route,
             Index s_vehicle,
             const std::vector<Index>& empty_route_ranks,
             std::vector<TWRoute>& sol,
             const Eval& best_known_gain);

  void apply() override;
};

} // namespace vroom::vrptw

#endif
