#ifndef VRPTW_ROUTE_SPLIT_H
#define VRPTW_ROUTE_SPLIT_H

/*

This file is part of VROOM.

Copyright (c) 2015-2022, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/cvrp/operators/route_split.h"

namespace vroom::vrptw {

class RouteSplit : public cvrp::RouteSplit {
private:
  TWRoute& _tw_s_route;
  const std::vector<std::reference_wrapper<TWRoute>> _empty_tw_route_refs;

  void compute_gain() override;

public:
  RouteSplit(const Input& input,
             const utils::SolutionState& sol_state,
             TWRoute& tw_s_route,
             Index s_vehicle,
             const std::vector<Index>& empty_route_ranks,
             std::vector<std::reference_wrapper<TWRoute>>& empty_route_refs,
             const Eval& best_known_gain);

  void apply() override;
};

} // namespace vroom::vrptw

#endif
