#ifndef VRPTW_UNASSIGNED_EXCHANGE_H
#define VRPTW_UNASSIGNED_EXCHANGE_H

/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/cvrp/operators/unassigned_exchange.h"

namespace vroom::vrptw {

class UnassignedExchange : public cvrp::UnassignedExchange {
private:
  TWRoute& _tw_s_route;

public:
  UnassignedExchange(const Input& input,
                     const utils::SolutionState& sol_state,
                     std::unordered_set<Index>& unassigned,
                     TWRoute& tw_s_route,
                     Index s_vehicle,
                     Index s_rank,
                     Index t_rank,
                     Index u);

  bool is_valid() override;

  void apply() override;
};

} // namespace vroom::vrptw

#endif
