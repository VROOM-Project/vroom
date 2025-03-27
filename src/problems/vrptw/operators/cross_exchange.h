#ifndef VRPTW_CROSS_EXCHANGE_H
#define VRPTW_CROSS_EXCHANGE_H

/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/cvrp/operators/cross_exchange.h"

namespace vroom::vrptw {

class CrossExchange : public cvrp::CrossExchange {
private:
  TWRoute& _tw_s_route;
  TWRoute& _tw_t_route;

public:
  CrossExchange(const Input& input,
                const utils::SolutionState& sol_state,
                TWRoute& tw_s_route,
                Index s_vehicle,
                Index s_rank,
                TWRoute& tw_t_route,
                Index t_vehicle,
                Index t_rank,
                bool check_s_reverse,
                bool check_t_reverse);

  bool is_valid() override;

  void apply() override;
};

} // namespace vroom::vrptw

#endif
