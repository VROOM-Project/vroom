#ifndef VRPTW_ROUTE_EXCHANGE_H
#define VRPTW_ROUTE_EXCHANGE_H

/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/cvrp/operators/route_exchange.h"

namespace vroom::vrptw {

class RouteExchange : public cvrp::RouteExchange {
private:
  TWRoute& _tw_s_route;
  TWRoute& _tw_t_route;

  const Amount _source_job_deliveries_sum;
  const Amount _target_job_deliveries_sum;

public:
  RouteExchange(const Input& input,
                const utils::SolutionState& sol_state,
                TWRoute& tw_s_route,
                Index s_vehicle,
                TWRoute& tw_t_route,
                Index t_vehicle);

  bool is_valid() override;

  void apply() override;
};

} // namespace vroom::vrptw

#endif
