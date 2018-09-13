/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/vrptw/local_search/exchange.h"

vrptw_exchange::vrptw_exchange(const input& input,
                               const solution_state& sol_state,
                               tw_solution& tw_sol,
                               index_t s_vehicle,
                               index_t s_rank,
                               index_t t_vehicle,
                               index_t t_rank)
  : cvrp_exchange(input,
                  sol_state,
                  tw_sol[s_vehicle].route,
                  s_vehicle,
                  s_rank,
                  tw_sol[t_vehicle].route,
                  t_vehicle,
                  t_rank),
    _tw_sol(tw_sol) {
}

bool vrptw_exchange::is_valid() const {
  return cvrp_exchange::is_valid() and
         _tw_sol[t_vehicle].is_valid_addition_for_tw(s_route.begin() + s_rank,
                                                     s_route.begin() + s_rank +
                                                       1,
                                                     t_rank,
                                                     t_rank + 1) and
         _tw_sol[s_vehicle].is_valid_addition_for_tw(t_route.begin() + t_rank,
                                                     t_route.begin() + t_rank +
                                                       1,
                                                     s_rank,
                                                     s_rank + 1);
}

void vrptw_exchange::apply() const {
  auto s_job_rank = s_route[s_rank];
  auto t_job_rank = t_route[t_rank];

  _tw_sol[s_vehicle].remove(s_rank, 1);
  _tw_sol[t_vehicle].remove(t_rank, 1);
  _tw_sol[s_vehicle].add(t_job_rank, s_rank);
  _tw_sol[t_vehicle].add(s_job_rank, t_rank);
}
