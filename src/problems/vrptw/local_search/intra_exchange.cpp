/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/vrptw/local_search/intra_exchange.h"

vrptw_intra_exchange::vrptw_intra_exchange(const input& input,
                                           const solution_state& sol_state,
                                           tw_solution& tw_sol,
                                           index_t s_vehicle,
                                           index_t s_rank,
                                           index_t t_rank)
  : cvrp_intra_exchange(input,
                        sol_state,
                        tw_sol[s_vehicle].route,
                        s_vehicle,
                        s_rank,
                        t_rank),
    _tw_sol(tw_sol),
    _moved_jobs(t_rank - s_rank + 1),
    _first_rank(s_rank),
    _last_rank(t_rank + 1) {
  std::copy(s_route.begin() + s_rank,
            s_route.begin() + t_rank + 1,
            _moved_jobs.begin());
  std::swap(_moved_jobs[0], _moved_jobs.back());
}

bool vrptw_intra_exchange::is_valid() {
  return _tw_sol[s_vehicle].is_valid_addition_for_tw(_input,
                                                     _moved_jobs.begin(),
                                                     _moved_jobs.end(),
                                                     _first_rank,
                                                     _last_rank);
}

void vrptw_intra_exchange::apply() {
  _tw_sol[s_vehicle].replace(_input,
                             _moved_jobs.begin(),
                             _moved_jobs.end(),
                             _first_rank,
                             _last_rank);
}
