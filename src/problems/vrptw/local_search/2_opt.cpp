/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/vrptw/local_search/2_opt.h"

vrptw_two_opt::vrptw_two_opt(const input& input,
                             const solution_state& sol_state,
                             tw_solution& tw_sol,
                             index_t s_vehicle,
                             index_t s_rank,
                             index_t t_vehicle,
                             index_t t_rank)
  : cvrp_two_opt(input,
                 sol_state,
                 tw_sol[s_vehicle].route,
                 s_vehicle,
                 s_rank,
                 tw_sol[t_vehicle].route,
                 t_vehicle,
                 t_rank),
    _tw_sol(tw_sol) {
}

bool vrptw_two_opt::is_valid() {
  return cvrp_two_opt::is_valid() and
         _tw_sol[t_vehicle].is_valid_addition_for_tw(_input,
                                                     s_route.begin() + s_rank +
                                                       1,
                                                     s_route.end(),
                                                     t_rank + 1,
                                                     t_route.size()) and
         _tw_sol[s_vehicle].is_valid_addition_for_tw(_input,
                                                     t_route.begin() + t_rank +
                                                       1,
                                                     t_route.end(),
                                                     s_rank + 1,
                                                     s_route.size());
}

void vrptw_two_opt::apply() {
  std::vector<index_t> t_job_ranks;
  t_job_ranks.insert(t_job_ranks.begin(),
                     t_route.begin() + t_rank + 1,
                     t_route.end());

  _tw_sol[t_vehicle].replace(_input,
                             s_route.begin() + s_rank + 1,
                             s_route.end(),
                             t_rank + 1,
                             t_route.size());
  _tw_sol[s_vehicle].replace(_input,
                             t_job_ranks.begin(),
                             t_job_ranks.end(),
                             s_rank + 1,
                             s_route.size());
}
