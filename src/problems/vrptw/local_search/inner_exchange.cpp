/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/vrptw/local_search/inner_exchange.h"

vrptw_inner_exchange::vrptw_inner_exchange(const input& input,
                                           const solution_state& sol_state,
                                           tw_solution& tw_sol,
                                           index_t s_vehicle,
                                           index_t s_rank,
                                           index_t t_rank)
  : cvrp_inner_exchange(input,
                        sol_state,
                        tw_sol[s_vehicle].route,
                        s_vehicle,
                        s_rank,
                        t_rank),
    _tw_sol(tw_sol) {
}

bool vrptw_inner_exchange::is_valid() const {
  std::vector<index_t> job_ranks(t_rank - s_rank + 1);

  std::copy(s_route.begin() + s_rank,
            s_route.begin() + t_rank + 1,
            job_ranks.begin());
  std::swap(job_ranks[0], job_ranks.back());

  return _tw_sol[s_vehicle].is_valid_addition_for_tw(_input,
                                                     job_ranks.begin(),
                                                     job_ranks.end(),
                                                     s_rank,
                                                     t_rank + 1);
}

void vrptw_inner_exchange::apply() const {
  index_t s_job_rank = s_route[s_rank];
  std::vector<index_t> t_job_ranks(1, s_route[t_rank]);

  _tw_sol[s_vehicle].remove(_input, t_rank, 1);

  _tw_sol[s_vehicle].replace(_input,
                             t_job_ranks.begin(),
                             t_job_ranks.end(),
                             s_rank,
                             s_rank + 1);

  _tw_sol[s_vehicle].add(_input, s_job_rank, t_rank);
}