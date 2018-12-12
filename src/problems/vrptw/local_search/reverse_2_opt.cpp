/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <iostream>

#include "problems/vrptw/local_search/reverse_2_opt.h"

vrptw_reverse_two_opt::vrptw_reverse_two_opt(const input& input,
                                             const solution_state& sol_state,
                                             tw_route& tw_s_route,
                                             index_t s_vehicle,
                                             index_t s_rank,
                                             tw_route& tw_t_route,
                                             index_t t_vehicle,
                                             index_t t_rank)
  : cvrp_reverse_two_opt(input,
                         sol_state,
                         static_cast<raw_route&>(tw_s_route),
                         s_vehicle,
                         s_rank,
                         static_cast<raw_route&>(tw_t_route),
                         t_vehicle,
                         t_rank),
    _tw_s_route(tw_s_route),
    _tw_t_route(tw_t_route) {
}

bool vrptw_reverse_two_opt::is_valid() {
  return cvrp_reverse_two_opt::is_valid() and
         _tw_t_route.is_valid_addition_for_tw(_input,
                                              s_route.rbegin(),
                                              s_route.rbegin() +
                                                s_route.size() - 1 - s_rank,
                                              0,
                                              t_rank + 1) and
         _tw_s_route.is_valid_addition_for_tw(_input,
                                              t_route.rbegin() +
                                                t_route.size() - 1 - t_rank,
                                              t_route.rend(),
                                              s_rank + 1,
                                              s_route.size());
}

void vrptw_reverse_two_opt::apply() {
  std::vector<index_t> t_job_ranks;
  t_job_ranks.insert(t_job_ranks.begin(),
                     t_route.rbegin() + t_route.size() - 1 - t_rank,
                     t_route.rend());

  _tw_t_route.replace(_input,
                      s_route.rbegin(),
                      s_route.rbegin() + s_route.size() - 1 - s_rank,
                      0,
                      t_rank + 1);

  _tw_s_route.replace(_input,
                      t_job_ranks.begin(),
                      t_job_ranks.end(),
                      s_rank + 1,
                      s_route.size());
}
