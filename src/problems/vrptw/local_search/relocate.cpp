/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/vrptw/local_search/relocate.h"

vrptw_relocate::vrptw_relocate(const input& input,
                               const solution_state& sol_state,
                               tw_solution& tw_sol,
                               index_t s_vehicle,
                               index_t s_rank,
                               index_t t_vehicle,
                               index_t t_rank)
  : cvrp_relocate(input,
                  sol_state,
                  tw_sol[s_vehicle].route,
                  s_vehicle,
                  s_rank,
                  tw_sol[t_vehicle].route,
                  t_vehicle,
                  t_rank),
    _tw_sol(tw_sol) {
}

bool vrptw_relocate::is_valid() const {
  return cvrp_relocate::is_valid() and
         _tw_sol[t_vehicle].is_valid_addition_for_tw(_input,
                                                     s_route[s_rank],
                                                     t_rank);
}

void vrptw_relocate::apply() const {
  auto relocate_job_rank = s_route[s_rank];

  _tw_sol[s_vehicle].remove(_input, s_rank, 1);
  _tw_sol[t_vehicle].add(_input, relocate_job_rank, t_rank);
}
