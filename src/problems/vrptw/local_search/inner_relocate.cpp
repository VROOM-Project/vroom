/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/vrptw/local_search/inner_relocate.h"

vrptw_inner_relocate::vrptw_inner_relocate(const input& input,
                                           const solution_state& sol_state,
                                           tw_solution& tw_sol,
                                           index_t s_vehicle,
                                           index_t s_rank,
                                           index_t t_rank)
  : cvrp_inner_relocate(input,
                        sol_state,
                        tw_sol[s_vehicle].route,
                        s_vehicle,
                        s_rank,
                        t_rank),
    _tw_sol(tw_sol),
    _rank_distance((s_rank < t_rank) ? t_rank - s_rank : s_rank - t_rank) {
}

bool vrptw_inner_relocate::is_valid() {
  std::vector<index_t> job_ranks(_rank_distance + 1);
  index_t first_rank;
  index_t last_rank;

  if (t_rank < s_rank) {
    job_ranks[0] = s_route[s_rank];
    std::copy(s_route.begin() + t_rank,
              s_route.begin() + s_rank,
              job_ranks.begin() + 1);

    first_rank = t_rank;
    last_rank = s_rank + 1;
  } else {
    std::copy(s_route.begin() + s_rank + 1,
              s_route.begin() + t_rank + 1,
              job_ranks.begin());
    job_ranks.back() = s_route[s_rank];

    first_rank = s_rank;
    last_rank = t_rank + 1;
  }

  return _tw_sol[s_vehicle].is_valid_addition_for_tw(_input,
                                                     job_ranks.begin(),
                                                     job_ranks.end(),
                                                     first_rank,
                                                     last_rank) and
         _tw_sol[s_vehicle].is_valid_removal(_input, s_rank, 1);
}

void vrptw_inner_relocate::apply() {
  auto relocate_job_rank = s_route[s_rank];

  _tw_sol[s_vehicle].remove(_input, s_rank, 1);
  _tw_sol[s_vehicle].add(_input, relocate_job_rank, t_rank);
}
