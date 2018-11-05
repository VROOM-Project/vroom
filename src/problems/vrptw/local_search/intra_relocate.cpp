/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/vrptw/local_search/intra_relocate.h"

vrptw_intra_relocate::vrptw_intra_relocate(const input& input,
                                           const solution_state& sol_state,
                                           tw_solution& tw_sol,
                                           index_t s_vehicle,
                                           index_t s_rank,
                                           index_t t_rank)
  : cvrp_intra_relocate(input,
                        sol_state,
                        tw_sol[s_vehicle].route,
                        s_vehicle,
                        s_rank,
                        t_rank),
    _tw_sol(tw_sol),
    _moved_jobs((s_rank < t_rank) ? t_rank - s_rank + 1 : s_rank - t_rank + 1),
    _first_rank(std::min(s_rank, t_rank)),
    _last_rank(std::max(s_rank, t_rank) + 1) {
  if (t_rank < s_rank) {
    _moved_jobs[0] = s_route[s_rank];
    std::copy(s_route.begin() + t_rank,
              s_route.begin() + s_rank,
              _moved_jobs.begin() + 1);
  } else {
    std::copy(s_route.begin() + s_rank + 1,
              s_route.begin() + t_rank + 1,
              _moved_jobs.begin());
    _moved_jobs.back() = s_route[s_rank];
  }
}

bool vrptw_intra_relocate::is_valid() {
  return _tw_sol[s_vehicle].is_valid_addition_for_tw(_input,
                                                     _moved_jobs.begin(),
                                                     _moved_jobs.end(),
                                                     _first_rank,
                                                     _last_rank);
}

void vrptw_intra_relocate::apply() {
  _tw_sol[s_vehicle].replace(_input,
                             _moved_jobs.begin(),
                             _moved_jobs.end(),
                             _first_rank,
                             _last_rank);
}

std::vector<index_t> vrptw_intra_relocate::addition_candidates() const {
  return {s_vehicle};
}
