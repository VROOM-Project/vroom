/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/vrptw/operators/relocate.h"

vrptw_relocate::vrptw_relocate(const input& input,
                               const solution_state& sol_state,
                               tw_route& tw_s_route,
                               index_t s_vehicle,
                               index_t s_rank,
                               tw_route& tw_t_route,
                               index_t t_vehicle,
                               index_t t_rank)
  : cvrp_relocate(input,
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

bool vrptw_relocate::is_valid() {
  return cvrp_relocate::is_valid() and
         _tw_t_route.is_valid_addition_for_tw(_input,
                                              s_route[s_rank],
                                              t_rank) and
         _tw_s_route.is_valid_removal(_input, s_rank, 1);
}

void vrptw_relocate::apply() {
  auto relocate_job_rank = s_route[s_rank];

  _tw_s_route.remove(_input, s_rank, 1);
  _tw_t_route.add(_input, relocate_job_rank, t_rank);
}
