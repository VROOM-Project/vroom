/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/vrptw/local_search/relocate.h"

vrptw_relocate::vrptw_relocate(const input& input,
                               tw_solution& tw_sol,
                               raw_solution& sol,
                               const solution_state& sol_state,
                               index_t source_vehicle,
                               index_t source_rank,
                               index_t target_vehicle,
                               index_t target_rank)
  : cvrp_relocate(input,
                  sol,
                  sol_state,
                  source_vehicle,
                  source_rank,
                  target_vehicle,
                  target_rank),
    _tw_sol(tw_sol) {
}

bool vrptw_relocate::is_valid() const {
  return cvrp_relocate::is_valid() and
         _tw_sol[target_vehicle]
           .is_valid_addition_for_tw(_sol[source_vehicle][source_rank],
                                     target_rank);
}

void vrptw_relocate::apply() const {
  // cvrp_relocate::apply();  // TODO adjust _tw_sol
}
