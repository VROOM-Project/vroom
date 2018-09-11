/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/cvrp/local_search/operator.h"

cvrp_ls_operator::cvrp_ls_operator(const input& input,
                                   raw_solution& sol,
                                   const solution_state& sol_state,
                                   index_t source_vehicle,
                                   index_t source_rank,
                                   index_t target_vehicle,
                                   index_t target_rank)
  : _input(input),
    _sol(sol),
    _sol_state(sol_state),
    source_vehicle(source_vehicle),
    source_rank(source_rank),
    target_vehicle(target_vehicle),
    target_rank(target_rank),
    gain_computed(false) {
}

gain_t cvrp_ls_operator::gain() {
  if (!gain_computed) {
    this->compute_gain();
  }
  return stored_gain;
}
