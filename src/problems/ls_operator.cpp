/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/ls_operator.h"

ls_operator::ls_operator(const input& input,
                         const solution_state& sol_state,
                         raw_route& s_raw_route,
                         index_t s_vehicle,
                         index_t s_rank,
                         raw_route& t_raw_route,
                         index_t t_vehicle,
                         index_t t_rank)
  : _input(input),
    _sol_state(sol_state),
    s_route(s_raw_route.route),
    s_vehicle(s_vehicle),
    s_rank(s_rank),
    t_route(t_raw_route.route),
    t_vehicle(t_vehicle),
    t_rank(t_rank),
    gain_computed(false) {
}

gain_t ls_operator::gain() {
  if (!gain_computed) {
    this->compute_gain();
  }
  return stored_gain;
}
