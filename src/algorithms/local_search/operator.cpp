/*

This file is part of VROOM.

Copyright (c) 2015-2019, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "algorithms/local_search/operator.h"

namespace vroom {
namespace ls {

Operator::Operator(const Input& input,
                   const utils::SolutionState& sol_state,
                   RawRoute& s_raw_route,
                   Index s_vehicle,
                   Index s_rank,
                   RawRoute& t_raw_route,
                   Index t_vehicle,
                   Index t_rank)
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

Gain Operator::gain() {
  if (!gain_computed) {
    this->compute_gain();
  }
  return stored_gain;
}

} // namespace ls
} // namespace vroom
