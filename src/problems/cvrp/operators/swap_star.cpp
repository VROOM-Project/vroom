/*

This file is part of VROOM.

Copyright (c) 2015-2021, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/cvrp/operators/swap_star.h"

namespace vroom {
namespace cvrp {

SwapStar::SwapStar(const Input& input,
                   const utils::SolutionState& sol_state,
                   RawRoute& s_route,
                   Index s_vehicle,
                   RawRoute& t_route,
                   Index t_vehicle)
  // Use dummy 0 values for unused ranks.
  : Operator(input, sol_state, s_route, s_vehicle, 0, t_route, t_vehicle, 0) {
  assert(s_vehicle != t_vehicle);
  assert(s_route.size() >= 1);
  assert(t_route.size() >= 1);
  assert(_input.vehicle_ok_with_vehicle(s_vehicle, t_vehicle));
}

void SwapStar::compute_gain() {
  choice =
    ls::compute_best_swap_star_choice(_input, _sol_state, source, target);
  if (choice.gain > 0) {
    stored_gain = choice.gain;
  }
  gain_computed = true;
}

bool SwapStar::is_valid() {
  // Not supposed to be used.
  assert(false);
  return true;
}

void SwapStar::apply() {
  // TODO
  assert(false);
}

std::vector<Index> SwapStar::addition_candidates() const {
  return {s_vehicle, t_vehicle};
}

std::vector<Index> SwapStar::update_candidates() const {
  return {s_vehicle, t_vehicle};
}

} // namespace cvrp
} // namespace vroom
