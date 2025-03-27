/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/cvrp/operators/swap_star.h"

namespace vroom::cvrp {

SwapStar::SwapStar(const Input& input,
                   const utils::SolutionState& sol_state,
                   RawRoute& s_route,
                   Index s_vehicle,
                   RawRoute& t_route,
                   Index t_vehicle,
                   const Eval& best_known_gain)
  // Use dummy 0 values for unused ranks.
  : Operator(OperatorName::SwapStar,
             input,
             sol_state,
             s_route,
             s_vehicle,
             0,
             t_route,
             t_vehicle,
             0),
    _best_known_gain(best_known_gain) {
  assert(s_vehicle != t_vehicle);
  assert(!s_route.empty());
  assert(!t_route.empty());
  assert(_input.vehicle_ok_with_vehicle(s_vehicle, t_vehicle));
}

void SwapStar::compute_gain() {
  choice = ls::compute_best_swap_star_choice(_input,
                                             _sol_state,
                                             s_vehicle,
                                             source,
                                             t_vehicle,
                                             target,
                                             _best_known_gain);
  if (choice.gain.cost > 0) {
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
  const auto s_value = s_route[choice.s_rank];
  const auto t_value = t_route[choice.t_rank];

  if (choice.s_rank == choice.insertion_in_source) {
    s_route[choice.s_rank] = t_value;
  } else {
    if (choice.s_rank < choice.insertion_in_source) {
      std::copy(s_route.begin() + choice.s_rank + 1,
                s_route.begin() + choice.insertion_in_source,
                s_route.begin() + choice.s_rank);
      s_route[choice.insertion_in_source - 1] = t_value;
    } else {
      std::copy(s_route.rend() - choice.s_rank,
                s_route.rend() - choice.insertion_in_source,
                s_route.rend() - choice.s_rank - 1);
      s_route[choice.insertion_in_source] = t_value;
    }
  }

  if (choice.t_rank == choice.insertion_in_target) {
    t_route[choice.t_rank] = s_value;
  } else {
    if (choice.t_rank < choice.insertion_in_target) {
      std::copy(t_route.begin() + choice.t_rank + 1,
                t_route.begin() + choice.insertion_in_target,
                t_route.begin() + choice.t_rank);
      t_route[choice.insertion_in_target - 1] = s_value;
    } else {
      std::copy(t_route.rend() - choice.t_rank,
                t_route.rend() - choice.insertion_in_target,
                t_route.rend() - choice.t_rank - 1);
      t_route[choice.insertion_in_target] = s_value;
    }
  }

  source.update_amounts(_input);
  target.update_amounts(_input);
}

std::vector<Index> SwapStar::addition_candidates() const {
  return {s_vehicle, t_vehicle};
}

std::vector<Index> SwapStar::update_candidates() const {
  return {s_vehicle, t_vehicle};
}

} // namespace vroom::cvrp
