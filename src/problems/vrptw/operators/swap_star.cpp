/*

This file is part of VROOM.

Copyright (c) 2015-2021, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/vrptw/operators/swap_star.h"

namespace vroom {
namespace vrptw {

SwapStar::SwapStar(const Input& input,
                   const utils::SolutionState& sol_state,
                   TWRoute& tw_s_route,
                   Index s_vehicle,
                   TWRoute& tw_t_route,
                   Index t_vehicle,
                   Gain best_known_gain)
  : cvrp::SwapStar(input,
                   sol_state,
                   static_cast<RawRoute&>(tw_s_route),
                   s_vehicle,
                   static_cast<RawRoute&>(tw_t_route),
                   t_vehicle,
                   best_known_gain),
    _tw_s_route(tw_s_route),
    _tw_t_route(tw_t_route) {
}

void SwapStar::compute_gain() {
  // Similar to cvrp::SwapStar::compute_gain but makes sure to trigger
  // ls::compute_best_swap_star_choice<TWRoute>.
  choice = ls::compute_best_swap_star_choice(_input,
                                             _sol_state,
                                             _tw_s_route,
                                             _tw_t_route,
                                             _best_known_gain);
  if (choice.gain > 0) {
    stored_gain = choice.gain;
  }
  gain_computed = true;
}

void SwapStar::apply() {
  const auto s_insert = ls::get_insert_range(s_route,
                                             choice.s_rank,
                                             t_route[choice.t_rank],
                                             choice.insertion_in_source);

  const auto t_insert = ls::get_insert_range(t_route,
                                             choice.t_rank,
                                             s_route[choice.s_rank],
                                             choice.insertion_in_target);

  _tw_s_route.replace(_input,
                      s_insert.range.begin(),
                      s_insert.range.end(),
                      s_insert.first_rank,
                      s_insert.last_rank);

  _tw_t_route.replace(_input,
                      t_insert.range.begin(),
                      t_insert.range.end(),
                      t_insert.first_rank,
                      t_insert.last_rank);
}

} // namespace vrptw
} // namespace vroom
