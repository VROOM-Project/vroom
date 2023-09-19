/*

This file is part of VROOM.

Copyright (c) 2015-2022, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/vrptw/operators/route_split.h"

namespace vroom::vrptw {

RouteSplit::RouteSplit(
  const Input& input,
  const utils::SolutionState& sol_state,
  TWRoute& tw_s_route,
  Index s_vehicle,
  std::vector<Index>&& empty_route_ranks,
  std::vector<std::reference_wrapper<TWRoute>>&& empty_route_refs,
  const Eval& best_known_gain)
  : cvrp::RouteSplit(input,
                     sol_state,
                     static_cast<RawRoute&>(tw_s_route),
                     s_vehicle,
                     std::move(empty_route_ranks),
                     std::vector<std::reference_wrapper<RawRoute>>(),
                     best_known_gain),
    _tw_s_route(tw_s_route),
    _empty_tw_route_refs(empty_route_refs) {
}

void RouteSplit::compute_gain() {
  // Similar to cvrp::RouteSplit::compute_gain but makes sure to
  // trigger ls::compute_best_route_split_choice<TWRoute>.
  choice = ls::compute_best_route_split_choice(_input,
                                               _sol_state,
                                               s_vehicle,
                                               _tw_s_route,
                                               _empty_route_ranks,
                                               _best_known_gain);
  if (choice.gain.cost > 0) {
    stored_gain = choice.gain;
  }
  gain_computed = true;
}

void RouteSplit::apply() {
  assert(choice.gain != NO_GAIN);

  // Empty route holding the end of the split.
  auto& end_route = _empty_tw_route_refs[choice.v_end].get();
  assert(end_route.empty());
  assert(end_route.vehicle_rank == _empty_route_ranks[choice.v_end]);

  const auto end_delivery =
    _tw_s_route.delivery_in_range(choice.split_rank, _tw_s_route.size());

  end_route.replace(_input,
                    end_delivery,
                    s_route.begin() + choice.split_rank,
                    s_route.end(),
                    0,
                    0);

  // Empty route holding the beginning of the split.
  auto& begin_route = _empty_tw_route_refs[choice.v_begin].get();
  assert(begin_route.empty());
  assert(begin_route.vehicle_rank == _empty_route_ranks[choice.v_begin]);

  const auto begin_delivery =
    _tw_s_route.delivery_in_range(0, choice.split_rank);

  begin_route.replace(_input,
                      begin_delivery,
                      s_route.begin(),
                      s_route.begin() + choice.split_rank,
                      0,
                      0);

  _tw_s_route.remove(_input, 0, s_route.size());
}

} // namespace vroom::vrptw
