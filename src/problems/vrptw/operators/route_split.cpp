/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/vrptw/operators/route_split.h"

namespace vroom::vrptw {

RouteSplit::RouteSplit(const Input& input,
                       const utils::SolutionState& sol_state,
                       TWRoute& tw_s_route,
                       Index s_vehicle,
                       const std::vector<Index>& empty_route_ranks,
                       std::vector<TWRoute>& sol,
                       const Eval& best_known_gain)
  : cvrp::RouteSplit(input,
                     sol_state,
                     static_cast<RawRoute&>(tw_s_route),
                     s_vehicle,
                     empty_route_ranks,
                     dummy_sol,
                     best_known_gain),
    _tw_s_route(tw_s_route),
    _tw_sol(sol) {
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

    // Ranks in choice are relative to _empty_route_ranks so we go
    // back to initial vehicle ranks in _sol.
    _begin_route_rank = _empty_route_ranks[choice.v_begin];
    _end_route_rank = _empty_route_ranks[choice.v_end];

    if (has_soft_skills() && has_preference_context()) {
      // Net delta across the two resulting routes: each task moves
      // from s_vehicle to its split vehicle, evaluated against the
      // frozen (still empty) content of the target route. Shipments
      // count once through their pickups. Same pattern as
      // cvrp::RouteSplit::compute_gain, but the base class _sol is a
      // dummy for this instantiation, so destinations are read from
      // _tw_sol (TW-aware insertion evals).
      int delta = 0;
      for (Index r = 0; r < s_route.size(); ++r) {
        const auto to_vehicle =
          (r < choice.split_rank) ? _begin_route_rank : _end_route_rank;
        // Destination rank is irrelevant on the frozen (empty) split
        // route, hence 0.
        delta += task_move_delta(s_route[r], s_vehicle, to_vehicle,
                                 _tw_sol[to_vehicle], 0);
      }
      preference_delta = delta;
    }
  }
  gain_computed = true;
}

void RouteSplit::apply() {
  assert(choice.gain != NO_GAIN);

  // Empty route holding the end of the split.
  auto& end_route = _tw_sol[_end_route_rank];
  assert(end_route.empty());

  const auto end_delivery =
    _tw_s_route.delivery_in_range(choice.split_rank, _tw_s_route.size());

  end_route.replace(_input,
                    end_delivery,
                    s_route.begin() + choice.split_rank,
                    s_route.end(),
                    0,
                    0);
  assert(end_route.max_load() ==
         _tw_s_route.sub_route_max_load_after(choice.split_rank));

  // Empty route holding the beginning of the split.
  auto& begin_route = _tw_sol[_begin_route_rank];
  assert(begin_route.empty());

  const auto begin_delivery =
    _tw_s_route.delivery_in_range(0, choice.split_rank);

  begin_route.replace(_input,
                      begin_delivery,
                      s_route.begin(),
                      s_route.begin() + choice.split_rank,
                      0,
                      0);
  assert(begin_route.max_load() ==
         _tw_s_route.sub_route_max_load_before(choice.split_rank));

  _tw_s_route.remove(_input, 0, s_route.size());
}

} // namespace vroom::vrptw
