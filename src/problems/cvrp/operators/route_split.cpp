/*

This file is part of VROOM.

Copyright (c) 2015-2022, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/cvrp/operators/route_split.h"

namespace vroom {
namespace cvrp {

std::vector<std::reference_wrapper<RawRoute>> RouteSplit::dummy_route_refs;

RouteSplit::RouteSplit(
  const Input& input,
  const utils::SolutionState& sol_state,
  RawRoute& s_route,
  Index s_vehicle,
  const std::vector<Index>& empty_route_ranks,
  const std::vector<std::reference_wrapper<RawRoute>>& empty_route_refs,
  const Eval& best_known_gain)
  // Use dummy 0 values for unused ranks.
  : Operator(OperatorName::RouteSplit,
             input,
             sol_state,
             s_route,
             s_vehicle,
             0,
             s_route,
             s_vehicle,
             0),
    _best_known_gain(best_known_gain),
    _empty_route_ranks(empty_route_ranks),
    _empty_route_refs(empty_route_refs),
    choice(ls::empty_route_split_choice) {
  assert(s_route.size() >= 2);
  assert(_empty_route_ranks.size() >= 2);
}

void RouteSplit::compute_gain() {
  choice = ls::compute_best_route_split_choice(_input,
                                               _sol_state,
                                               s_vehicle,
                                               source,
                                               _empty_route_ranks,
                                               _best_known_gain);
  if (choice.gain.cost > 0) {
    stored_gain = choice.gain;
  }
  gain_computed = true;
}

bool RouteSplit::is_valid() {
  // Not supposed to be used.
  assert(false);
  return true;
}

void RouteSplit::apply() {
  assert(choice.gain != NO_GAIN);

  // Empty route holding the end of the split.
  auto& end_route = _empty_route_refs[choice.v_end].get();
  assert(end_route.empty());

  std::move(s_route.begin() + choice.split_rank,
            s_route.end(),
            std::back_inserter(end_route.route));

  // Empty route holding the beginning of the split.
  auto& begin_route = _empty_route_refs[choice.v_begin].get();
  assert(begin_route.empty());

  std::move(s_route.begin(),
            s_route.begin() + choice.split_rank,
            std::back_inserter(begin_route.route));

  s_route.clear();

  source.update_amounts(_input);
  end_route.update_amounts(_input);
  begin_route.update_amounts(_input);
}

std::vector<Index> RouteSplit::addition_candidates() const {
  return {s_vehicle,
          _empty_route_ranks[choice.v_begin],
          _empty_route_ranks[choice.v_end]};
}

std::vector<Index> RouteSplit::update_candidates() const {
  return {s_vehicle,
          _empty_route_ranks[choice.v_begin],
          _empty_route_ranks[choice.v_end]};
}

bool RouteSplit::invalidated_by(Index rank) const {
  assert(choice.gain != NO_GAIN);
  return rank == _empty_route_ranks[choice.v_begin] or
         rank == _empty_route_ranks[choice.v_end];
}

} // namespace cvrp
} // namespace vroom
