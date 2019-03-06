/*

This file is part of VROOM.

Copyright (c) 2015-2019, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/vrptw/operators/or_opt.h"

namespace vroom {
namespace vrptw {

OrOpt::OrOpt(const Input& input,
             const utils::SolutionState& sol_state,
             TWRoute& tw_s_route,
             Index s_vehicle,
             Index s_rank,
             TWRoute& tw_t_route,
             Index t_vehicle,
             Index t_rank)
  : cvrp::OrOpt(input,
                sol_state,
                static_cast<RawRoute&>(tw_s_route),
                s_vehicle,
                s_rank,
                static_cast<RawRoute&>(tw_t_route),
                t_vehicle,
                t_rank),
    _tw_s_route(tw_s_route),
    _tw_t_route(tw_t_route),
    _is_normal_valid(false),
    _is_reverse_valid(false) {
}

void OrOpt::compute_gain() {
  cvrp::OrOpt::compute_gain();
  assert(_is_normal_valid or _is_reverse_valid);

  if (reverse_s_edge) {
    if (!_is_reverse_valid) {
      // Biggest potential gain is obtained when reversing edge, but
      // this does not match TW constraints, so update gain and edge
      // direction to not reverse.
      stored_gain = normal_stored_gain;
      reverse_s_edge = false;
    }
  } else {
    if (!_is_normal_valid) {
      // Biggest potential gain is obtained when not reversing edge,
      // but this does not match TW constraints, so update gain and
      // edge direction to reverse.
      stored_gain = reversed_stored_gain;
      reverse_s_edge = true;
    }
  }
}

bool OrOpt::is_valid() {
  bool valid = cvrp::OrOpt::is_valid();

  if (valid and _tw_s_route.is_valid_removal(_input, s_rank, 2)) {
    // Keep edge direction.
    auto s_start = s_route.begin() + s_rank;
    _is_normal_valid = _tw_t_route.is_valid_addition_for_tw(_input,
                                                            s_start,
                                                            s_start + 2,
                                                            t_rank,
                                                            t_rank);
    // Reverse edge direction.
    auto s_reverse_start = s_route.rbegin() + s_route.size() - 2 - s_rank;
    _is_reverse_valid =
      _tw_t_route.is_valid_addition_for_tw(_input,
                                           s_reverse_start,
                                           s_reverse_start + 2,
                                           t_rank,
                                           t_rank);
  }

  return valid and (_is_normal_valid or _is_reverse_valid);
}

void OrOpt::apply() {
  if (reverse_s_edge) {
    auto s_reverse_start = s_route.rbegin() + s_route.size() - 2 - s_rank;
    _tw_t_route.replace(_input,
                        s_reverse_start,
                        s_reverse_start + 2,
                        t_rank,
                        t_rank);
    _tw_s_route.remove(_input, s_rank, 2);
  } else {
    auto s_start = s_route.begin() + s_rank;
    _tw_t_route.replace(_input, s_start, s_start + 2, t_rank, t_rank);
    _tw_s_route.remove(_input, s_rank, 2);
  }
}

} // namespace vrptw
} // namespace vroom
