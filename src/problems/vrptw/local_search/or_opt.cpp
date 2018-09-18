/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/vrptw/local_search/or_opt.h"

vrptw_or_opt::vrptw_or_opt(const input& input,
                           const solution_state& sol_state,
                           tw_solution& tw_sol,
                           index_t s_vehicle,
                           index_t s_rank,
                           index_t t_vehicle,
                           index_t t_rank)
  : cvrp_or_opt(input,
                sol_state,
                tw_sol[s_vehicle].route,
                s_vehicle,
                s_rank,
                tw_sol[t_vehicle].route,
                t_vehicle,
                t_rank),
    _tw_sol(tw_sol),
    _is_normal_valid(false),
    _is_reverse_valid(false) {
}

gain_t vrptw_or_opt::gain() {
  ls_operator::gain();
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

  return stored_gain;
}

bool vrptw_or_opt::is_valid() {
  bool valid = cvrp_or_opt::is_valid();

  if (valid) {
    // Keep edge direction.
    auto s_start = s_route.begin() + s_rank;
    _is_normal_valid = _tw_sol[t_vehicle].is_valid_addition_for_tw(s_start,
                                                                   s_start + 2,
                                                                   t_rank,
                                                                   t_rank);
    // Reverse edge direction.
    auto s_reverse_start = s_route.rbegin() + s_route.size() - 2 - s_rank;
    _is_reverse_valid =
      _tw_sol[t_vehicle].is_valid_addition_for_tw(s_reverse_start,
                                                  s_reverse_start + 2,
                                                  t_rank,
                                                  t_rank);
  }

  return valid and (_is_normal_valid or _is_reverse_valid);
}

void vrptw_or_opt::apply() const {
  if (reverse_s_edge) {
    auto s_reverse_start = s_route.rbegin() + s_route.size() - 2 - s_rank;
    _tw_sol[t_vehicle].replace(s_reverse_start,
                               s_reverse_start + 2,
                               t_rank,
                               t_rank);
    _tw_sol[s_vehicle].remove(s_rank, 2);
  } else {
    auto s_start = s_route.begin() + s_rank;
    _tw_sol[t_vehicle].replace(s_start, s_start + 2, t_rank, t_rank);
    _tw_sol[s_vehicle].remove(s_rank, 2);
  }
}
