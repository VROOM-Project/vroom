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
    _tw_sol(tw_sol) {
}

bool vrptw_or_opt::is_valid() {
  bool valid = cvrp_or_opt::is_valid();

  if (valid) {
    // Required for relevant reverse_s_edge, normal_stored_gain and
    // reversed_stored_gain values.
    compute_gain();

    if (reverse_s_edge) {
      // Biggest potential gain is obtained when reversing edge.
      auto s_reverse_start = s_route.rbegin() + s_route.size() - 2 - s_rank;

      if (!_tw_sol[t_vehicle].is_valid_addition_for_tw(s_reverse_start,
                                                       s_reverse_start + 2,
                                                       t_rank,
                                                       t_rank)) {
        // Edge direction with biggest potential gain raises an invalid
        // solution, trying the other direction.
        auto s_start = s_route.begin() + s_rank;
        valid &= _tw_sol[t_vehicle].is_valid_addition_for_tw(s_start,
                                                             s_start + 2,
                                                             t_rank,
                                                             t_rank);
        if (valid) {
          // Update gain and edge direction.
          stored_gain = normal_stored_gain;
          reverse_s_edge = false;
        }
      }
    } else {
      // Biggest potential gain is obtained when not reversing edge.
      auto s_start = s_route.begin() + s_rank;
      if (!_tw_sol[t_vehicle].is_valid_addition_for_tw(s_start,
                                                       s_start + 2,
                                                       t_rank,
                                                       t_rank)) {
        // Edge direction with biggest potential gain raises an invalid
        // solution, trying the other direction.
        auto s_reverse_start = s_route.rbegin() + s_route.size() - 2 - s_rank;
        valid &=
          _tw_sol[t_vehicle].is_valid_addition_for_tw(s_reverse_start,
                                                      s_reverse_start + 2,
                                                      t_rank,
                                                      t_rank);
        if (valid) {
          // Update gain and edge direction.
          stored_gain = reversed_stored_gain;
          reverse_s_edge = true;
        }
      }
    }
  }

  return valid;
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
