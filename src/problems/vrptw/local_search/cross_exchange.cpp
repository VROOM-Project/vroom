/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/vrptw/local_search/cross_exchange.h"

vrptw_cross_exchange::vrptw_cross_exchange(const input& input,
                                           const solution_state& sol_state,
                                           tw_solution& tw_sol,
                                           index_t s_vehicle,
                                           index_t s_rank,
                                           index_t t_vehicle,
                                           index_t t_rank)
  : cvrp_cross_exchange(input,
                        sol_state,
                        tw_sol[s_vehicle].route,
                        s_vehicle,
                        s_rank,
                        tw_sol[t_vehicle].route,
                        t_vehicle,
                        t_rank),
    _tw_sol(tw_sol),
    _s_is_normal_valid(false),
    _s_is_reverse_valid(false),
    _t_is_normal_valid(false),
    _t_is_reverse_valid(false) {
}

gain_t vrptw_cross_exchange::gain() {
  ls_operator::gain();
  assert(_s_is_normal_valid or _s_is_reverse_valid);
  assert(_t_is_normal_valid or _t_is_reverse_valid);

  gain_t s_gain;
  if (reverse_t_edge) {
    s_gain = reversed_s_gain;
    if (!_s_is_reverse_valid) {
      // Biggest potential gain is obtained when reversing edge, but
      // this does not match TW constraints, so update gain and edge
      // direction to not reverse.
      s_gain = normal_s_gain;
      reverse_t_edge = false;
    }
  } else {
    s_gain = normal_s_gain;
    if (!_s_is_normal_valid) {
      // Biggest potential gain is obtained when not reversing edge,
      // but this does not match TW constraints, so update gain and
      // edge direction to reverse.
      s_gain = reversed_s_gain;
      reverse_t_edge = true;
    }
  }

  gain_t t_gain;
  if (reverse_s_edge) {
    t_gain = reversed_t_gain;
    if (!_t_is_reverse_valid) {
      // Biggest potential gain is obtained when reversing edge, but
      // this does not match TW constraints, so update gain and edge
      // direction to not reverse.
      t_gain = normal_t_gain;
      reverse_s_edge = false;
    }
  } else {
    t_gain = normal_t_gain;
    if (!_t_is_normal_valid) {
      // Biggest potential gain is obtained when not reversing edge,
      // but this does not match TW constraints, so update gain and
      // edge direction to reverse.
      t_gain = reversed_t_gain;
      reverse_s_edge = true;
    }
  }

  return s_gain + t_gain;
}

bool vrptw_cross_exchange::is_valid() {
  bool valid = cvrp_cross_exchange::is_valid();

  if (valid) {
    // Keep target edge direction when inserting in source route.
    auto t_start = t_route.begin() + t_rank;
    _s_is_normal_valid =
      _tw_sol[s_vehicle].is_valid_addition_for_tw(t_start,
                                                  t_start + 2,
                                                  s_rank,
                                                  s_rank + 2);
    // Reverse target edge direction when inserting in source route.
    auto t_reverse_start = t_route.rbegin() + t_route.size() - 2 - t_rank;
    _s_is_reverse_valid =
      _tw_sol[s_vehicle].is_valid_addition_for_tw(t_reverse_start,
                                                  t_reverse_start + 2,
                                                  s_rank,
                                                  s_rank + 2);
    valid = _s_is_normal_valid or _s_is_reverse_valid;
  }

  if (valid) {
    // Keep source edge direction when inserting in target route.
    auto s_start = s_route.begin() + s_rank;
    _t_is_normal_valid =
      _tw_sol[t_vehicle].is_valid_addition_for_tw(s_start,
                                                  s_start + 2,
                                                  t_rank,
                                                  t_rank + 2);
    // Reverse source edge direction when inserting in target route..
    auto s_reverse_start = s_route.rbegin() + s_route.size() - 2 - s_rank;
    _t_is_reverse_valid =
      _tw_sol[t_vehicle].is_valid_addition_for_tw(s_reverse_start,
                                                  s_reverse_start + 2,
                                                  t_rank,
                                                  t_rank + 2);
    valid = _t_is_normal_valid or _t_is_reverse_valid;
  }

  return valid;
}

void vrptw_cross_exchange::apply() const {
  std::vector<index_t> t_job_ranks;
  if (!reverse_t_edge) {
    auto t_start = t_route.begin() + t_rank;
    t_job_ranks.insert(t_job_ranks.begin(), t_start, t_start + 2);
  } else {
    auto t_reverse_start = t_route.rbegin() + t_route.size() - 2 - t_rank;
    t_job_ranks.insert(t_job_ranks.begin(),
                       t_reverse_start,
                       t_reverse_start + 2);
  }

  if (!reverse_s_edge) {
    _tw_sol[t_vehicle].replace(s_route.begin() + s_rank,
                               s_route.begin() + s_rank + 2,
                               t_rank,
                               t_rank + 2);
  } else {
    auto s_reverse_start = s_route.rbegin() + s_route.size() - 2 - s_rank;
    _tw_sol[t_vehicle].replace(s_reverse_start,
                               s_reverse_start + 2,
                               t_rank,
                               t_rank + 2);
  }

  _tw_sol[s_vehicle].replace(t_job_ranks.begin(),
                             t_job_ranks.end(),
                             s_rank,
                             s_rank + 2);
}
