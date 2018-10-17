/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/vrptw/local_search/inner_mixed_exchange.h"

vrptw_inner_mixed_exchange::vrptw_inner_mixed_exchange(
  const input& input,
  const solution_state& sol_state,
  tw_solution& tw_sol,
  index_t s_vehicle,
  index_t s_rank,
  index_t t_rank)
  : cvrp_inner_mixed_exchange(input,
                              sol_state,
                              tw_sol[s_vehicle].route,
                              s_vehicle,
                              s_rank,
                              t_rank),
    _tw_sol(tw_sol),
    _s_is_normal_valid(false),
    _s_is_reverse_valid(false),
    _moved_jobs((s_rank < t_rank) ? t_rank - s_rank + 2 : s_rank - t_rank + 1),
    _first_rank(std::min(s_rank, t_rank)),
    _last_rank((t_rank < s_rank) ? s_rank + 1 : t_rank + 2) {
  index_t s_node;
  if (t_rank < s_rank) {
    s_node = 0;
    _t_edge_first = _moved_jobs.size() - 2;
    _t_edge_last = _moved_jobs.size() - 1;

    std::copy(s_route.begin() + t_rank + 2,
              s_route.begin() + s_rank,
              _moved_jobs.begin() + 1);
  } else {
    _t_edge_first = 0;
    _t_edge_last = 1;
    s_node = _moved_jobs.size() - 1;

    std::copy(s_route.begin() + s_rank + 1,
              s_route.begin() + t_rank,
              _moved_jobs.begin() + 2);
  }

  _moved_jobs[s_node] = s_route[s_rank];
  _moved_jobs[_t_edge_first] = s_route[t_rank];
  _moved_jobs[_t_edge_last] = s_route[t_rank + 1];
}

void vrptw_inner_mixed_exchange::compute_gain() {
  cvrp_inner_mixed_exchange::compute_gain();
  assert(_s_is_normal_valid or _s_is_reverse_valid);

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

  stored_gain = s_gain + t_gain;
}

bool vrptw_inner_mixed_exchange::is_valid() {
  _s_is_normal_valid =
    _tw_sol[s_vehicle].is_valid_addition_for_tw(_input,
                                                _moved_jobs.begin(),
                                                _moved_jobs.end(),
                                                _first_rank,
                                                _last_rank);

  std::swap(_moved_jobs[_t_edge_first], _moved_jobs[_t_edge_last]);

  _s_is_reverse_valid =
    _tw_sol[s_vehicle].is_valid_addition_for_tw(_input,
                                                _moved_jobs.begin(),
                                                _moved_jobs.end(),
                                                _first_rank,
                                                _last_rank);

  // Reset to initial situation before potential application.
  std::swap(_moved_jobs[_t_edge_first], _moved_jobs[_t_edge_last]);

  return _s_is_normal_valid or _s_is_reverse_valid;
}

void vrptw_inner_mixed_exchange::apply() {
  if (reverse_t_edge) {
    std::swap(_moved_jobs[_t_edge_first], _moved_jobs[_t_edge_last]);
  }

  _tw_sol[s_vehicle].replace(_input,
                             _moved_jobs.begin(),
                             _moved_jobs.end(),
                             _first_rank,
                             _last_rank);
}
