/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/vrptw/local_search/intra_cross_exchange.h"

vrptw_intra_cross_exchange::vrptw_intra_cross_exchange(
  const input& input,
  const solution_state& sol_state,
  tw_route& tw_s_route,
  index_t s_vehicle,
  index_t s_rank,
  index_t t_rank)
  : cvrp_intra_cross_exchange(input,
                              sol_state,
                              static_cast<raw_route&>(tw_s_route),
                              s_vehicle,
                              s_rank,
                              t_rank),
    _tw_s_route(tw_s_route),
    _s_normal_t_normal_is_valid(false),
    _s_normal_t_reverse_is_valid(false),
    _s_reverse_t_reverse_is_valid(false),
    _s_reverse_t_normal_is_valid(false),
    _moved_jobs(t_rank - s_rank + 2),
    _first_rank(s_rank),
    _last_rank(t_rank + 2) {
  _moved_jobs[0] = s_route[t_rank];
  _moved_jobs[1] = s_route[t_rank + 1];
  std::copy(s_route.begin() + s_rank + 2,
            s_route.begin() + t_rank,
            _moved_jobs.begin() + 2);
  _moved_jobs[_moved_jobs.size() - 2] = s_route[s_rank];
  _moved_jobs[_moved_jobs.size() - 1] = s_route[s_rank + 1];
}

void vrptw_intra_cross_exchange::compute_gain() {
  cvrp_intra_cross_exchange::compute_gain();
  assert(_s_normal_t_normal_is_valid or _s_normal_t_reverse_is_valid or
         _s_reverse_t_reverse_is_valid or _s_reverse_t_normal_is_valid);

  stored_gain = std::numeric_limits<gain_t>::min();

  if (_s_normal_t_normal_is_valid) {
    gain_t current_gain = normal_s_gain + normal_t_gain;
    if (current_gain > stored_gain) {
      stored_gain = current_gain;
      reverse_s_edge = false;
      reverse_t_edge = false;
    }
  }

  if (_s_normal_t_reverse_is_valid) {
    gain_t current_gain = reversed_s_gain + normal_t_gain;
    if (current_gain > stored_gain) {
      stored_gain = current_gain;
      reverse_s_edge = false;
      reverse_t_edge = true;
    }
  }

  if (_s_reverse_t_reverse_is_valid) {
    gain_t current_gain = reversed_s_gain + reversed_t_gain;
    if (current_gain > stored_gain) {
      stored_gain = current_gain;
      reverse_s_edge = true;
      reverse_t_edge = true;
    }
  }

  if (_s_reverse_t_normal_is_valid) {
    gain_t current_gain = normal_s_gain + reversed_t_gain;
    if (current_gain > stored_gain) {
      stored_gain = current_gain;
      reverse_s_edge = true;
      reverse_t_edge = false;
    }
  }
}

bool vrptw_intra_cross_exchange::is_valid() {
  _s_normal_t_normal_is_valid =
    _tw_s_route.is_valid_addition_for_tw(_input,
                                         _moved_jobs.begin(),
                                         _moved_jobs.end(),
                                         _first_rank,
                                         _last_rank);

  std::swap(_moved_jobs[0], _moved_jobs[1]);
  _s_normal_t_reverse_is_valid =
    _tw_s_route.is_valid_addition_for_tw(_input,
                                         _moved_jobs.begin(),
                                         _moved_jobs.end(),
                                         _first_rank,
                                         _last_rank);

  std::swap(_moved_jobs[_moved_jobs.size() - 2],
            _moved_jobs[_moved_jobs.size() - 1]);
  _s_reverse_t_reverse_is_valid =
    _tw_s_route.is_valid_addition_for_tw(_input,
                                         _moved_jobs.begin(),
                                         _moved_jobs.end(),
                                         _first_rank,
                                         _last_rank);

  std::swap(_moved_jobs[0], _moved_jobs[1]);
  _s_reverse_t_normal_is_valid =
    _tw_s_route.is_valid_addition_for_tw(_input,
                                         _moved_jobs.begin(),
                                         _moved_jobs.end(),
                                         _first_rank,
                                         _last_rank);

  // Reset to initial situation before potential application.
  std::swap(_moved_jobs[_moved_jobs.size() - 2],
            _moved_jobs[_moved_jobs.size() - 1]);

  return _s_normal_t_normal_is_valid or _s_normal_t_reverse_is_valid or
         _s_reverse_t_reverse_is_valid or _s_reverse_t_normal_is_valid;
}

void vrptw_intra_cross_exchange::apply() {
  if (reverse_t_edge) {
    std::swap(_moved_jobs[0], _moved_jobs[1]);
  }
  if (reverse_s_edge) {
    std::swap(_moved_jobs[_moved_jobs.size() - 2],
              _moved_jobs[_moved_jobs.size() - 1]);
  }

  _tw_s_route.replace(_input,
                      _moved_jobs.begin(),
                      _moved_jobs.end(),
                      _first_rank,
                      _last_rank);
}

std::vector<index_t> vrptw_intra_cross_exchange::addition_candidates() const {
  return {s_vehicle};
}
