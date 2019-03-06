/*

This file is part of VROOM.

Copyright (c) 2015-2019, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/vrptw/operators/cross_exchange.h"

namespace vroom {
namespace vrptw {

CrossExchange::CrossExchange(const Input& input,
                             const utils::SolutionState& sol_state,
                             TWRoute& tw_s_route,
                             Index s_vehicle,
                             Index s_rank,
                             TWRoute& tw_t_route,
                             Index t_vehicle,
                             Index t_rank)
  : cvrp::CrossExchange(input,
                        sol_state,
                        static_cast<RawRoute&>(tw_s_route),
                        s_vehicle,
                        s_rank,
                        static_cast<RawRoute&>(tw_t_route),
                        t_vehicle,
                        t_rank),
    _tw_s_route(tw_s_route),
    _tw_t_route(tw_t_route),
    _s_is_normal_valid(false),
    _s_is_reverse_valid(false),
    _t_is_normal_valid(false),
    _t_is_reverse_valid(false) {
}

void CrossExchange::compute_gain() {
  cvrp::CrossExchange::compute_gain();
  assert(_s_is_normal_valid or _s_is_reverse_valid);
  assert(_t_is_normal_valid or _t_is_reverse_valid);

  Gain s_gain;
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

  Gain t_gain;
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

  stored_gain = s_gain + t_gain;
}

bool CrossExchange::is_valid() {
  bool valid = cvrp::CrossExchange::is_valid();

  if (valid) {
    // Keep target edge direction when inserting in source route.
    auto t_start = t_route.begin() + t_rank;
    _s_is_normal_valid = _tw_s_route.is_valid_addition_for_tw(_input,
                                                              t_start,
                                                              t_start + 2,
                                                              s_rank,
                                                              s_rank + 2);
    // Reverse target edge direction when inserting in source route.
    auto t_reverse_start = t_route.rbegin() + t_route.size() - 2 - t_rank;
    _s_is_reverse_valid =
      _tw_s_route.is_valid_addition_for_tw(_input,
                                           t_reverse_start,
                                           t_reverse_start + 2,
                                           s_rank,
                                           s_rank + 2);
    valid = _s_is_normal_valid or _s_is_reverse_valid;
  }

  if (valid) {
    // Keep source edge direction when inserting in target route.
    auto s_start = s_route.begin() + s_rank;
    _t_is_normal_valid = _tw_t_route.is_valid_addition_for_tw(_input,
                                                              s_start,
                                                              s_start + 2,
                                                              t_rank,
                                                              t_rank + 2);
    // Reverse source edge direction when inserting in target route.
    auto s_reverse_start = s_route.rbegin() + s_route.size() - 2 - s_rank;
    _t_is_reverse_valid =
      _tw_t_route.is_valid_addition_for_tw(_input,
                                           s_reverse_start,
                                           s_reverse_start + 2,
                                           t_rank,
                                           t_rank + 2);
    valid = _t_is_normal_valid or _t_is_reverse_valid;
  }

  return valid;
}

void CrossExchange::apply() {
  std::vector<Index> t_job_ranks;
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
    _tw_t_route.replace(_input,
                        s_route.begin() + s_rank,
                        s_route.begin() + s_rank + 2,
                        t_rank,
                        t_rank + 2);
  } else {
    auto s_reverse_start = s_route.rbegin() + s_route.size() - 2 - s_rank;
    _tw_t_route.replace(_input,
                        s_reverse_start,
                        s_reverse_start + 2,
                        t_rank,
                        t_rank + 2);
  }

  _tw_s_route.replace(_input,
                      t_job_ranks.begin(),
                      t_job_ranks.end(),
                      s_rank,
                      s_rank + 2);
}

} // namespace vrptw
} // namespace vroom
