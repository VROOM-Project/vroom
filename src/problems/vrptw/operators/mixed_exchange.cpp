/*

This file is part of VROOM.

Copyright (c) 2015-2019, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/vrptw/operators/mixed_exchange.h"

namespace vroom {
namespace vrptw {

MixedExchange::MixedExchange(const Input& input,
                             const utils::SolutionState& sol_state,
                             TWRoute& tw_s_route,
                             Index s_vehicle,
                             Index s_rank,
                             TWRoute& tw_t_route,
                             Index t_vehicle,
                             Index t_rank)
  : cvrp::MixedExchange(input,
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
    _s_is_reverse_valid(false) {
}

void MixedExchange::compute_gain() {
  cvrp::MixedExchange::compute_gain();
  assert(_s_is_normal_valid or _s_is_reverse_valid);

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

  stored_gain = s_gain + t_gain;
}

bool MixedExchange::is_valid() {
  bool valid = cvrp::MixedExchange::is_valid();

  valid &= _tw_t_route.is_valid_addition_for_tw(_input,
                                                s_route.begin() + s_rank,
                                                s_route.begin() + s_rank + 1,
                                                t_rank,
                                                t_rank + 1);

  if (valid) {
    // Keep target edge direction when inserting in source route.
    auto t_start = t_route.begin() + t_rank;
    _s_is_normal_valid = _tw_s_route.is_valid_addition_for_tw(_input,
                                                              t_start,
                                                              t_start + 2,
                                                              s_rank,
                                                              s_rank + 1);
    // Reverse target edge direction when inserting in source route.
    auto t_reverse_start = t_route.rbegin() + t_route.size() - 2 - t_rank;
    _s_is_reverse_valid =
      _tw_s_route.is_valid_addition_for_tw(_input,
                                           t_reverse_start,
                                           t_reverse_start + 2,
                                           s_rank,
                                           s_rank + 1);
    valid = _s_is_normal_valid or _s_is_reverse_valid;
  }

  return valid;
}

void MixedExchange::apply() {
  std::vector<Index> s_job_ranks({s_route[s_rank]});
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

  _tw_s_route.replace(_input,
                      t_job_ranks.begin(),
                      t_job_ranks.end(),
                      s_rank,
                      s_rank + 1);

  _tw_t_route.replace(_input,
                      s_job_ranks.begin(),
                      s_job_ranks.end(),
                      t_rank,
                      t_rank + 2);
}

} // namespace vrptw
} // namespace vroom
