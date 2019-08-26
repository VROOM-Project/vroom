/*

This file is part of VROOM.

Copyright (c) 2015-2019, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/vrptw/operators/intra_mixed_exchange.h"

namespace vroom {
namespace vrptw {

IntraMixedExchange::IntraMixedExchange(const Input& input,
                                       const utils::SolutionState& sol_state,
                                       TWRoute& tw_s_route,
                                       Index s_vehicle,
                                       Index s_rank,
                                       Index t_rank)
  : cvrp::IntraMixedExchange(input,
                             sol_state,
                             static_cast<RawRoute&>(tw_s_route),
                             s_vehicle,
                             s_rank,
                             t_rank),
    _tw_s_route(tw_s_route) {
}

bool IntraMixedExchange::is_valid() {
  bool valid = cvrp::IntraMixedExchange::is_valid();

  if (valid) {
    _s_is_normal_valid &=
      _tw_s_route.is_valid_addition_for_tw(_input,
                                           _moved_jobs.begin(),
                                           _moved_jobs.end(),
                                           _first_rank,
                                           _last_rank);

    std::swap(_moved_jobs[_t_edge_first], _moved_jobs[_t_edge_last]);

    _s_is_reverse_valid &=
      _tw_s_route.is_valid_addition_for_tw(_input,
                                           _moved_jobs.begin(),
                                           _moved_jobs.end(),
                                           _first_rank,
                                           _last_rank);

    // Reset to initial situation before potential application.
    std::swap(_moved_jobs[_t_edge_first], _moved_jobs[_t_edge_last]);

    valid = _s_is_normal_valid or _s_is_reverse_valid;
  }

  return valid;
}

void IntraMixedExchange::apply() {
  if (reverse_t_edge) {
    std::swap(_moved_jobs[_t_edge_first], _moved_jobs[_t_edge_last]);
  }

  _tw_s_route.replace(_input,
                      _moved_jobs.begin(),
                      _moved_jobs.end(),
                      _first_rank,
                      _last_rank);
}

std::vector<Index> IntraMixedExchange::addition_candidates() const {
  return {s_vehicle};
}

} // namespace vrptw
} // namespace vroom
