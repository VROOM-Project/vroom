/*

This file is part of VROOM.

Copyright (c) 2015-2019, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/vrptw/operators/intra_exchange.h"

namespace vroom {
namespace vrptw {

IntraExchange::IntraExchange(const Input& input,
                             const utils::SolutionState& sol_state,
                             TWRoute& tw_s_route,
                             Index s_vehicle,
                             Index s_rank,
                             Index t_rank)
  : cvrp::IntraExchange(input,
                        sol_state,
                        static_cast<RawRoute&>(tw_s_route),
                        s_vehicle,
                        s_rank,
                        t_rank),
    _tw_s_route(tw_s_route),
    _moved_jobs(t_rank - s_rank + 1),
    _first_rank(s_rank),
    _last_rank(t_rank + 1) {
  std::copy(s_route.begin() + s_rank,
            s_route.begin() + t_rank + 1,
            _moved_jobs.begin());
  std::swap(_moved_jobs[0], _moved_jobs.back());
}

bool IntraExchange::is_valid() {
  return _tw_s_route.is_valid_addition_for_tw(_input,
                                              _moved_jobs.begin(),
                                              _moved_jobs.end(),
                                              _first_rank,
                                              _last_rank);
}

void IntraExchange::apply() {
  _tw_s_route.replace(_input,
                      _moved_jobs.begin(),
                      _moved_jobs.end(),
                      _first_rank,
                      _last_rank);
}

std::vector<Index> IntraExchange::addition_candidates() const {
  return {s_vehicle};
}

} // namespace vrptw
} // namespace vroom
