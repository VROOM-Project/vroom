/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/vrptw/operators/intra_relocate.h"

namespace vroom {
namespace vrptw {

IntraRelocate::IntraRelocate(const Input& input,
                             const utils::SolutionState& sol_state,
                             TWRoute& tw_s_route,
                             Index s_vehicle,
                             Index s_rank,
                             Index t_rank)
  : cvrp::IntraRelocate(input,
                        sol_state,
                        static_cast<RawRoute&>(tw_s_route),
                        s_vehicle,
                        s_rank,
                        t_rank),
    _tw_s_route(tw_s_route),
    _moved_jobs((s_rank < t_rank) ? t_rank - s_rank + 1 : s_rank - t_rank + 1),
    _first_rank(std::min(s_rank, t_rank)),
    _last_rank(std::max(s_rank, t_rank) + 1) {
  if (t_rank < s_rank) {
    _moved_jobs[0] = s_route[s_rank];
    std::copy(s_route.begin() + t_rank,
              s_route.begin() + s_rank,
              _moved_jobs.begin() + 1);
  } else {
    std::copy(s_route.begin() + s_rank + 1,
              s_route.begin() + t_rank + 1,
              _moved_jobs.begin());
    _moved_jobs.back() = s_route[s_rank];
  }
}

bool IntraRelocate::is_valid() {
  return _tw_s_route.is_valid_addition_for_tw(_input,
                                              _moved_jobs.begin(),
                                              _moved_jobs.end(),
                                              _first_rank,
                                              _last_rank);
}

void IntraRelocate::apply() {
  _tw_s_route.replace(_input,
                      _moved_jobs.begin(),
                      _moved_jobs.end(),
                      _first_rank,
                      _last_rank);
}

std::vector<Index> IntraRelocate::addition_candidates() const {
  return {s_vehicle};
}

} // namespace vrptw
} // namespace vroom
