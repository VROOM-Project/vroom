/*

This file is part of VROOM.

Copyright (c) 2015-2019, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/vrptw/operators/pd_shift.h"
#include "utils/helpers.h"

namespace vroom {
namespace vrptw {

PDShift::PDShift(const Input& input,
                 const utils::SolutionState& sol_state,
                 TWRoute& tw_s_route,
                 Index s_vehicle,
                 Index s_p_rank,
                 Index s_d_rank,
                 TWRoute& tw_t_route,
                 Index t_vehicle,
                 Gain gain_threshold)
  : cvrp::PDShift(input,
                  sol_state,
                  static_cast<RawRoute&>(tw_s_route),
                  s_vehicle,
                  s_p_rank,
                  s_d_rank,
                  static_cast<RawRoute&>(tw_t_route),
                  t_vehicle,
                  gain_threshold),
    _tw_s_route(tw_s_route),
    _tw_t_route(tw_t_route) {
}

void PDShift::compute_gain() {
  const auto& m = _input.get_matrix();
  const auto& v = _input.vehicles[t_vehicle];

  // For source vehicle, we consider the cost of removing P&D already
  // stored in remove_gain.

  // For target vehicle, we go through all insertion options for
  // pickup and delivery target ranks, storing best ranks along the
  // way.
  for (unsigned t_p_rank = 0; t_p_rank < t_route.size(); ++t_p_rank) {
    Gain t_p_gain = -utils::addition_cost(_input,
                                          m,
                                          s_route[_s_p_rank],
                                          v,
                                          t_route,
                                          t_p_rank);

    // TODO use early abort
    // if (_remove_gain + t_p_gain < stored_gain) {
    //   // Over best known gain stored so far even without delivery
    //   // insertion.
    //   continue;
    // }

    std::vector<Index> modified_with_pd({s_route[_s_p_rank]});

    for (unsigned t_d_rank = t_p_rank; t_d_rank <= t_route.size(); ++t_d_rank) {
      Gain target_gain;
      if (t_p_rank == t_d_rank) {
        target_gain = -utils::addition_cost(_input,
                                            m,
                                            s_route[_s_p_rank],
                                            v,
                                            t_route,
                                            t_p_rank,
                                            t_p_rank + 1);
      } else {
        // TODO precompute t_d_gain ?
        target_gain = t_p_gain - utils::addition_cost(_input,
                                                      m,
                                                      s_route[_s_d_rank],
                                                      v,
                                                      t_route,
                                                      t_d_rank);
        modified_with_pd.push_back(t_route[t_d_rank - 1]);
      }

      if (_remove_gain + target_gain > stored_gain) {
        modified_with_pd.push_back(s_route[_s_d_rank]);
        bool valid =
          target
            .is_valid_addition_for_capacity_inclusion(_input,
                                                      _input.zero_amount(),
                                                      modified_with_pd.begin(),
                                                      modified_with_pd.end(),
                                                      t_p_rank,
                                                      t_d_rank);
        valid &= _tw_t_route.is_valid_addition_for_tw(_input,
                                                      modified_with_pd.begin(),
                                                      modified_with_pd.end(),
                                                      t_p_rank,
                                                      t_d_rank);

        // Checking for removal validity is like checking insertion of
        // source portion without P&D in place.
        valid &=
          _tw_s_route.is_valid_addition_for_tw(_input,
                                               s_route.begin() + _s_p_rank + 1,
                                               s_route.begin() + _s_d_rank,
                                               _s_p_rank,
                                               _s_d_rank + 1);

        modified_with_pd.pop_back();

        if (valid) {
          _valid = true;
          stored_gain = _remove_gain + target_gain;
          _best_t_p_rank = t_p_rank;
          _best_t_d_rank = t_d_rank;
        }
      }
    }
  }

  gain_computed = true;
}

void PDShift::apply() {
  std::vector<Index> target_with_pd({s_route[_s_p_rank]});
  std::copy(t_route.begin() + _best_t_p_rank,
            t_route.begin() + _best_t_d_rank,
            std::back_inserter(target_with_pd));
  target_with_pd.push_back(s_route[_s_d_rank]);

  _tw_t_route.replace(_input,
                      target_with_pd.begin(),
                      target_with_pd.end(),
                      _best_t_p_rank,
                      _best_t_d_rank);

  if (_s_d_rank == _s_p_rank + 1) {
    _tw_s_route.remove(_input, _s_p_rank, 2);
  } else {
    std::vector<Index> source_without_pd(s_route.begin() + _s_p_rank + 1,
                                         s_route.begin() + _s_d_rank);

    _tw_s_route.replace(_input,
                        source_without_pd.begin(),
                        source_without_pd.end(),
                        _s_p_rank,
                        _s_d_rank + 1);
  }
}

} // namespace vrptw
} // namespace vroom
