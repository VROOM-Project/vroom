/*

This file is part of VROOM.

Copyright (c) 2015-2019, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/cvrp/operators/pd_shift.h"
#include "utils/helpers.h"

namespace vroom {
namespace cvrp {

PDShift::PDShift(const Input& input,
                 const utils::SolutionState& sol_state,
                 RawRoute& s_route,
                 Index s_vehicle,
                 Index s_p_rank,
                 Index s_d_rank,
                 RawRoute& t_route,
                 Index t_vehicle,
                 Gain gain_threshold)
  : Operator(input, sol_state, s_route, s_vehicle, 0, t_route, t_vehicle, 0),
    _s_p_rank(s_p_rank),
    _s_d_rank(s_d_rank),
    _valid(false) {
  assert(s_vehicle != t_vehicle);
  assert(s_route.size() >= 2);
  assert(s_p_rank < s_d_rank);
  assert(s_d_rank < s_route.size());

  stored_gain = gain_threshold;

  // Compute gain when removing P&D from source route.
  const auto& m = _input.get_matrix();
  const auto& v = _input.vehicles[s_vehicle];
  Index pickup_index = _input.jobs[s_route.route[_s_p_rank]].index();
  Index delivery_index = _input.jobs[s_route.route[_s_d_rank]].index();

  if (_s_p_rank + 1 == _s_d_rank) {
    // Pickup and delivery in a row.
    Gain previous_cost = 0;
    Gain next_cost = 0;
    Gain new_edge_cost = 0;
    Index p_index;
    Index n_index;

    // Compute cost for step before pickup.
    bool has_previous_step = false;
    if (_s_p_rank > 0) {
      has_previous_step = true;
      p_index = _input.jobs[s_route.route[_s_p_rank - 1]].index();
      previous_cost = m[p_index][pickup_index];
    } else {
      if (v.has_start()) {
        has_previous_step = true;
        p_index = v.start.get().index();
        previous_cost = m[p_index][pickup_index];
      }
    }

    // Compute cost for step after delivery.
    bool has_next_step = false;
    if (_s_d_rank < s_route.size() - 1) {
      has_next_step = true;
      n_index = _input.jobs[s_route.route[_s_d_rank + 1]].index();
      next_cost = m[delivery_index][n_index];
    } else {
      if (v.has_end()) {
        has_next_step = true;
        n_index = v.end.get().index();
        next_cost = m[delivery_index][n_index];
      }
    }

    if (has_previous_step and has_next_step and (s_route.size() > 2)) {
      // No new edge with an open trip or if removing P&D creates an
      // empty route.
      new_edge_cost = m[p_index][n_index];
    }

    _remove_gain = previous_cost + m[pickup_index][delivery_index] + next_cost -
                   new_edge_cost;
  } else {
    // Simply add both gains as neighbouring edges are disjoint.
    _remove_gain = _sol_state.node_gains[s_vehicle][_s_p_rank] +
                   _sol_state.node_gains[s_vehicle][s_d_rank];
  }
}

Gain PDShift::get_remove_gain() const {
  return _remove_gain;
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

bool PDShift::is_valid() {
  assert(gain_computed);
  return _valid;
}

void PDShift::apply() {
  std::vector<Index> target_with_pd({s_route[_s_p_rank]});
  std::copy(t_route.begin() + _best_t_p_rank,
            t_route.begin() + _best_t_d_rank,
            std::back_inserter(target_with_pd));
  target_with_pd.push_back(s_route[_s_d_rank]);

  target.replace(_input,
                 target_with_pd.begin(),
                 target_with_pd.end(),
                 _best_t_p_rank,
                 _best_t_d_rank);

  if (_s_d_rank == _s_p_rank + 1) {
    s_route.erase(s_route.begin() + _s_p_rank, s_route.begin() + _s_p_rank + 2);
  } else {
    std::vector<Index> source_without_pd(s_route.begin() + _s_p_rank + 1,
                                         s_route.begin() + _s_d_rank);
    source.replace(_input,
                   source_without_pd.begin(),
                   source_without_pd.end(),
                   _s_p_rank,
                   _s_d_rank + 1);
  }
}

std::vector<Index> PDShift::addition_candidates() const {
  return {s_vehicle};
}

std::vector<Index> PDShift::update_candidates() const {
  return {s_vehicle, t_vehicle};
}

} // namespace cvrp
} // namespace vroom
