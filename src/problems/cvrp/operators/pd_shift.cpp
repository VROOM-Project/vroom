/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/cvrp/operators/pd_shift.h"

#include "algorithms/local_search/insertion_search.h"

namespace vroom::cvrp {

PDShift::PDShift(const Input& input,
                 const utils::SolutionState& sol_state,
                 RawRoute& s_route,
                 Index s_vehicle,
                 Index s_p_rank,
                 Index s_d_rank,
                 RawRoute& t_route,
                 Index t_vehicle,
                 const Eval& gain_threshold)
  : Operator(OperatorName::PDShift,
             input,
             sol_state,
             s_route,
             s_vehicle,
             0,
             t_route,
             t_vehicle,
             0),
    _s_p_rank(s_p_rank),
    _s_d_rank(s_d_rank) {
  assert(s_vehicle != t_vehicle);
  assert(s_route.size() >= 2);
  assert(s_p_rank < s_d_rank);
  assert(s_d_rank < s_route.size());
  assert(s_route.route[s_p_rank] + 1 == s_route.route[s_d_rank]);

  s_gain = _sol_state.pd_gains[s_vehicle][_s_p_rank];

  if (t_route.empty()) {
    t_gain.cost -= _input.vehicles[t_vehicle].fixed_cost();
  }

  assert(_input.vehicles[s_vehicle].ok_for_range_bounds(
    _sol_state.route_evals[s_vehicle] - s_gain));

  stored_gain = gain_threshold;
}

void PDShift::compute_gain() {
  // In preference mode any feasible insertion qualifies (NO_EVAL
  // threshold idiom): a shift that gains preference may win with a
  // negative Eval gain, and the lexicographic key then discards the
  // preference-losing or preference-neutral regressions.
  const Eval pd_threshold = (has_soft_skills() && has_preference_context())
                              ? NO_EVAL
                              : s_gain - stored_gain;

  if (const ls::RouteInsertion rs =
        ls::compute_best_insertion_pd(_input,
                                      _sol_state,
                                      s_route[_s_p_rank],
                                      t_vehicle,
                                      target,
                                      pd_threshold);
      rs.eval != NO_EVAL) {
    _valid = true;
    t_gain -= rs.eval;
    stored_gain = s_gain + t_gain;
    _best_t_p_rank = rs.pickup_rank;
    _best_t_d_rank = rs.delivery_rank;

    if (has_soft_skills() && has_preference_context()) {
      // The shipment moves as a whole, so its preference counts once
      // through the pickup endpoint, on the actual insertion eval.
      const auto job_rank = s_route[_s_p_rank];
      const int post =
        task_post_match(job_rank, t_vehicle, [&]() { return rs.eval; }) ? 1
                                                                        : 0;
      preference_delta =
        post - (task_pre_match(job_rank, s_vehicle) ? 1 : 0);
    }
  }

  gain_computed = true;
}

bool PDShift::is_valid() {
  assert(gain_computed);
  return _valid;
}

void PDShift::apply() {
  std::vector<Index> target_with_pd;
  target_with_pd.reserve(_best_t_d_rank - _best_t_p_rank + 2);
  target_with_pd.push_back(s_route[_s_p_rank]);

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
    source.update_amounts(_input);
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

} // namespace vroom::cvrp
