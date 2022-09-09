/*

This file is part of VROOM.

Copyright (c) 2015-2022, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "algorithms/local_search/operator.h"

namespace vroom {
namespace ls {

OperatorName Operator::get_name() const {
  return _name;
}

Eval Operator::gain() {
  if (!gain_computed) {
    this->compute_gain();
  }
  return stored_gain;
}

bool Operator::is_valid_for_source_max_travel_time() const {
  const auto& s_v = _input.vehicles[s_vehicle];
  const auto s_travel_time = _sol_state.route_evals[s_vehicle].duration;
  return s_travel_time <= s_v.max_travel_time + s_gain.duration;
}

bool Operator::is_valid_for_target_max_travel_time() const {
  const auto& t_v = _input.vehicles[t_vehicle];
  const auto t_travel_time = _sol_state.route_evals[t_vehicle].duration;
  return t_travel_time <= t_v.max_travel_time + t_gain.duration;
}

std::vector<Index> Operator::required_unassigned() const {
  return std::vector<Index>();
}

} // namespace ls
} // namespace vroom
