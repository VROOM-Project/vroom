#ifndef CVRP_ROUTE_SHIFT_H
#define CVRP_ROUTE_SHIFT_H

/*

This file is part of VROOM.

Copyright (c) 2015-2022, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "algorithms/local_search/operator.h"

namespace vroom {
namespace cvrp {

class RouteShift : public ls::Operator {
private:
  bool _gain_upper_bound_computed;
  Eval _start_t_gain;
  Eval _end_t_gain;

protected:
  bool is_start_valid;
  bool is_end_valid;
  bool shift_to_start;
  bool shift_to_end;

  virtual void compute_gain() override;

public:
  RouteShift(const Input& input,
             const utils::SolutionState& sol_state,
             RawRoute& s_route,
             Index s_vehicle,
             RawRoute& t_route,
             Index t_vehicle);

  // Compute and store all possible costs depending on whether the
  // insertion happens at the beginning or the end of target route.
  Eval gain_upper_bound();

  virtual bool is_valid() override;

  virtual void apply() override;

  virtual std::vector<Index> addition_candidates() const override;

  virtual std::vector<Index> update_candidates() const override;
};

} // namespace cvrp
} // namespace vroom

#endif
