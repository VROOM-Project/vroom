#ifndef OPERATOR_H
#define OPERATOR_H

/*

This file is part of VROOM.

Copyright (c) 2015-2019, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/typedefs.h"
#include "structures/vroom/input/input.h"
#include "structures/vroom/raw_route.h"
#include "structures/vroom/solution_state.h"

namespace vroom {
namespace ls {

class Operator {
protected:
  const Input& _input;
  const utils::SolutionState& _sol_state;

  // Source of move for this operator.
  std::vector<Index>& s_route;
  const Index s_vehicle;
  const Index s_rank;
  // Target of move for this operator.
  std::vector<Index>& t_route;
  const Index t_vehicle;
  const Index t_rank;

  bool gain_computed;
  Gain stored_gain;

  virtual void compute_gain() = 0;

public:
  Operator(const Input& input,
           const utils::SolutionState& sol_state,
           RawRoute& s_route,
           Index s_vehicle,
           Index s_rank,
           RawRoute& t_route,
           Index t_vehicle,
           Index t_rank);

  virtual Gain gain();

  virtual bool is_valid() = 0;

  virtual void apply() = 0;

  virtual std::vector<Index> addition_candidates() const = 0;

  virtual std::vector<Index> update_candidates() const = 0;

  virtual ~Operator() {
  }
};

} // namespace ls
} // namespace vroom

#endif
