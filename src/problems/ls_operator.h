#ifndef LS_OPERATOR_H
#define LS_OPERATOR_H

/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/typedefs.h"
#include "structures/vroom/input/input.h"
#include "structures/vroom/solution_state.h"

class ls_operator {
protected:
  const input& _input;
  const solution_state& _sol_state;

  // Source of move for this operator.
  std::vector<index_t>& s_route;
  const index_t s_vehicle;
  const index_t s_rank;
  // Target of move for this operator.
  std::vector<index_t>& t_route;
  const index_t t_vehicle;
  const index_t t_rank;

  bool gain_computed;
  gain_t stored_gain;

  virtual void compute_gain() = 0;

public:
  ls_operator(const input& input,
              const solution_state& sol_state,
              std::vector<index_t>& s_route,
              index_t s_vehicle,
              index_t s_rank,
              std::vector<index_t>& t_route,
              index_t t_vehicle,
              index_t t_rank);

  virtual gain_t gain();

  virtual bool is_valid() const = 0;

  virtual void apply() const = 0;

  virtual std::vector<index_t> addition_candidates() const = 0;

  virtual ~ls_operator() {
  }
};

#endif
