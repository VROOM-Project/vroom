#ifndef CVRP_LS_OPERATOR_H
#define CVRP_LS_OPERATOR_H

/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/typedefs.h"
#include "structures/vroom/input/input.h"
#include "structures/vroom/solution_state.h"

class cvrp_ls_operator {
protected:
  const input& _input;
  raw_solution& _sol;
  const solution_state& _sol_state;

  const index_t source_vehicle;
  const index_t source_rank;
  const index_t target_vehicle;
  const index_t target_rank;
  bool gain_computed;
  gain_t stored_gain;

  virtual void compute_gain() = 0;

public:
  cvrp_ls_operator(const input& input,
                   raw_solution& sol,
                   const solution_state& sol_state,
                   index_t source_vehicle,
                   index_t source_rank,
                   index_t target_vehicle,
                   index_t target_rank);

  gain_t gain();

  virtual bool is_valid() const = 0;

  virtual void apply() const = 0;

  virtual std::vector<index_t> addition_candidates() const = 0;

  virtual ~cvrp_ls_operator() {
  }
};

#endif
