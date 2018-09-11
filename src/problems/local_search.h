#ifndef LOCAL_SEARCH_H
#define LOCAL_SEARCH_H

/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/typedefs.h"
#include "structures/vroom/input/input.h"
#include "structures/vroom/solution_state.h"

struct solution_indicators {
  unsigned unassigned;
  cost_t cost;
  unsigned used_vehicles;
};

class local_search {
protected:
  const input& _input;
  const matrix<cost_t>& _m;
  const std::size_t V;
  const amount_t _amount_lower_bound;
  const amount_t _double_amount_lower_bound;

  solution_state _sol_state;

public:
  local_search(const input& input, const raw_solution& sol);

  virtual ~local_search();

  virtual solution_indicators indicators() const = 0;

  virtual void run() = 0;
};

#endif
