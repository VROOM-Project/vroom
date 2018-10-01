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

class local_search {
protected:
  const input& _input;
  const matrix<cost_t>& _m;
  const std::size_t V;
  const amount_t _amount_lower_bound;
  const amount_t _double_amount_lower_bound;

  const unsigned _max_nb_jobs_removal;
  std::vector<index_t> _all_routes;

  solution_state _sol_state;

public:
  local_search(const input& input, unsigned max_nb_jobs_removal);

  virtual ~local_search();

  virtual solution_indicators indicators() const = 0;

  virtual void run() = 0;
};

#endif
