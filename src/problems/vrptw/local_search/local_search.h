#ifndef VRPTW_LOCAL_SEARCH_H
#define VRPTW_LOCAL_SEARCH_H

/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/local_search.h"
#include "structures/vroom/tw_route.h"

using tw_solution = std::vector<tw_route>;

class vrptw_local_search : public local_search {
private:
  tw_solution& _tw_sol;

public:
  vrptw_local_search(const input& input, tw_solution& tw_sol);

  virtual solution_indicators indicators() const override;

  virtual void run() override;
};

#endif
