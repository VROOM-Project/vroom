#ifndef CVRP_LOCAL_SEARCH_H
#define CVRP_LOCAL_SEARCH_H

/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "../../../structures/typedefs.h"

class input;

class cvrp_local_search {
private:
  const input& _input;
  raw_solution& _sol;

public:
  cvrp_local_search(const input& input, raw_solution& sol);

  void run();
};

#endif
