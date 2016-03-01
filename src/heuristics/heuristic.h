#ifndef HEURISTIC_H
#define HEURISTIC_H

/*

This file is part of VROOM.

Copyright (c) 2015-2016, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include<string>
#include<list>
#include "../structures/typedefs.h"
#include "../structures/tsp.h"

class heuristic{

public:
  virtual std::list<index_t> build_solution(const tsp& instance) = 0;

  virtual ~heuristic() {}

protected:
  heuristic();
};

#endif
