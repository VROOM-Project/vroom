#ifndef CVRP_H
#define CVRP_H

/*

This file is part of VROOM.

Copyright (c) 2015-2017, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "../tsp/tsp.h"
#include "../vrp.h"
#include "./heuristics/clustering.h"

class cvrp : public vrp {
public:
  cvrp(const input& input);

  virtual solution solve(unsigned nb_threads) const override;
};

#endif
