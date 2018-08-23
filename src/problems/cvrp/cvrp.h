#ifndef CVRP_H
#define CVRP_H

/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/vrp.h"

class cvrp : public vrp {
private:
  bool empty_cluster(const std::vector<index_t>& cluster, index_t v) const;

public:
  cvrp(const input& input);

  virtual solution solve(unsigned exploration_level,
                         unsigned nb_threads) const override;
};

#endif
