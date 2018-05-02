#ifndef VRP_H
#define VRP_H

/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <list>
#include <vector>

#include "../structures/typedefs.h"

class input;

class vrp {
  // Abstract class describing a VRP (vehicle routing problem).
protected:
  const input& _input;

public:
  vrp(const input& input);

  virtual ~vrp();

  virtual std::vector<std::list<index_t>> solve(unsigned nb_threads) const = 0;

  std::vector<std::list<index_t>> solve() const;
};

#endif
