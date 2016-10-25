#ifndef VRP_H
#define VRP_H

/*

This file is part of VROOM.

Copyright (c) 2015-2016, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "../structures/vroom/output/output.h"

class input;

class vrp{
  // Abstract class describing a VRP (vehicle routing problem).
protected:
  const input& _input;

public:
  vrp(const input& input);

  virtual output solve(unsigned nb_threads) const = 0;

  output solve() const{
    return solve(1);
  };
};

#endif
