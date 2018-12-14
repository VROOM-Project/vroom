#ifndef ROUTING_H
#define ROUTING_H

/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <vector>

#include "structures/abstract/matrix.h"
#include "structures/vroom/location.h"
#include "structures/vroom/solution/route.h"

template <class T> class Routing {

public:
  virtual Matrix<T> get_matrix(const std::vector<Location>& locs) const = 0;

  virtual void add_route_info(Route& route) const = 0;

  virtual ~Routing() {
  }

protected:
  Routing() {
  }
};

#endif
