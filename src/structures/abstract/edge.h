#ifndef EDGE_H
#define EDGE_H

/*

This file is part of VROOM.

Copyright (c) 2015-2017, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <algorithm>

#include "../typedefs.h"

template <class T> class edge {

private:
  index_t _first_vertex;
  index_t _second_vertex;
  T _weight;

public:
  edge(index_t first_vertex, index_t second_vertex, T weight);

  index_t get_first_vertex() const;

  index_t get_second_vertex() const;

  bool operator<(const edge& rhs) const;

  bool operator==(const edge& rhs) const;

  T get_weight() const;
};

#endif
