#ifndef EDGE_H
#define EDGE_H

/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/typedefs.h"

namespace vroom::utils {

template <class T> class Edge {

private:
  Index _first_vertex;
  Index _second_vertex;
  T _weight;

public:
  Edge(Index first_vertex, Index second_vertex, T weight)
    : _first_vertex(std::min(first_vertex, second_vertex)),
      _second_vertex(std::max(first_vertex, second_vertex)),
      _weight(weight) {
  }

  Index get_first_vertex() const {
    return _first_vertex;
  };

  Index get_second_vertex() const {
    return _second_vertex;
  };

  bool operator<(const Edge& rhs) const {
    return (this->_first_vertex < rhs._first_vertex) ||
           ((this->_first_vertex == rhs._first_vertex) &&
            (this->_second_vertex < rhs._second_vertex));
  }

  bool operator==(const Edge& rhs) const {
    return (this->_first_vertex == rhs._first_vertex) &&
           (this->_second_vertex == rhs._second_vertex);
  }

  T get_weight() const {
    return _weight;
  };
};

} // namespace vroom::utils

#endif
