#ifndef EDGE_H
#define EDGE_H

/*

This file is part of VROOM.

Copyright (c) 2015-2016, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <iostream>
#include <algorithm>
#include "./typedefs.h"

template <class T> class edge{

private:
  index_t _first_vertex;
  index_t _second_vertex;
  T _weight;

public:
  edge(index_t first_vertex,
       index_t second_vertex,
       T weight):
    _first_vertex(std::min(first_vertex, second_vertex)),
    _second_vertex(std::max(first_vertex, second_vertex)),
    _weight(weight) {}
  
  index_t get_first_vertex() const{
    return _first_vertex;
  }

  index_t get_second_vertex() const{
    return _second_vertex;
  }

  bool operator<(const edge& rhs) const{
  return (this->_first_vertex < rhs._first_vertex)
    or ((this->_first_vertex == rhs._first_vertex)
        and (this->_second_vertex < rhs._second_vertex));
  }

  bool operator==(const edge& rhs) const{
    return (this->_first_vertex == rhs._first_vertex)
      and (this->_second_vertex == rhs._second_vertex);
  }

  T get_weight() const{
    return _weight;
  }
};

#endif
