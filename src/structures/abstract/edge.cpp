/*

This file is part of VROOM.

Copyright (c) 2015-2017, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "edge.h"

template <class T>
edge<T>::edge(index_t first_vertex,
              index_t second_vertex,
              T weight):
  _first_vertex(std::min(first_vertex, second_vertex)),
  _second_vertex(std::max(first_vertex, second_vertex)),
  _weight(weight) {}

template <class T>
index_t edge<T>::get_first_vertex() const{
  return _first_vertex;
}

template <class T>
index_t edge<T>::get_second_vertex() const{
  return _second_vertex;
}

template <class T>
bool edge<T>::operator<(const edge& rhs) const{
  return (this->_first_vertex < rhs._first_vertex)
    or ((this->_first_vertex == rhs._first_vertex)
        and (this->_second_vertex < rhs._second_vertex));
}

template <class T>
bool edge<T>::operator==(const edge& rhs) const{
  return (this->_first_vertex == rhs._first_vertex)
    and (this->_second_vertex == rhs._second_vertex);
}

template <class T>
T edge<T>::get_weight() const{
  return _weight;
}

template class edge<distance_t>;
