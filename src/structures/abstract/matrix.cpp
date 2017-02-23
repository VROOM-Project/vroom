/*

This file is part of VROOM.

Copyright (c) 2015-2017, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "matrix.h"

template <class T>
line<T>::line(std::size_t n):
  parent(n) {}

template <class T>
matrix<T>::matrix(std::size_t n) : parent(n, line<T>(n)) {}

template <class T>
matrix<T>::matrix() : matrix(0) {}

template <class T>
matrix<T> matrix<T>::get_sub_matrix(const std::vector<index_t>& indices) const {
  matrix<T> sub_matrix{indices.size()};
  for (std::size_t i = 0; i < indices.size(); ++i) {
    for (std::size_t j = 0; j < indices.size(); ++j) {
      sub_matrix[i][j] = (*this)[indices[i]][indices[j]];
    }
  }
  return sub_matrix;
}

template class matrix<distance_t>;
template class line<distance_t>;