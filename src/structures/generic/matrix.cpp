/*

This file is part of VROOM.

Copyright (c) 2015-2021, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/generic/matrix.h"

namespace vroom {

template <class T> Matrix<T>::Matrix(std::size_t n) : n(n) {
  data.resize(n * n);
}

template <class T> Matrix<T>::Matrix() : Matrix(0) {
}

template <class T>
Matrix<T> Matrix<T>::get_sub_matrix(const std::vector<Index>& indices) const {
  Matrix<T> sub_matrix(indices.size());
  for (std::size_t i = 0; i < indices.size(); ++i) {
    for (std::size_t j = 0; j < indices.size(); ++j) {
      sub_matrix[i][j] = (*this)[indices[i]][indices[j]];
    }
  }
  return sub_matrix;
}

template class Matrix<Cost>;

} // namespace vroom
