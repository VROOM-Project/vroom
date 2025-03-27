#ifndef MATRIX_H
#define MATRIX_H

/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <vector>

#include "structures/typedefs.h"

namespace vroom {

template <class T> class Matrix {

  std::size_t n;
  std::vector<T> data;

public:
  Matrix() : Matrix(0) {
  }

  explicit Matrix(std::size_t n) : Matrix(n, 0) {
  }

  Matrix(std::size_t n, T value) : n(n), data(n * n, value) {
  }

  Matrix<T> get_sub_matrix(const std::vector<Index>& indices) const {
    Matrix<T> sub_matrix(indices.size());
    for (std::size_t i = 0; i < indices.size(); ++i) {
      for (std::size_t j = 0; j < indices.size(); ++j) {
        sub_matrix[i][j] = (*this)[indices[i]][indices[j]];
      }
    }
    return sub_matrix;
  }

  T* operator[](std::size_t i) {
    return data.data() + (i * n);
  }
  const T* operator[](std::size_t i) const {
    return data.data() + (i * n);
  }

  std::size_t size() const {
    return n;
  }

#if USE_PYTHON_BINDINGS
  T* get_data() {
    return data.data();
  }
#endif
};

} // namespace vroom

#endif
