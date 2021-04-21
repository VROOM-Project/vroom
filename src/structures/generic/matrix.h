#ifndef MATRIX_H
#define MATRIX_H

/*

This file is part of VROOM.

Copyright (c) 2015-2021, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <initializer_list>

#include "structures/typedefs.h"

namespace vroom {

template <class T> class Matrix {

  std::size_t n;
  std::vector<T> data;

public:
  Matrix();

  Matrix(std::size_t n);

  Matrix<T> get_sub_matrix(const std::vector<Index>& indices) const;

  T* operator[](std::size_t i) {
    return data.data() + (i * n);
  }
  const T* operator[](std::size_t i) const {
    return data.data() + (i * n);
  }

  std::size_t size() const {
    return n;
  }
};

} // namespace vroom

#endif
