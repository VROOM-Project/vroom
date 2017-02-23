#ifndef MATRIX_H
#define MATRIX_H

/*

This file is part of VROOM.

Copyright (c) 2015-2017, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <vector>

#include "../typedefs.h"

template <class T>
class line : private std::vector<T> {

  using parent = std::vector<T>;

public:
  using parent::size;
  using parent::operator[];

  line(std::size_t n);
};

template <class T>
class matrix : private std::vector<line<T>> {

  using parent = std::vector<line<T>>;

public:
  using parent::size;
  using parent::operator[];

  matrix();

  matrix(std::size_t n);

  matrix<T> get_sub_matrix(const std::vector<index_t>& indices) const;
};

#endif
