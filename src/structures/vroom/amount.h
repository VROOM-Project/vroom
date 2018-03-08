#ifndef AMOUNT_H
#define AMOUNT_H

/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <vector>

#include "../typedefs.h"

class amount_t : private std::vector<capacity_t> {

  using parent = std::vector<capacity_t>;

public:
  amount_t();
  amount_t(std::size_t size);

  using parent::size;
  using parent::operator[];
  using parent::push_back;

  amount_t& operator+=(const amount_t& rhs);

  amount_t& operator-=(const amount_t& rhs);

  friend amount_t operator+(amount_t lhs, const amount_t& rhs);

  friend amount_t operator-(amount_t lhs, const amount_t& rhs);

  friend bool operator<(const amount_t& lhs, const amount_t& rhs);

  friend bool operator<=(const amount_t& lhs, const amount_t& rhs);

  friend bool operator==(const amount_t& lhs, const amount_t& rhs);
};

#endif
