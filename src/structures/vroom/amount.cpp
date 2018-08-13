/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/vroom/amount.h"

amount_t& amount_t::operator+=(const amount_t& rhs) {
  assert(this->size() == rhs.size());
  for (std::size_t i = 0; i < this->size(); ++i) {
    (*this)[i] += rhs[i];
  }
  return *this;
}

amount_t& amount_t::operator-=(const amount_t& rhs) {
  assert(this->size() == rhs.size());
  for (std::size_t i = 0; i < this->size(); ++i) {
    (*this)[i] -= rhs[i];
  }
  return *this;
}

