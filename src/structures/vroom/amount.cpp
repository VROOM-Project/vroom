/*

This file is part of VROOM.

Copyright (c) 2015-2017, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "amount.h"

amount_t::amount_t() : parent() {
}

amount_t::amount_t(std::size_t size) : parent(size, 0) {
}

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

amount_t operator+(amount_t lhs, const amount_t& rhs) {
  lhs += rhs;
  return lhs;
}

amount_t operator-(amount_t lhs, const amount_t& rhs) {
  lhs -= rhs;
  return lhs;
}

bool operator<(const amount_t& lhs, const amount_t& rhs) {
  bool is_strict_inf = true;
  assert(lhs.size() == rhs.size());
  for (std::size_t i = 0; i < lhs.size(); ++i) {
    if (lhs[i] >= rhs[i]) {
      is_strict_inf = false;
      break;
    }
  }

  return is_strict_inf;
}

bool operator<=(const amount_t& lhs, const amount_t& rhs) {
  bool is_inf = true;
  assert(lhs.size() == rhs.size());
  for (std::size_t i = 0; i < lhs.size(); ++i) {
    if (lhs[i] > rhs[i]) {
      is_inf = false;
      break;
    }
  }

  return is_inf;
}

bool operator==(const amount_t& lhs, const amount_t& rhs) {
  bool is_equal = true;
  assert(lhs.size() == rhs.size());
  for (std::size_t i = 0; i < lhs.size(); ++i) {
    if (lhs[i] != rhs[i]) {
      is_equal = false;
      break;
    }
  }

  return is_equal;
}
