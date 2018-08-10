#ifndef AMOUNT_H
#define AMOUNT_H

/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <vector>

#include "structures/typedefs.h"

template <typename E> class amount_expression_t {
public:
  capacity_t operator[](size_t i) const {
    return static_cast<const E&>(*this)[i];
  };
  std::size_t size() const {
    return static_cast<const E&>(*this).size();
  };
  bool empty() const {
    return size() == 0;
  };
};

template <typename E1, typename E2>
bool operator<(const amount_expression_t<E1>& lhs,
               const amount_expression_t<E2>& rhs) {
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

template <typename E1, typename E2>
bool operator<=(const amount_expression_t<E1>& lhs,
                const amount_expression_t<E2>& rhs) {
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

template <typename E1, typename E2>
bool operator==(const amount_expression_t<E1>& lhs,
                const amount_expression_t<E2>& rhs) {
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

class amount_t : public amount_expression_t<amount_t> {

  std::vector<capacity_t> elems;

public:
  amount_t() = default;

  amount_t(std::size_t size) : elems(size, 0){};

  template <typename E> amount_t(const amount_expression_t<E>& u) {
    elems.resize(u.size());
    for (std::size_t i = 0; i < u.size(); ++i) {
      elems[i] = u[i];
    }
  }

  void push_back(capacity_t c) {
    elems.push_back(c);
  };

  capacity_t operator[](std::size_t i) const {
    return elems[i];
  };

  capacity_t& operator[](std::size_t i) {
    return elems[i];
  };

  std::size_t size() const {
    return elems.size();
  };

  amount_t& operator+=(const amount_t& rhs);

  amount_t& operator-=(const amount_t& rhs);
};

template <typename E1, typename E2>
class amount_sum_t : public amount_expression_t<amount_sum_t<E1, E2>> {
  E1 const& lhs;
  E2 const& rhs;

public:
  amount_sum_t(const E1& a, const E2& b) : lhs(a), rhs(b) {
    assert(a.size() == b.size());
  };

  capacity_t operator[](std::size_t i) const {
    return lhs[i] + rhs[i];
  };

  std::size_t size() const {
    return lhs.size();
  };
};

template <typename E1, typename E2>
auto operator+(const amount_expression_t<E1>& u,
               const amount_expression_t<E2>& v)
  -> amount_sum_t<amount_expression_t<E1>, amount_expression_t<E2>> {
  return {u, v};
}

template <typename E1, typename E2>
class amount_diff_t : public amount_expression_t<amount_diff_t<E1, E2>> {
  const E1& lhs;
  const E2& rhs;

public:
  amount_diff_t(const E1& a, const E2& b) : lhs(a), rhs(b) {
    assert(a.size() == b.size());
  };

  capacity_t operator[](std::size_t i) const {
    return lhs[i] - rhs[i];
  };
  
  std::size_t size() const {
    return lhs.size();
  };
};

template <typename E1, typename E2>
auto operator-(const amount_expression_t<E1>& lhs,
               const amount_expression_t<E2>& rhs)
  -> amount_diff_t<amount_expression_t<E1>, amount_expression_t<E2>> {
  return {lhs, rhs};
}

#endif
