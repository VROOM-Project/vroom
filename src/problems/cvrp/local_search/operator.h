#ifndef LS_OPERATOR_H
#define LS_OPERATOR_H

/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "../../../structures/typedefs.h"
#include "../../../structures/vroom/input/input.h"

class ls_operator {
protected:
  const input& _input;
  raw_solution& _sol;
  std::vector<amount_t>& _amounts;

  const index_t source_vehicle;
  const index_t source_rank;
  const index_t target_vehicle;
  const index_t target_rank;
  bool gain_computed;
  gain_t stored_gain;

  virtual void compute_gain() = 0;

public:
  ls_operator(const input& input,
              raw_solution& sol,
              std::vector<amount_t>& amounts,
              index_t source_vehicle,
              index_t source_rank,
              index_t target_vehicle,
              index_t target_rank);

  gain_t gain() const;

  virtual bool is_valid() const = 0;

  virtual void apply() const = 0;

  virtual void log() const = 0;

  virtual ~ls_operator() {
  }

  friend bool operator<(const ls_operator& lhs, const ls_operator& rhs);
};

#endif
