#ifndef CROSS_EXCHANGE_H
#define CROSS_EXCHANGE_H

/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "operator.h"

class cross_exchange : public ls_operator {
private:
  virtual void compute_gain() override;

  bool reverse_source_edge;
  bool reverse_target_edge;

public:
  cross_exchange(const input& input,
                 raw_solution& sol,
                 std::vector<amount_t>& amounts,
                 index_t source_vehicle,
                 index_t source_rank,
                 index_t target_vehicle,
                 index_t target_rank);

  virtual bool is_valid() const override;

  virtual void apply() const override;

  virtual void log() const override;
};

#endif
