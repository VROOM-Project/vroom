#ifndef RELOCATE_H
#define RELOCATE_H

/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "operator.h"

class relocate : public ls_operator {
private:
  virtual void compute_gain() override;

public:
  relocate(const input& input,
           raw_solution& sol,
           std::vector<amount_t>& amounts,
           index_t source_vehicle,
           index_t source_rank,
           index_t target_vehicle,
           index_t target_rank);

  virtual bool is_valid() const override;

  virtual void apply() const override;

  virtual void log() const override;

  virtual std::vector<index_t> addition_candidates() const override;
};

#endif
