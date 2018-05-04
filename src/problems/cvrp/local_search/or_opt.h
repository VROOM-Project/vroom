#ifndef OR_OPT_H
#define OR_OPT_H

/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "../../../structures/typedefs.h"
#include "../../../structures/vroom/input/input.h"

class or_opt {
private:
  const input& _input;
  raw_solution& _sol;
  std::vector<amount_t>& _amounts;

  static gain_t compute_gain(const input& input,
                             const raw_solution& sol,
                             index_t source_vehicle,
                             index_t source_rank,
                             index_t target_vehicle,
                             index_t target_rank);

public:
  const index_t source_vehicle;
  const index_t source_rank;
  const index_t target_vehicle;
  const index_t target_rank;
  const gain_t gain;

  or_opt(const input& input,
         raw_solution& sol,
         std::vector<amount_t>& amounts,
         index_t source_vehicle,
         index_t source_rank,
         index_t target_vehicle,
         index_t target_rank);

  bool is_valid() const;

  void log() const;
};

#endif
