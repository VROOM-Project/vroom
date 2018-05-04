#ifndef RELOCATE_H
#define RELOCATE_H

/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "../../../structures/typedefs.h"
#include "../../../structures/vroom/input/input.h"

class relocate {
private:
  const input& _input;
  raw_solution& _sol;

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

  relocate(const input& input,
           raw_solution& sol,
           index_t source_vehicle,
           index_t source_rank,
           index_t target_vehicle,
           index_t target_rank);

  void log() const;
};

#endif
