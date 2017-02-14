#ifndef TSP_H
#define TSP_H

/*

This file is part of VROOM.

Copyright (c) 2015-2017, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>
#include <list>
#include "../vrp.h"
#include "../../structures/abstract/undirected_graph.h"
#include "./heuristics/christofides.h"
#include "./heuristics/local_search.h"

class tsp: public vrp{
private:
  index_t _vehicle_rank;
  matrix<distance_t> _matrix;
  matrix<distance_t> _symmetrized_matrix;
  bool _is_symmetric;
  bool _force_start;
  index_t _start;
  bool _force_end;
  index_t _end;

public:
  tsp(const input& input,
      index_t vehicle_rank);

  distance_t cost(const std::list<index_t>& tour) const;

  distance_t symmetrized_cost(const std::list<index_t>& tour) const;

  virtual solution solve(unsigned nb_threads) const override;
};

#endif
