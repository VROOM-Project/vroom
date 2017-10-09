#ifndef TSP_H
#define TSP_H

/*

This file is part of VROOM.

Copyright (c) 2015-2017, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <fstream>
#include <iomanip>
#include <iostream>
#include <list>
#include <string>

#include "../../structures/abstract/undirected_graph.h"
#include "../vrp.h"
#include "./heuristics/christofides.h"
#include "./heuristics/local_search.h"

class tsp : public vrp {
private:
  index_t _vehicle_rank;
  // Holds the matching from index in _matrix to index in global
  // matrix.
  std::vector<index_t> _tsp_index_to_global;
  matrix<distance_t> _matrix;
  matrix<distance_t> _symmetrized_matrix;
  bool _is_symmetric;
  bool _has_start;
  index_t _start;
  bool _has_end;
  index_t _end;
  bool _round_trip;

public:
  tsp(const input &input,
      std::vector<index_t> problem_indices,
      index_t vehicle_rank);

  distance_t cost(const std::list<index_t>& tour) const;

  distance_t symmetrized_cost(const std::list<index_t>& tour) const;

  virtual solution solve(unsigned nb_threads) const override;
};

#endif
