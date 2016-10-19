#ifndef TSP_H
#define TSP_H

/*

This file is part of VROOM.

Copyright (c) 2015-2016, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <iostream>
#include <fstream>
#include <string>
#include <list>
#include "../../structures/typedefs.h"
#include "../../structures/abstract/matrix.h"
#include "../../structures/abstract/undirected_graph.h"
#include "../../structures/vroom/output/output.h"

class tsp: public vrp{
private:
  matrix<distance_t> _symmetrized_matrix;
  undirected_graph<distance_t> _symmetrized_graph;
  bool _is_symmetric;
  bool _force_start;
  index_t _start;
  bool _force_end;
  index_t _end;

public:
  tsp(const std::vector<job>& jobs,
      const std::vector<vehicle>& vehicles,
      matrix<distance_t> matrix):

  distance_t cost(const std::list<index_t>& tour) const;

  distance_t symmetrized_cost(const std::list<index_t>& tour) const;
};

#endif
