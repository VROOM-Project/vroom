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
#include "./typedefs.h"
#include "./abstract/matrix.h"
#include "./abstract/undirected_graph.h"

class tsp{
protected:
  const pbl_context_t& _pbl_context;
  matrix<distance_t> _matrix;
  matrix<distance_t> _symmetrized_matrix;
  undirected_graph<distance_t> _symmetrized_graph;
  bool _is_symmetric;

public:
  tsp(const pbl_context_t& pbl_context, const matrix<distance_t>& matrix);

  const matrix<distance_t>& get_matrix() const;

  const matrix<distance_t>& get_symmetrized_matrix() const;

  const undirected_graph<distance_t>& get_symmetrized_graph() const;

  bool is_symmetric() const;

  bool force_start() const;

  index_t get_start() const;

  bool force_end() const;

  index_t get_end() const;

  std::size_t size() const;

  distance_t cost(const std::list<index_t>& tour) const;

  distance_t symmetrized_cost(const std::list<index_t>& tour) const;
};

#endif
