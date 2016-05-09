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
#include "./matrix.h"
#include "./undirected_graph.h"
#include "../loaders/problem_io.h"

class tsp{
protected:
  matrix<distance_t> _matrix;
  matrix<distance_t> _symmetrized_matrix;
  undirected_graph<distance_t> _symmetrized_graph;
  bool _is_symmetric;
  const bool _force_start;
  const index_t _start;
  const bool _force_end;
  const index_t _end;

public:
  tsp(const problem_io<distance_t>& loader,
      bool force_start,
      index_t start,
      bool force_end,
      index_t end);
  
  const matrix<distance_t>& get_matrix() const;

  const matrix<distance_t>& get_symmetrized_matrix() const;

  const undirected_graph<distance_t>& get_symmetrized_graph() const;

  const bool is_symmetric() const;

  const bool force_start() const;

  const index_t get_start() const;

  const bool force_end() const;

  const index_t get_end() const;

  std::size_t size() const;

  distance_t cost(const std::list<index_t>& tour) const;

  distance_t symmetrized_cost(const std::list<index_t>& tour) const;
};

#endif
