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
#include "../loaders/tsplib_loader.h"
#include "../loaders/osrm_wrapper.h"

class tsp{
protected:
  std::unique_ptr<problem_io<distance_t>> _loader;
  matrix<distance_t> _matrix;
  matrix<distance_t> _symmetrized_matrix;
  undirected_graph<distance_t> _symmetrized_graph;
  bool _is_symmetric;
  const cl_args_t _cl_args;

public:
  tsp(const cl_args_t& cl_args);
  
  const matrix<distance_t>& get_matrix() const;

  const matrix<distance_t>& get_symmetrized_matrix() const;

  const undirected_graph<distance_t>& get_symmetrized_graph() const;

  const bool is_symmetric() const;

  std::size_t size() const;

  distance_t cost(const std::list<index_t>& tour) const;

  distance_t symmetrized_cost(const std::list<index_t>& tour) const;

  void get_route(const std::list<index_t>& tour,
                 rapidjson::Value& value,
                 rapidjson::Document::AllocatorType& allocator) const;

  void get_tour(const std::list<index_t>& tour,
                 rapidjson::Value& value,
                 rapidjson::Document::AllocatorType& allocator) const;

  void get_route_infos(const std::list<index_t>& tour,
                       rapidjson::Document& output) const;
};

#endif
