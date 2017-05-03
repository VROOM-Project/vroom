#ifndef INPUT_H
#define INPUT_H

/*

This file is part of VROOM.

Copyright (c) 2015-2017, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <array>
#include <chrono>
#include <unordered_map>
#include <vector>

#include <boost/optional.hpp>

#include "../../../problems/tsp/tsp.h"
#include "../../../routing/routed_wrapper.h"
#include "../../../routing/routing_io.h"
#include "../../../utils/exceptions.h"
#include "../../abstract/matrix.h"
#include "../../typedefs.h"
#include "../job.h"
#include "../vehicle.h"
#if LIBOSRM
#include "../../../routing/libosrm_wrapper.h"
#endif

class vrp;

class input {
private:
  std::chrono::high_resolution_clock::time_point _start_loading;
  std::chrono::high_resolution_clock::time_point _end_loading;
  std::chrono::high_resolution_clock::time_point _end_solving;
  std::chrono::high_resolution_clock::time_point _end_routing;
  index_t _location_number;
  PROBLEM_T _problem_type;
  std::unique_ptr<routing_io<distance_t>> _routing_wrapper;
  const bool _geometry;
  void set_matrix();
  std::unordered_map<index_t, index_t> _index_to_job_rank;
  std::unique_ptr<vrp> get_problem() const;

public:
  std::vector<job_t> _jobs;
  std::vector<vehicle> _vehicles;
  matrix<distance_t> _matrix;
  // List of locations added through add_* matching the matrix
  // ordering.
  std::vector<location_t> _ordered_locations;

  input(std::unique_ptr<routing_io<distance_t>> routing_wrapper, bool geometry);

  void add_job(index_t id, const optional_coords_t& coords);

  void add_job(index_t id,
               const optional_coords_t& coords,
               index_t index);

  void add_vehicle(index_t id,
                   const optional_coords_t& start_coords,
                   const optional_coords_t& end_coords);

  void add_vehicle(index_t id,
                   const optional_coords_t& start_coords,
                   const optional_coords_t& end_coords,
                   boost::optional<index_t> start_index,
                   boost::optional<index_t> end_index);

  index_t get_location_number() const;

  // Retrieve the corresponding location from a matrix index.
  location_t get_location_at(index_t index) const;

  index_t get_job_rank_from_index(index_t index) const;

  PROBLEM_T get_problem_type() const;

  solution solve(unsigned nb_thread);

  friend input parse(const cl_args_t& cl_args);
};

#endif
