#ifndef INPUT_H
#define INPUT_H

/*

This file is part of VROOM.

Copyright (c) 2015-2016, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <array>
#include <vector>
#include <unordered_map>
#include <boost/optional.hpp>
#include "../../typedefs.h"
#include "../../abstract/matrix.h"
#include "../job.h"
#include "../vehicle.h"
#include "../../../utils/exceptions.h"
#include "../../../routing/routing_io.h"
#include "../../../routing/routed_wrapper.h"
#if LIBOSRM
#include "../../../routing/libosrm_wrapper.h"
#endif

class vrp;

class input{
private:
  index_t _location_number;
  PROBLEM_T _problem_type;
  void set_routing(std::unique_ptr<routing_io<distance_t>> routing_wrapper);
  void set_matrix();
  std::unordered_map<index_t, index_t> _index_to_job_rank;

public:
  std::vector<job_t> _jobs;
  std::vector<vehicle> _vehicles;
  matrix<distance_t> _matrix;
  std::unique_ptr<routing_io<distance_t>> _routing_wrapper;
  // List of locations added through add_* matching the matrix
  // ordering.
  std::vector<location_t> _ordered_locations;

  input();

  void add_job(index_t id, const optional_coords_t& coords);

  void add_vehicle(index_t id,
                   const optional_coords_t& start_coords,
                   const optional_coords_t& end_coords);

  index_t get_location_number() const;

  // Retrieve the corresponding location from a matrix index.
  location_t get_location_at(index_t index) const;

  index_t get_job_rank_from_index(index_t index) const;

  PROBLEM_T get_problem_type() const;

  std::unique_ptr<vrp> get_problem() const;

  friend input parse(const cl_args_t& cl_args);
};

#endif
