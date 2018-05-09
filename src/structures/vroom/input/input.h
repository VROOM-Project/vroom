#ifndef INPUT_H
#define INPUT_H

/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <chrono>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "routing/routing_io.h"
#include "structures/abstract/matrix.h"
#include "structures/typedefs.h"
#include "structures/vroom/job.h"
#include "structures/vroom/solution/solution.h"
#include "structures/vroom/vehicle.h"

class vrp;

struct type_with_id {
  TYPE type;
  ID_t id;
};

class input {
private:
  std::chrono::high_resolution_clock::time_point _start_loading;
  std::chrono::high_resolution_clock::time_point _end_loading;
  std::chrono::high_resolution_clock::time_point _end_solving;
  std::chrono::high_resolution_clock::time_point _end_routing;
  std::unique_ptr<routing_io<cost_t>> _routing_wrapper;
  bool _has_skills;
  const bool _geometry;
  matrix<cost_t> _matrix;
  std::vector<location_t> _locations;
  unsigned _amount_size;
  std::vector<std::vector<bool>> _vehicle_to_job_compatibility;
  void check_amount_size(unsigned size);
  std::unique_ptr<vrp> get_problem() const;
  void check_cost_bound() const;
  void set_vehicle_to_job_compatibility();
  std::unordered_set<index_t> _matrix_used_index;
  bool _all_locations_have_coords;

  solution format_solution(const raw_solution& routes_as_list) const;

public:
  std::vector<job_t> _jobs;
  std::vector<vehicle_t> _vehicles;

  input(std::unique_ptr<routing_io<cost_t>> routing_wrapper, bool geometry);

  void add_job(const job_t& job);

  void add_vehicle(const vehicle_t& vehicle);

  void set_matrix(matrix<cost_t>&& m);

  const matrix<cost_t>& get_matrix() const;

  matrix<cost_t> get_sub_matrix(const std::vector<index_t>& indices) const;

  solution solve(unsigned nb_thread);

  friend class clustering;
};

#endif
