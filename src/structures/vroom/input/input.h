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

#include "routing/routing.h"
#include "structures/generic/matrix.h"
#include "structures/typedefs.h"
#include "structures/vroom/job.h"
#include "structures/vroom/solution/solution.h"
#include "structures/vroom/vehicle.h"

class VRP;

class Input {
private:
  std::chrono::high_resolution_clock::time_point _start_loading;
  std::chrono::high_resolution_clock::time_point _end_loading;
  std::chrono::high_resolution_clock::time_point _end_solving;
  std::chrono::high_resolution_clock::time_point _end_routing;
  std::unique_ptr<Routing<Cost>> _routing_wrapper;
  bool _has_skills;
  bool _has_TW;
  bool _homogeneous_locations;
  const bool _geometry;
  Matrix<Cost> _matrix;
  std::vector<Location> _locations;
  unsigned _amount_size;
  Amount _amount_lower_bound;
  std::vector<std::vector<bool>> _vehicle_to_job_compatibility;
  void check_amount_size(unsigned size);
  std::unique_ptr<VRP> get_problem() const;
  void check_cost_bound() const;
  void set_vehicle_to_job_compatibility();
  std::unordered_set<Index> _matrix_used_index;
  bool _all_locations_have_coords;

  void store_amount_lower_bound(const Amount& amount);

public:
  std::vector<Job> _jobs;
  std::vector<Vehicle> _vehicles;

  Input(std::unique_ptr<Routing<Cost>> routing_wrapper, bool geometry);

  void add_job(const Job& job);

  void add_vehicle(const Vehicle& vehicle);

  void set_matrix(Matrix<Cost>&& m);

  unsigned amount_size() const;

  Amount get_amount_lower_bound() const;

  bool has_skills() const;

  bool has_homogeneous_locations() const;

  bool vehicle_ok_with_job(Index v_index, Index j_index) const;

  const Matrix<Cost>& get_matrix() const;

  Matrix<Cost> get_sub_matrix(const std::vector<Index>& indices) const;

  Solution solve(unsigned exploration_level, unsigned nb_thread);
};

#endif
