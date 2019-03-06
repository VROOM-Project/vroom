#ifndef INPUT_H
#define INPUT_H

/*

This file is part of VROOM.

Copyright (c) 2015-2019, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <chrono>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "routing/wrapper.h"
#include "structures/generic/matrix.h"
#include "structures/typedefs.h"
#include "structures/vroom/job.h"
#include "structures/vroom/solution/solution.h"
#include "structures/vroom/vehicle.h"

namespace vroom {

class VRP;

class Input {
private:
  std::chrono::high_resolution_clock::time_point _start_loading;
  std::chrono::high_resolution_clock::time_point _end_loading;
  std::chrono::high_resolution_clock::time_point _end_solving;
  std::chrono::high_resolution_clock::time_point _end_routing;
  std::unique_ptr<routing::Wrapper<Cost>> _routing_wrapper;
  bool _no_addition_yet;
  bool _has_skills;
  bool _has_TW;
  bool _homogeneous_locations;
  bool _geometry;
  Matrix<Cost> _matrix;
  std::vector<Location> _locations;
  std::unordered_map<Location, Index> _locations_to_index;
  unsigned _amount_size;
  Amount _amount_lower_bound;
  std::vector<std::vector<bool>> _vehicle_to_job_compatibility;
  std::vector<std::vector<bool>> _vehicle_to_vehicle_compatibility;
  std::unordered_set<Index> _matrix_used_index;
  bool _all_locations_have_coords;

  void check_amount_size(unsigned size);

  std::unique_ptr<VRP> get_problem() const;

  void check_cost_bound() const;

  void set_compatibility();

  void store_amount_lower_bound(const Amount& amount);

public:
  std::vector<Job> jobs;
  std::vector<Vehicle> vehicles;

  Input();

  void set_geometry(bool geometry);

  void set_routing(std::unique_ptr<routing::Wrapper<Cost>> routing_wrapper);

  void add_job(const Job& job);

  void add_vehicle(const Vehicle& vehicle);

  void set_matrix(Matrix<Cost>&& m);

  unsigned amount_size() const;

  Amount get_amount_lower_bound() const;

  bool has_skills() const;

  bool has_homogeneous_locations() const;

  bool vehicle_ok_with_job(Index v_index, Index j_index) const;

  // Returns true iff both vehicles have common job candidates.
  bool vehicle_ok_with_vehicle(Index v1_index, Index v2_index) const;

  const Matrix<Cost>& get_matrix() const;

  Matrix<Cost> get_sub_matrix(const std::vector<Index>& indices) const;

  Solution solve(unsigned exploration_level,
                 unsigned nb_thread,
                 const std::vector<HeuristicParameters>& h_param =
                   std::vector<HeuristicParameters>());
};

} // namespace vroom

#endif
