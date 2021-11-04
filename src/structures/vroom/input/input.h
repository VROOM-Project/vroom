#ifndef INPUT_H
#define INPUT_H

/*

This file is part of VROOM.

Copyright (c) 2015-2021, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <chrono>
#include <memory>
#include <unordered_map>

#include "routing/wrapper.h"
#include "structures/generic/matrix.h"
#include "structures/typedefs.h"
#include "structures/vroom/solution/solution.h"
#include "structures/vroom/vehicle.h"

namespace vroom {

namespace io {
// Profile name used as key.
using Servers = std::unordered_map<std::string, Server>;
} // namespace io

class VRP;

class Input {
private:
  TimePoint _start_loading;
  TimePoint _end_loading;
  TimePoint _end_solving;
  TimePoint _end_routing;
  std::unordered_set<std::string> _profiles;
  std::vector<std::unique_ptr<routing::Wrapper>> _routing_wrappers;
  bool _no_addition_yet;
  bool _has_skills;
  bool _has_TW;
  bool _has_custom_location_index;
  bool _has_initial_routes;
  bool _homogeneous_locations;
  bool _homogeneous_profiles;
  bool _geometry;
  bool _has_jobs;
  bool _has_shipments;
  std::unordered_map<std::string, Matrix<Duration>> _durations_matrices;
  std::unordered_map<std::string, Matrix<Duration>> _costs_matrices;
  Cost _cost_upper_bound;
  std::vector<Location> _locations;
  std::unordered_map<Location, Index> _locations_to_index;
  std::vector<std::vector<unsigned char>> _vehicle_to_job_compatibility;
  std::vector<std::vector<bool>> _vehicle_to_vehicle_compatibility;
  std::unordered_set<Index> _matrices_used_index;
  Index _max_matrices_used_index;
  bool _all_locations_have_coords;

  const unsigned _amount_size;
  const Amount _zero;

  const io::Servers _servers;
  const ROUTER _router;

  std::unique_ptr<VRP> get_problem() const;

  void check_job(Job& job);

  Cost check_cost_bound(const Matrix<Cost>& matrix) const;

  void set_skills_compatibility();
  void set_extra_compatibility();
  void set_vehicles_compatibility();
  void set_vehicles_costs();
  void set_vehicle_steps_ranks();
  void set_matrices(unsigned nb_thread);

  void add_routing_wrapper(const std::string& profile);

public:
  std::vector<Job> jobs;
  std::vector<Vehicle> vehicles;

  // Store rank in jobs accessible from job/pickup/delivery id.
  std::unordered_map<Id, Index> job_id_to_rank;
  std::unordered_map<Id, Index> pickup_id_to_rank;
  std::unordered_map<Id, Index> delivery_id_to_rank;

  Input(unsigned amount_size,
        const io::Servers& servers = {},
        ROUTER router = ROUTER::OSRM);

  void set_geometry(bool geometry);

  void add_job(const Job& job);

  void add_shipment(const Job& pickup, const Job& delivery);

  void add_vehicle(const Vehicle& vehicle);

  void set_durations_matrix(const std::string& profile, Matrix<Duration>&& m);
  void set_costs_matrix(const std::string& profile, Matrix<Cost>&& m);

  const Amount& zero_amount() const {
    return _zero;
  }

  bool has_skills() const;

  bool has_jobs() const;

  bool has_shipments() const;

  Cost get_cost_upper_bound() const {
    return _cost_upper_bound;
  }

  bool has_homogeneous_locations() const;

  bool has_homogeneous_profiles() const;

  bool vehicle_ok_with_job(size_t v_index, size_t j_index) const {
    return (bool)_vehicle_to_job_compatibility[v_index][j_index];
  }

  // Returns true iff both vehicles have common job candidates.
  bool vehicle_ok_with_vehicle(Index v1_index, Index v2_index) const;

  Solution solve(unsigned exploration_level,
                 unsigned nb_thread,
                 const Timeout& timeout = Timeout(),
                 const std::vector<HeuristicParameters>& h_param =
                   std::vector<HeuristicParameters>());

  Solution check(unsigned nb_thread);
};

} // namespace vroom

#endif
