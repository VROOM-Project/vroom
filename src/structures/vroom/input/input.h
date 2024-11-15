#ifndef INPUT_H
#define INPUT_H

/*

This file is part of VROOM.

Copyright (c) 2015-2024, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <chrono>
#include <memory>
#include <optional>
#include <unordered_map>

#include "routing/wrapper.h"
#include "structures/generic/matrix.h"
#include "structures/typedefs.h"
#include "structures/vroom/matrices.h"
#include "structures/vroom/solution/solution.h"
#include "structures/vroom/vehicle.h"

namespace vroom {

namespace io {
// Profile name used as key.
using Servers =
  std::unordered_map<std::string, Server, StringHash, std::equal_to<>>;
} // namespace io

class VRP;

class Input {
private:
  TimePoint _start_loading{std::chrono::high_resolution_clock::now()};
  TimePoint _end_loading;
  TimePoint _end_solving;
  TimePoint _end_routing;
  std::unordered_set<std::string> _profiles;
  std::vector<std::unique_ptr<routing::Wrapper>> _routing_wrappers;
  bool _apply_TSPFix;
  bool _no_addition_yet{true};
  bool _has_skills{false};
  bool _has_TW{false};
  bool _has_all_coordinates{true};
  bool _has_custom_location_index;
  bool _has_initial_routes{false};
  bool _homogeneous_locations{true};
  bool _homogeneous_profiles{true};
  bool _homogeneous_costs{true};
  bool _geometry{false};
  bool _report_distances;
  bool _has_jobs{false};
  bool _has_shipments{false};
  std::unordered_map<std::string,
                     Matrix<UserDuration>,
                     StringHash,
                     std::equal_to<>>
    _durations_matrices;
  std::unordered_map<std::string,
                     Matrix<UserDistance>,
                     StringHash,
                     std::equal_to<>>
    _distances_matrices;
  std::unordered_map<std::string, Matrix<UserCost>, StringHash, std::equal_to<>>
    _costs_matrices;
  std::unordered_map<std::string, Cost, StringHash, std::equal_to<>>
    _max_cost_per_hour;
  Cost _cost_upper_bound{0};
  std::vector<Location> _locations;
  std::unordered_map<Location, Index> _locations_to_index;
  std::unordered_set<Location> _locations_used_several_times;
  std::vector<std::vector<unsigned char>> _vehicle_to_job_compatibility;
  std::vector<std::vector<bool>> _vehicle_to_vehicle_compatibility;
  std::unordered_set<Index> _matrices_used_index;
  Index _max_matrices_used_index{0};
  bool _all_locations_have_coords{true};
  std::vector<std::vector<Eval>> _jobs_vehicles_evals;

  // Used in plan mode since we store route geometries while
  // generating sparse matrices.
  std::vector<std::string> _vehicles_geometry;

  std::optional<unsigned> _amount_size;
  Amount _zero;

  const io::Servers _servers;
  const ROUTER _router;

  std::unique_ptr<VRP> get_problem() const;

  void check_amount_size(const Amount& amount);
  void check_job(Job& job);

  void run_basic_checks() const;

  UserCost check_cost_bound(const Matrix<UserCost>& matrix) const;

  void set_skills_compatibility();
  void set_extra_compatibility();
  void set_vehicles_compatibility();
  void set_vehicles_costs();
  void set_vehicles_max_tasks();
  void set_jobs_vehicles_evals();
  void set_vehicle_steps_ranks();
  void init_missing_matrices(const std::string& profile);

  routing::Matrices get_matrices_by_profile(const std::string& profile,
                                            bool sparse_filling);

  void set_matrices(unsigned nb_thread, bool sparse_filling = false);

  void add_routing_wrapper(const std::string& profile);

public:
  std::vector<Job> jobs;
  std::vector<Vehicle> vehicles;

  // Store rank in jobs accessible from job/pickup/delivery id.
  std::unordered_map<Id, Index> job_id_to_rank;
  std::unordered_map<Id, Index> pickup_id_to_rank;
  std::unordered_map<Id, Index> delivery_id_to_rank;

  Input(io::Servers servers = {},
        ROUTER router = ROUTER::OSRM,
        bool apply_TSPFix = false);

  unsigned get_amount_size() const {
    assert(_amount_size.has_value());
    return _amount_size.value();
  }

  void set_geometry(bool geometry);

  void add_job(const Job& job);

  void add_shipment(const Job& pickup, const Job& delivery);

  void add_vehicle(const Vehicle& vehicle);

  void set_durations_matrix(const std::string& profile,
                            Matrix<UserDuration>&& m);

  void set_distances_matrix(const std::string& profile,
                            Matrix<UserDistance>&& m);

  void set_costs_matrix(const std::string& profile, Matrix<UserCost>&& m);

  const Amount& zero_amount() const {
    return _zero;
  }

  bool apply_TSPFix() const {
    return _apply_TSPFix;
  }

  bool is_used_several_times(const Location& location) const;

  bool has_skills() const;

  bool has_jobs() const;

  bool has_shipments() const;

  bool report_distances() const;

  Cost get_cost_upper_bound() const {
    return _cost_upper_bound;
  }

  bool all_locations_have_coords() const {
    return _all_locations_have_coords;
  }

  const std::vector<std::vector<Eval>>& jobs_vehicles_evals() const {
    return _jobs_vehicles_evals;
  }

  bool has_homogeneous_locations() const;

  bool has_homogeneous_profiles() const;

  bool has_homogeneous_costs() const;

  bool has_initial_routes() const;

  bool vehicle_ok_with_job(size_t v_index, size_t j_index) const {
    return static_cast<bool>(_vehicle_to_job_compatibility[v_index][j_index]);
  }

  // Returns true iff both vehicles have common job candidates.
  bool vehicle_ok_with_vehicle(Index v1_index, Index v2_index) const;

  Solution solve(unsigned nb_searches,
                 unsigned depth,
                 unsigned nb_thread,
                 const Timeout& timeout = Timeout(),
                 const std::vector<HeuristicParameters>& h_param =
                   std::vector<HeuristicParameters>());

  Solution check(unsigned nb_thread);
};

} // namespace vroom

#endif
