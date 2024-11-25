/*

This file is part of VROOM.

Copyright (c) 2015-2024, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <algorithm>
#include <mutex>
#include <thread>

#if USE_LIBOSRM
#include "osrm/exception.hpp"
#endif

#include "algorithms/validation/check.h"
#include "problems/cvrp/cvrp.h"
#include "problems/vrptw/vrptw.h"
#if USE_LIBOSRM
#include "routing/libosrm_wrapper.h"
#endif
#include "routing/ors_wrapper.h"
#include "routing/osrm_routed_wrapper.h"
#include "routing/valhalla_wrapper.h"
#include "structures/vroom/input/input.h"
#include "utils/helpers.h"

namespace vroom {

Input::Input(io::Servers servers, ROUTER router, bool apply_TSPFix)
  : _apply_TSPFix(apply_TSPFix), _servers(std::move(servers)), _router(router) {
}

void Input::set_geometry(bool geometry) {
  _geometry = geometry;
}

void Input::add_routing_wrapper(const std::string& profile) {
#if !USE_ROUTING
  throw RoutingException("VROOM compiled without routing support.");
#else

  if (!_has_all_coordinates) {
    throw InputException("Missing coordinates for routing engine.");
  }

  assert(std::find_if(_routing_wrappers.begin(),
                      _routing_wrappers.end(),
                      [&](const auto& wr) { return wr->profile == profile; }) ==
         _routing_wrappers.end());

  auto& routing_wrapper = _routing_wrappers.emplace_back();

  switch (_router) {
  case ROUTER::OSRM: {
    // Use osrm-routed.
    auto search = _servers.find(profile);
    if (search == _servers.end()) {
      throw InputException("Invalid profile: " + profile + ".");
    }
    routing_wrapper =
      std::make_unique<routing::OsrmRoutedWrapper>(profile, search->second);
  } break;
  case ROUTER::LIBOSRM:
#if USE_LIBOSRM
    // Use libosrm.
    try {
      routing_wrapper = std::make_unique<routing::LibosrmWrapper>(profile);
    } catch (const osrm::exception& e) {
      throw InputException("Invalid profile: " + profile + ".");
    }
    break;
#else
    // Attempt to use libosrm while compiling without it.
    throw RoutingException("VROOM compiled without libosrm installed.");
#endif
  case ROUTER::ORS: {
    // Use ORS http wrapper.
    auto search = _servers.find(profile);
    if (search == _servers.end()) {
      throw InputException("Invalid profile: " + profile + ".");
    }
    routing_wrapper =
      std::make_unique<routing::OrsWrapper>(profile, search->second);
  } break;
  case ROUTER::VALHALLA: {
    // Use Valhalla http wrapper.
    auto search = _servers.find(profile);
    if (search == _servers.end()) {
      throw InputException("Invalid profile: " + profile + ".");
    }
    routing_wrapper =
      std::make_unique<routing::ValhallaWrapper>(profile, search->second);
  } break;
  }
#endif
}

void Input::check_amount_size(const Amount& amount) {
  const auto size = amount.size();

  if (!_amount_size.has_value()) {
    // Only setup once on first call.
    _amount_size = size;
    _zero = Amount(size);
  } else {
    if (size != _amount_size.value()) {
      throw InputException(
        std::format("Inconsistent delivery length: {} instead of {}.",
                    size,
                    _amount_size.value()));
    }
  }
}

void Input::check_job(Job& job) {
  // Ensure delivery and pickup size consistency.
  check_amount_size(job.delivery);
  check_amount_size(job.pickup);

  // Ensure that location index are either always or never provided.
  bool has_location_index = job.location.user_index();
  if (_no_addition_yet) {
    _no_addition_yet = false;
    _has_custom_location_index = has_location_index;
  } else {
    if (_has_custom_location_index != has_location_index) {
      throw InputException("Missing location index.");
    }
  }

  // Check whether all locations have coordinates.
  _has_all_coordinates = _has_all_coordinates && job.location.has_coordinates();

  // Check for time-windows and skills.
  _has_TW = _has_TW || (!(job.tws.size() == 1) || !job.tws[0].is_default());
  _has_skills = _has_skills || !job.skills.empty();

  if (!job.location.user_index()) {
    // Index of job in the matrices is not specified in input, check
    // for already stored location or assign new index.
    auto search = _locations_to_index.find(job.location);
    if (search != _locations_to_index.end()) {
      // Using stored index for existing location.
      job.location.set_index(search->second);
      _locations_used_several_times.insert(job.location);
    } else {
      // Append new location and store corresponding index.
      auto new_index = _locations.size();
      job.location.set_index(new_index);
      _locations.push_back(job.location);
      _locations_to_index.try_emplace(job.location, new_index);
    }
  } else {
    // All jobs have a location_index in input, we only store
    // locations in case one profile matrix is not provided in input
    // and need to be computed.
    auto search = _locations_to_index.find(job.location);
    if (search == _locations_to_index.end()) {
      _locations.push_back(job.location);
      _locations_to_index.try_emplace(job.location, _locations.size() - 1);
    } else {
      _locations_used_several_times.insert(job.location);
    }
  }

  _matrices_used_index.insert(job.index());
  _max_matrices_used_index = std::max(_max_matrices_used_index, job.index());
  _all_locations_have_coords =
    _all_locations_have_coords && job.location.has_coordinates();
}

void Input::run_basic_checks() const {
  if (vehicles.empty()) {
    throw InputException("No vehicle defined.");
  }
  if (jobs.empty()) {
    throw InputException("No task defined.");
  }
  if (_geometry && !_all_locations_have_coords) {
    // Early abort when info is required with missing coordinates.
    throw InputException("Route geometry request with missing coordinates.");
  }
}

void Input::add_job(const Job& job) {
  if (job.type != JOB_TYPE::SINGLE) {
    throw InputException("Wrong job type.");
  }
  if (job_id_to_rank.contains(job.id)) {
    throw InputException(std::format("Duplicate job id: {}.", job.id));
  }
  job_id_to_rank[job.id] = jobs.size();
  jobs.push_back(job);
  check_job(jobs.back());
  _has_jobs = true;
}

void Input::add_shipment(const Job& pickup, const Job& delivery) {
  if (pickup.priority != delivery.priority) {
    throw InputException(
      std::
        format("Inconsistent shipment priority for pickup {} and delivery {}.",
               pickup.id,
               delivery.id));
  }
  if (!(pickup.pickup == delivery.delivery)) {
    throw InputException(
      std::format("Inconsistent shipment amount for pickup {} and delivery {}.",
                  pickup.id,
                  delivery.id));
  }
  if (pickup.skills.size() != delivery.skills.size()) {
    throw InputException(
      std::format("Inconsistent shipment skills for pickup {} and delivery {}.",
                  pickup.id,
                  delivery.id));
  }
  for (const auto s : pickup.skills) {
    if (!delivery.skills.contains(s)) {
      throw InputException(
        std::format("Inconsistent shipment skills for pickup {} and {}.",
                    pickup.id,
                    delivery.id));
    }
  }

  if (pickup.type != JOB_TYPE::PICKUP) {
    throw InputException(std::format("Wrong type for pickup {}.", pickup.id));
  }
  if (pickup_id_to_rank.contains(pickup.id)) {
    throw InputException(std::format("Duplicate pickup id: {}.", pickup.id));
  }
  pickup_id_to_rank[pickup.id] = jobs.size();
  jobs.push_back(pickup);
  check_job(jobs.back());

  if (delivery.type != JOB_TYPE::DELIVERY) {
    throw InputException(
      std::format("Wrong type for delivery {}.", delivery.id));
  }
  if (delivery_id_to_rank.contains(delivery.id)) {
    throw InputException(
      std::format("Duplicate delivery id: {}.", delivery.id));
  }
  delivery_id_to_rank[delivery.id] = jobs.size();
  jobs.push_back(delivery);
  check_job(jobs.back());
  _has_shipments = true;
}

void Input::add_vehicle(const Vehicle& vehicle) {
  vehicles.push_back(vehicle);

  auto& current_v = vehicles.back();

  // Ensure amount size consistency.
  check_amount_size(current_v.capacity);

  // Check for time-windows and skills.
  _has_TW = _has_TW || !vehicle.tw.is_default() || !vehicle.breaks.empty();
  _has_skills = _has_skills || !current_v.skills.empty();

  bool has_location_index = false;
  bool has_all_coordinates = true;
  if (current_v.has_start()) {
    auto& start_loc = current_v.start.value();

    has_location_index = start_loc.user_index();
    has_all_coordinates = start_loc.has_coordinates();

    if (!start_loc.user_index()) {
      // Index of start in the matrices is not specified in input,
      // check for already stored location or assign new index.
      assert(start_loc.has_coordinates());
      auto search = _locations_to_index.find(start_loc);
      if (search != _locations_to_index.end()) {
        // Using stored index for existing location.
        start_loc.set_index(search->second);
        _locations_used_several_times.insert(start_loc);
      } else {
        // Append new location and store corresponding index.
        auto new_index = _locations.size();
        start_loc.set_index(new_index);
        _locations.push_back(start_loc);
        _locations_to_index.try_emplace(start_loc, new_index);
      }
    } else {
      // All starts have a location_index in input, we only store
      // locations in case one profile matrix is not provided in input
      // and need to be computed.
      auto search = _locations_to_index.find(start_loc);
      if (search == _locations_to_index.end()) {
        _locations.push_back(start_loc);
        _locations_to_index.try_emplace(start_loc, _locations.size() - 1);
      } else {
        _locations_used_several_times.insert(start_loc);
      }
    }

    _matrices_used_index.insert(start_loc.index());
    _max_matrices_used_index =
      std::max(_max_matrices_used_index, start_loc.index());
    _all_locations_have_coords =
      _all_locations_have_coords && start_loc.has_coordinates();
  }

  if (current_v.has_end()) {
    auto& end_loc = current_v.end.value();

    if (current_v.has_start() && (has_location_index != end_loc.user_index())) {
      // Start and end provided in a non-consistent manner with regard
      // to location index definition.
      throw InputException("Missing start_index or end_index.");
    }

    has_location_index = end_loc.user_index();
    has_all_coordinates = has_all_coordinates && end_loc.has_coordinates();

    if (!end_loc.user_index()) {
      // Index of this end in the matrix was not specified upon
      // vehicle creation.
      assert(end_loc.has_coordinates());
      auto search = _locations_to_index.find(end_loc);
      if (search != _locations_to_index.end()) {
        // Using stored index for existing location.
        end_loc.set_index(search->second);
        _locations_used_several_times.insert(end_loc);
      } else {
        // Append new location and store corresponding index.
        auto new_index = _locations.size();
        end_loc.set_index(new_index);
        _locations.push_back(end_loc);
        _locations_to_index.try_emplace(end_loc, new_index);
      }
    } else {
      // All ends have a location_index in input, we only store
      // locations in case one profile matrix is not provided in input
      // and need to be computed.
      auto search = _locations_to_index.find(end_loc);
      if (search == _locations_to_index.end()) {
        _locations.push_back(end_loc);
        _locations_to_index.try_emplace(end_loc, _locations.size() - 1);
      } else {
        _locations_used_several_times.insert(end_loc);
      }
    }

    _matrices_used_index.insert(end_loc.index());
    _max_matrices_used_index =
      std::max(_max_matrices_used_index, end_loc.index());
    _all_locations_have_coords =
      _all_locations_have_coords && end_loc.has_coordinates();
  }

  // Ensure that location index are either always or never provided.
  if (_no_addition_yet) {
    _no_addition_yet = false;
    _has_custom_location_index = has_location_index;
  } else {
    if (_has_custom_location_index != has_location_index) {
      throw InputException("Missing location index.");
    }
  }

  // Check whether all locations have coordinates.
  _has_all_coordinates = _has_all_coordinates && has_all_coordinates;

  _has_initial_routes = _has_initial_routes || !current_v.steps.empty();

  // Check for homogeneous locations among vehicles.
  if (vehicles.size() > 1) {
    _homogeneous_locations =
      _homogeneous_locations &&
      vehicles.front().has_same_locations(vehicles.back());
    _homogeneous_profiles = _homogeneous_profiles &&
                            vehicles.front().has_same_profile(vehicles.back());
    _homogeneous_costs =
      _homogeneous_costs && vehicles.front().costs == vehicles.back().costs;
  }

  _profiles.insert(current_v.profile);

  auto search = _max_cost_per_hour.find(current_v.profile);
  if (search == _max_cost_per_hour.end()) {
    _max_cost_per_hour.try_emplace(current_v.profile, current_v.costs.per_hour);
  } else {
    search->second = std::max(search->second, current_v.costs.per_hour);
  }
}

void Input::set_durations_matrix(const std::string& profile,
                                 Matrix<UserDuration>&& m) {
  if (m.size() == 0) {
    throw InputException("Empty durations matrix for " + profile + " profile.");
  }
  _durations_matrices.insert_or_assign(profile, std::move(m));
}

void Input::set_distances_matrix(const std::string& profile,
                                 Matrix<UserDistance>&& m) {
  if (m.size() == 0) {
    throw InputException("Empty distances matrix for " + profile + " profile.");
  }
  _distances_matrices.insert_or_assign(profile, std::move(m));
}

void Input::set_costs_matrix(const std::string& profile, Matrix<UserCost>&& m) {
  if (m.size() == 0) {
    throw InputException("Empty costs matrix for " + profile + " profile.");
  }
  _costs_matrices.insert_or_assign(profile, std::move(m));
}

bool Input::is_used_several_times(const Location& location) const {
  return _locations_used_several_times.contains(location);
}

bool Input::has_skills() const {
  return _has_skills;
}

bool Input::has_jobs() const {
  return _has_jobs;
}

bool Input::has_shipments() const {
  return _has_shipments;
}

bool Input::report_distances() const {
  return _report_distances;
}

bool Input::has_homogeneous_locations() const {
  return _homogeneous_locations;
}

bool Input::has_homogeneous_profiles() const {
  return _homogeneous_profiles;
}

bool Input::has_homogeneous_costs() const {
  return _homogeneous_costs;
}

bool Input::has_initial_routes() const {
  return _has_initial_routes;
}

bool Input::vehicle_ok_with_vehicle(Index v1_index, Index v2_index) const {
  return _vehicle_to_vehicle_compatibility[v1_index][v2_index];
}

UserCost Input::check_cost_bound(const Matrix<UserCost>& matrix) const {
  // Check that we don't have any overflow while computing an upper
  // bound for solution cost.

  std::vector<UserCost> max_cost_per_line(matrix.size(), 0);
  std::vector<UserCost> max_cost_per_column(matrix.size(), 0);

  for (const auto i : _matrices_used_index) {
    for (const auto j : _matrices_used_index) {
      max_cost_per_line[i] = std::max(max_cost_per_line[i], matrix[i][j]);
      max_cost_per_column[j] = std::max(max_cost_per_column[j], matrix[i][j]);
    }
  }

  UserCost jobs_departure_bound = 0;
  UserCost jobs_arrival_bound = 0;
  for (const auto& j : jobs) {
    jobs_departure_bound =
      utils::add_without_overflow(jobs_departure_bound,
                                  max_cost_per_line[j.index()]);
    jobs_arrival_bound =
      utils::add_without_overflow(jobs_arrival_bound,
                                  max_cost_per_column[j.index()]);
  }

  UserCost jobs_bound = std::max(jobs_departure_bound, jobs_arrival_bound);

  UserCost start_bound = 0;
  UserCost end_bound = 0;
  for (const auto& v : vehicles) {
    if (v.has_start()) {
      start_bound =
        utils::add_without_overflow(start_bound,
                                    max_cost_per_line[v.start.value().index()]);
    }
    if (v.has_end()) {
      end_bound =
        utils::add_without_overflow(end_bound,
                                    max_cost_per_column[v.end.value().index()]);
    }
  }

  UserCost bound = utils::add_without_overflow(start_bound, jobs_bound);
  return utils::add_without_overflow(bound, end_bound);
}

void Input::set_skills_compatibility() {
  // Default to no restriction when no skills are provided.
  _vehicle_to_job_compatibility = std::vector<
    std::vector<unsigned char>>(vehicles.size(),
                                std::vector<unsigned char>(jobs.size(), true));
  if (_has_skills) {
    for (std::size_t v = 0; v < vehicles.size(); ++v) {
      const auto& v_skills = vehicles[v].skills;

      for (std::size_t j = 0; j < jobs.size(); ++j) {
        bool is_compatible = true;
        for (const auto& s : jobs[j].skills) {
          if (!v_skills.contains(s)) {
            is_compatible = false;
            break;
          }
        }
        _vehicle_to_job_compatibility[v][j] = is_compatible;
      }
    }
  }
}

void Input::set_extra_compatibility() {
  // Derive potential extra incompatibilities : jobs or shipments with
  // amount that does not fit into vehicle or that cannot be added to
  // an empty route for vehicle based on the timing constraints (when
  // they apply).
  for (std::size_t v = 0; v < vehicles.size(); ++v) {
    TWRoute empty_route(*this, v, _zero.size());
    for (Index j = 0; j < jobs.size(); ++j) {
      if (_vehicle_to_job_compatibility[v][j]) {
        bool is_compatible =
          empty_route.is_valid_addition_for_capacity(*this,
                                                     jobs[j].pickup,
                                                     jobs[j].delivery,
                                                     0);

        bool is_shipment_pickup = (jobs[j].type == JOB_TYPE::PICKUP);

        if (is_compatible && _has_TW) {
          if (jobs[j].type == JOB_TYPE::SINGLE) {
            is_compatible =
              is_compatible &&
              empty_route.is_valid_addition_for_tw_without_max_load(*this,
                                                                    j,
                                                                    0);
          } else {
            assert(is_shipment_pickup);
            std::vector<Index> p_d({j, static_cast<Index>(j + 1)});
            is_compatible =
              is_compatible && empty_route.is_valid_addition_for_tw(*this,
                                                                    _zero,
                                                                    p_d.begin(),
                                                                    p_d.end(),
                                                                    0,
                                                                    0);
          }
        }

        _vehicle_to_job_compatibility[v][j] = is_compatible;
        if (is_shipment_pickup) {
          // Skipping matching delivery which is next in line in jobs.
          _vehicle_to_job_compatibility[v][j + 1] = is_compatible;
          ++j;
        }
      }
    }
  }
}

void Input::set_vehicles_compatibility() {
  _vehicle_to_vehicle_compatibility =
    std::vector<std::vector<bool>>(vehicles.size(),
                                   std::vector<bool>(vehicles.size(), false));
  for (std::size_t v1 = 0; v1 < vehicles.size(); ++v1) {
    _vehicle_to_vehicle_compatibility[v1][v1] = true;
    for (std::size_t v2 = v1 + 1; v2 < vehicles.size(); ++v2) {
      for (std::size_t j = 0; j < jobs.size(); ++j) {
        if (_vehicle_to_job_compatibility[v1][j] &&
            _vehicle_to_job_compatibility[v2][j]) {
          _vehicle_to_vehicle_compatibility[v1][v2] = true;
          _vehicle_to_vehicle_compatibility[v2][v1] = true;
          break;
        }
      }
    }
  }
}

void Input::set_vehicles_costs() {
  for (auto& vehicle : vehicles) {
    auto duration_m = _durations_matrices.find(vehicle.profile);
    assert(duration_m != _durations_matrices.end());
    vehicle.cost_wrapper.set_durations_matrix(&(duration_m->second));

    auto distance_m = _distances_matrices.find(vehicle.profile);
    assert(distance_m != _distances_matrices.end());
    vehicle.cost_wrapper.set_distances_matrix(&(distance_m->second));

    auto c_m = _costs_matrices.find(vehicle.profile);
    if (c_m != _costs_matrices.end()) {
      // A custom cost matrix is provided for this vehicle.

      if (vehicle.costs.per_hour != DEFAULT_COST_PER_HOUR) {
        // Using a non-default "per-hour" value means defining costs
        // based on durations with a multiplicative factor. This is
        // inconsistent with providing a custom costs matrix.
        throw InputException(
          "Custom costs are incompatible with using a per_hour value.");
      }

      // Set plain custom costs matrix and reset cost factor.
      constexpr bool reset_cost_factor = true;
      vehicle.cost_wrapper.set_costs_matrix(&(c_m->second), reset_cost_factor);
    } else {
      vehicle.cost_wrapper.set_costs_matrix(&(duration_m->second));
    }
  }
}

void Input::set_vehicles_max_tasks() {
  if (const auto amount_size = get_amount_size();
      _has_jobs && !_has_shipments && amount_size > 0) {
    // For job-only instances where capacity restrictions apply:
    // compute an upper bound of the number of jobs for each vehicle
    // based on pickups load and delivery loads. This requires sorting
    // jobs and pickup/delivery values across all amount components.
    struct JobAmount {
      Index rank;
      Capacity amount;

      bool operator<(const JobAmount& rhs) const {
        return this->amount < rhs.amount;
      }
    };

    std::vector<std::vector<JobAmount>>
      job_pickups_per_component(amount_size,
                                std::vector<JobAmount>(jobs.size()));
    std::vector<std::vector<JobAmount>>
      job_deliveries_per_component(amount_size,
                                   std::vector<JobAmount>(jobs.size()));
    for (std::size_t i = 0; i < amount_size; ++i) {
      for (Index j = 0; j < jobs.size(); ++j) {
        job_pickups_per_component[i][j] = JobAmount({j, jobs[j].pickup[i]});
        job_deliveries_per_component[i][j] =
          JobAmount({j, jobs[j].delivery[i]});
      }

      std::sort(job_pickups_per_component[i].begin(),
                job_pickups_per_component[i].end());

      std::sort(job_deliveries_per_component[i].begin(),
                job_deliveries_per_component[i].end());
    }

    for (Index v = 0; v < vehicles.size(); ++v) {
      std::size_t max_tasks = jobs.size();

      for (std::size_t i = 0; i < amount_size; ++i) {
        Capacity pickup_sum = 0;
        Capacity delivery_sum = 0;
        std::size_t doable_pickups = 0;
        std::size_t doable_deliveries = 0;

        for (std::size_t j = 0; j < jobs.size(); ++j) {
          if (vehicle_ok_with_job(v, job_pickups_per_component[i][j].rank) &&
              pickup_sum <= vehicles[v].capacity[i]) {
            pickup_sum += job_pickups_per_component[i][j].amount;
            if (pickup_sum <= vehicles[v].capacity[i]) {
              ++doable_pickups;
            }
          }
          if (vehicle_ok_with_job(v, job_deliveries_per_component[i][j].rank) &&
              delivery_sum <= vehicles[v].capacity[i]) {
            delivery_sum += job_deliveries_per_component[i][j].amount;
            if (delivery_sum <= vehicles[v].capacity[i]) {
              ++doable_deliveries;
            }
          }
        }

        const auto doable_tasks = std::min(doable_pickups, doable_deliveries);
        max_tasks = std::min(max_tasks, doable_tasks);
      }

      vehicles[v].max_tasks = std::min(vehicles[v].max_tasks, max_tasks);
    }
  }

  if (_has_TW) {
    // Compute an upper bound of the number of tasks for each vehicle
    // based on time window amplitude and lower bounds of tasks times.
    struct JobTime {
      Index rank;
      Duration action;

      bool operator<(const JobTime& rhs) const {
        return this->action < rhs.action;
      }
    };

    std::vector<JobTime> job_times(jobs.size());
    for (Index j = 0; j < jobs.size(); ++j) {
      const auto action =
        jobs[j].service +
        (is_used_several_times(jobs[j].location) ? 0 : jobs[j].setup);
      job_times[j] = {j, action};
    }
    std::sort(job_times.begin(), job_times.end());

    for (Index v = 0; v < vehicles.size(); ++v) {
      auto& vehicle = vehicles[v];

      if (vehicle.tw.is_default()) {
        // No restriction will apply.
        continue;
      }

      const auto vehicle_duration = vehicle.available_duration();
      std::size_t doable_tasks = 0;
      Duration time_sum = 0;

      for (std::size_t j = 0; j < jobs.size(); ++j) {
        if (time_sum > vehicle_duration) {
          break;
        }
        if (vehicle_ok_with_job(v, job_times[j].rank)) {
          ++doable_tasks;
          time_sum += job_times[j].action;
        }
      }

      vehicle.max_tasks = std::min(vehicle.max_tasks, doable_tasks);
    }
  }
}

void Input::set_jobs_vehicles_evals() {
  // For a single job j, evals[j][v] evaluates fetching job j in an
  // empty route from vehicle at rank v. For a pickup job j,
  // evals[j][v] evaluates fetching job j **and** associated delivery
  // in an empty route from vehicle at rank v.
  _jobs_vehicles_evals =
    std::vector<std::vector<Eval>>(jobs.size(),
                                   std::vector<Eval>(vehicles.size(),
                                                     Eval(_cost_upper_bound)));

  for (std::size_t j = 0; j < jobs.size(); ++j) {
    Index j_index = jobs[j].index();
    bool is_pickup = (jobs[j].type == JOB_TYPE::PICKUP);

    Index last_job_index = j_index;
    if (is_pickup) {
      assert((j + 1 < jobs.size()) && (jobs[j + 1].type == JOB_TYPE::DELIVERY));
      last_job_index = jobs[j + 1].index();
    }

    for (std::size_t v = 0; v < vehicles.size(); ++v) {
      const auto& vehicle = vehicles[v];

      if (!vehicle_ok_with_job(v, j)) {
        continue;
      }

      auto& current_eval = _jobs_vehicles_evals[j][v];

      current_eval = is_pickup ? vehicle.eval(j_index, last_job_index) : Eval();
      if (vehicle.has_start()) {
        current_eval += vehicle.eval(vehicle.start.value().index(), j_index);
      }
      if (vehicle.has_end()) {
        current_eval +=
          vehicle.eval(last_job_index, vehicle.end.value().index());
      }

      if (is_pickup) {
        // Assign same eval to delivery.
        _jobs_vehicles_evals[j + 1][v] = current_eval;
      }
    }

    if (is_pickup) {
      // Skip delivery.
      ++j;
    }
  }
}

void Input::set_vehicle_steps_ranks() {
  std::unordered_set<Id> planned_job_ids;
  std::unordered_set<Id> planned_pickup_ids;
  std::unordered_set<Id> planned_delivery_ids;

  for (auto& current_vehicle : vehicles) {
    for (auto& step : current_vehicle.steps) {
      if (step.type == STEP_TYPE::BREAK) {
        auto search = current_vehicle.break_id_to_rank.find(step.id);
        if (search == current_vehicle.break_id_to_rank.end()) {
          throw InputException(
            std::format("Invalid break id {} for vehicle {}.",
                        step.id,
                        current_vehicle.id));
        }
        step.rank = search->second;
      }

      if (step.type == STEP_TYPE::JOB) {
        assert(step.job_type.has_value());
        switch (step.job_type.value()) {
        case JOB_TYPE::SINGLE: {
          auto search = job_id_to_rank.find(step.id);
          if (search == job_id_to_rank.end()) {
            throw InputException(
              std::format("Invalid job id {} for vehicle {}.",
                          step.id,
                          current_vehicle.id));
          }
          step.rank = search->second;

          if (planned_job_ids.contains(step.id)) {
            throw InputException(
              std::format("Duplicate job id {} in input steps for vehicle {}.",
                          step.id,
                          current_vehicle.id));
          }
          planned_job_ids.insert(step.id);
          break;
        }
        case JOB_TYPE::PICKUP: {
          auto search = pickup_id_to_rank.find(step.id);
          if (search == pickup_id_to_rank.end()) {
            throw InputException(
              std::format("Invalid pickup id {} for vehicle {}.",
                          step.id,
                          current_vehicle.id));
          }
          step.rank = search->second;

          if (planned_pickup_ids.contains(step.id)) {
            throw InputException(
              std::
                format("Duplicate pickup id {} in input steps for vehicle {}.",
                       step.id,
                       current_vehicle.id));
          }
          planned_pickup_ids.insert(step.id);
          break;
        }
        case JOB_TYPE::DELIVERY: {
          auto search = delivery_id_to_rank.find(step.id);
          if (search == delivery_id_to_rank.end()) {
            throw InputException(
              std::format("Invalid delivery id {} for vehicle {}.",
                          step.id,
                          current_vehicle.id));
          }
          step.rank = search->second;

          if (planned_delivery_ids.contains(step.id)) {
            throw InputException(
              std::format("Duplicate delivery id {} in input steps for vehicle "
                          "{}.",
                          step.id,
                          current_vehicle.id));
          }
          planned_delivery_ids.insert(step.id);
          break;
        }
        }
      }
    }
  }
}

void Input::init_missing_matrices(const std::string& profile) {
  // Even with custom matrices, we still need routing after
  // optimization if geometry is requested.
  bool create_routing_wrapper = _geometry;

  if (const auto durations_m = _durations_matrices.find(profile);
      durations_m == _durations_matrices.end()) {
    // No custom durations matrix.

    if (_distances_matrices.contains(profile)) {
      // We don't accept distances matrices without durations
      // matrices.
      throw InputException(
        "Custom matrix provided for distances but not for durations for " +
        profile + " profile.");
    }

    // No durations/distances matrices have been manually set,
    // create empty ones to allow for concurrent modification later
    // on.
    create_routing_wrapper = true;
    _durations_matrices.try_emplace(profile);
    _distances_matrices.try_emplace(profile);
  } else {
    // Custom durations matrix defined.
    if (!_distances_matrices.contains(profile)) {
      // No custom distances.
      if (_geometry) {
        // Get distances from routing engine later on since routing
        // is explicitly requested.
        _distances_matrices.try_emplace(profile);
      } else {
        // Routing-less optimization with no distances involved,
        // fill internal distances matrix with zeros.
        _distances_matrices.try_emplace(profile, durations_m->second.size(), 0);
      }
    }
  }

  if (create_routing_wrapper) {
    add_routing_wrapper(profile);
  }
}

routing::Matrices Input::get_matrices_by_profile(const std::string& profile,
                                                 bool sparse_filling) {
  auto rw = std::ranges::find_if(_routing_wrappers, [&](const auto& wr) {
    return wr->profile == profile;
  });
  assert(rw != _routing_wrappers.end());

  if (sparse_filling) {
    _vehicles_geometry.resize(vehicles.size());
  }

  // Note: get_sparse_matrices relies on getting in input *all*
  // vehicles as it refers to vehicle ranks to store geometries.
  return sparse_filling ? (*rw)->get_sparse_matrices(_locations,
                                                     this->vehicles,
                                                     this->jobs,
                                                     _vehicles_geometry)
                        : (*rw)->get_matrices(_locations);
}

void Input::set_matrices(unsigned nb_thread, bool sparse_filling) {
  if ((!_durations_matrices.empty() || !_distances_matrices.empty() ||
       !_costs_matrices.empty()) &&
      !_has_custom_location_index) {
    throw InputException("Missing location index.");
  }
  if ((_durations_matrices.empty() && _costs_matrices.empty()) &&
      _has_custom_location_index) {
    throw InputException(
      "Unexpected location index while no custom matrices provided.");
  }

  // Report distances either if geometry is explicitly requested, or
  // if distance matrices are manually provided.
  _report_distances = _geometry || !_distances_matrices.empty();

  if (!_distances_matrices.empty()) {
    // Distances matrices should be either always or never provided.
    for (const auto& profile : _profiles) {
      if (!_distances_matrices.contains(profile)) {
        throw InputException("Missing distances matrix for " + profile +
                             " profile.");
      }
    }
  }

  // Split computing matrices across threads based on number of
  // profiles.
  const auto nb_buckets =
    std::min(nb_thread, static_cast<unsigned>(_profiles.size()));

  std::vector<std::vector<std::string>>
    thread_profiles(nb_buckets, std::vector<std::string>());

  std::size_t t_rank = 0;
  for (const auto& profile : _profiles) {
    thread_profiles[t_rank % nb_buckets].push_back(profile);
    ++t_rank;

    init_missing_matrices(profile);
  }

  std::exception_ptr ep = nullptr;
  std::mutex ep_m;
  std::mutex cost_bound_m;

  auto run_on_profiles = [&](const std::vector<std::string>& profiles) {
    try {
      for (const auto& profile : profiles) {
        auto durations_m = _durations_matrices.find(profile);
        auto distances_m = _distances_matrices.find(profile);

        // Required matrices not manually set have been defined as
        // empty above in init_missing_matrices.
        assert(durations_m != _durations_matrices.end());
        assert(distances_m != _distances_matrices.end());
        const bool define_durations = (durations_m->second.size() == 0);
        const bool define_distances = (distances_m->second.size() == 0);
        assert(!define_durations || define_distances);

        if (define_durations || define_distances) {
          if (_locations.size() == 1) {
            durations_m->second = Matrix<UserDuration>(1);
            distances_m->second = Matrix<UserDistance>(1);
          } else {
            auto matrices = get_matrices_by_profile(profile, sparse_filling);

            if (!_has_custom_location_index) {
              // Location indices are set based on order in _locations.
              if (define_durations) {
                durations_m->second = std::move(matrices.durations);
              }
              if (define_distances) {
                distances_m->second = std::move(matrices.distances);
              }
            } else {
              // Location indices are provided in input so we need an
              // indirection based on order in _locations.
              if (define_durations) {
                Matrix<UserDuration> full_m(_max_matrices_used_index + 1);
                for (Index i = 0; i < _locations.size(); ++i) {
                  const auto& loc_i = _locations[i];
                  for (Index j = 0; j < _locations.size(); ++j) {
                    full_m[loc_i.index()][_locations[j].index()] =
                      matrices.durations[i][j];
                  }
                }

                durations_m->second = std::move(full_m);
              }
              if (define_distances) {
                Matrix<UserDistance> full_m(_max_matrices_used_index + 1);
                for (Index i = 0; i < _locations.size(); ++i) {
                  const auto& loc_i = _locations[i];
                  for (Index j = 0; j < _locations.size(); ++j) {
                    full_m[loc_i.index()][_locations[j].index()] =
                      matrices.distances[i][j];
                  }
                }

                distances_m->second = std::move(full_m);
              }
            }
          }
        }

        if (durations_m->second.size() <= _max_matrices_used_index) {
          throw InputException(
            "location_index exceeding durations matrix size for " + profile +
            " profile.");
        }

        if (distances_m->second.size() <= _max_matrices_used_index) {
          throw InputException(
            "location_index exceeding distances matrix size for " + profile +
            " profile.");
        }

        const auto c_m = _costs_matrices.find(profile);

        if (c_m != _costs_matrices.end()) {
          if (c_m->second.size() <= _max_matrices_used_index) {
            throw InputException(
              "location_index exceeding costs matrix size for " + profile +
              " profile.");
          }

          // Check for potential overflow in solution cost.
          const UserCost current_bound = check_cost_bound(c_m->second);
          std::scoped_lock<std::mutex> lock(cost_bound_m);
          _cost_upper_bound =
            std::max(_cost_upper_bound,
                     utils::scale_from_user_cost(current_bound));
        } else {
          // Durations matrix will be used for costs.
          const UserCost current_bound = check_cost_bound(durations_m->second);

          auto search = _max_cost_per_hour.find(profile);
          assert(search != _max_cost_per_hour.end());
          const auto max_cost_per_hour_for_profile = search->second;

          std::scoped_lock<std::mutex> lock(cost_bound_m);
          _cost_upper_bound =
            std::max(_cost_upper_bound,
                     max_cost_per_hour_for_profile *
                       utils::scale_from_user_duration(current_bound));
        }
      }
    } catch (...) {
      std::scoped_lock<std::mutex> lock(ep_m);
      ep = std::current_exception();
    }
  };

  std::vector<std::jthread> matrix_threads;
  matrix_threads.reserve(thread_profiles.size());

  for (const auto& profiles : thread_profiles) {
    matrix_threads.emplace_back(run_on_profiles, profiles);
  }

  for (auto& t : matrix_threads) {
    t.join();
  }

  if (ep != nullptr) {
    std::rethrow_exception(ep);
  }
}

std::unique_ptr<VRP> Input::get_problem() const {
  if (_has_TW) {
    return std::make_unique<VRPTW>(*this);
  }

  return std::make_unique<CVRP>(*this);
}

Solution Input::solve(unsigned nb_searches,
                      unsigned depth,
                      unsigned nb_thread,
                      const Timeout& timeout,
                      const std::vector<HeuristicParameters>& h_param) {
  run_basic_checks();

  if (_has_initial_routes) {
    set_vehicle_steps_ranks();
  }

  set_matrices(nb_thread);
  set_vehicles_costs();

  // Fill vehicle/job compatibility matrices.
  set_skills_compatibility();
  set_extra_compatibility();
  set_vehicles_compatibility();

  set_jobs_vehicles_evals();

  // Add implicit max_tasks constraints derived from capacity and
  // TW. Note: rely on set_extra_compatibility being run previously to
  // catch wrong breaks definition.
  set_vehicles_max_tasks();

  // Load relevant problem.
  auto instance = get_problem();
  _end_loading = std::chrono::high_resolution_clock::now();

  auto loading = std::chrono::duration_cast<std::chrono::milliseconds>(
    _end_loading - _start_loading);

  // Decide time allocated for solving, 0 means only heuristics will
  // be applied.
  Timeout solve_time;
  if (timeout.has_value()) {
    solve_time = (loading <= timeout.value()) ? (timeout.value() - loading)
                                              : std::chrono::milliseconds(0);
  }

  // Solve.
  auto sol =
    instance->solve(nb_searches, depth, nb_thread, solve_time, h_param);

  // Update timing info.
  sol.summary.computing_times.loading = loading.count();

  _end_solving = std::chrono::high_resolution_clock::now();
  sol.summary.computing_times.solving =
    std::chrono::duration_cast<std::chrono::milliseconds>(_end_solving -
                                                          _end_loading)
      .count();

  if (_geometry) {
    for (auto& route : sol.routes) {
      const auto& profile = route.profile;
      auto rw = std::ranges::find_if(_routing_wrappers, [&](const auto& wr) {
        return wr->profile == profile;
      });
      if (rw == _routing_wrappers.end()) {
        throw InputException(
          "Route geometry request with non-routable profile " + profile + ".");
      }
      (*rw)->add_geometry(route);
    }

    _end_routing = std::chrono::high_resolution_clock::now();
    auto routing = std::chrono::duration_cast<std::chrono::milliseconds>(
                     _end_routing - _end_solving)
                     .count();

    sol.summary.computing_times.routing = routing;
  }

  return sol;
}

Solution Input::check(unsigned nb_thread) {
#if USE_LIBGLPK
  run_basic_checks();

  set_vehicle_steps_ranks();

  constexpr bool sparse_filling = true;
  set_matrices(nb_thread, sparse_filling);
  set_vehicles_costs();

  // Fill basic skills compatibility matrix.
  set_skills_compatibility();

  _end_loading = std::chrono::high_resolution_clock::now();

  auto loading = std::chrono::duration_cast<std::chrono::milliseconds>(
                   _end_loading - _start_loading)
                   .count();

  // Check.
  std::unordered_map<Index, Index> route_rank_to_v_rank;
  auto sol =
    validation::check_and_set_ETA(*this, nb_thread, route_rank_to_v_rank);

  // Update timing info.
  sol.summary.computing_times.loading = loading;

  _end_solving = std::chrono::high_resolution_clock::now();
  sol.summary.computing_times.solving =
    std::chrono::duration_cast<std::chrono::milliseconds>(_end_solving -
                                                          _end_loading)
      .count();

  if (_geometry) {
    for (std::size_t i = 0; i < sol.routes.size(); ++i) {
      auto& route = sol.routes[i];

      auto search = route_rank_to_v_rank.find(i);
      assert(search != route_rank_to_v_rank.end());
      const auto v_rank = search->second;
      route.geometry = std::move(_vehicles_geometry[v_rank]);
    }

    _end_routing = std::chrono::high_resolution_clock::now();
    auto routing = std::chrono::duration_cast<std::chrono::milliseconds>(
                     _end_routing - _end_solving)
                     .count();

    sol.summary.computing_times.routing = routing;
  }

  return sol;
#else
  // Attempt to use libglpk while compiling without it.
  throw InputException("VROOM compiled without libglpk installed.");
  // Silence -Wunused-parameter warning.
  (void)nb_thread;
#endif
}

} // namespace vroom
