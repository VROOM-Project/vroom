/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <algorithm>
#include <mutex>
#include <semaphore>
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
#include "utils/budget_repair.h"

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
  const bool has_location_index = job.location.user_index();
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
  // Record per-vehicle objective penalties for this job.
  std::vector<Cost> penalties(vehicles.size(), 0);
  for (const auto& [vid, pen] : jobs.back().vehicle_penalties) {
    const auto search = _vehicle_id_to_rank.find(vid);
    if (search == _vehicle_id_to_rank.end()) {
      throw InputException(
        std::format("Unknown vehicle id {} in vehicle_penalties for job {}.",
                    vid,
                    jobs.back().id));
    }
    penalties[search->second] = pen;
  }
  _job_vehicle_penalties.push_back(std::move(penalties));
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
  // Record penalties for pickup (shipment penalties apply once, on pickup only).
  {
    std::vector<Cost> penalties(vehicles.size(), 0);
    for (const auto& [vid, pen] : jobs.back().vehicle_penalties) {
      const auto search = _vehicle_id_to_rank.find(vid);
      if (search == _vehicle_id_to_rank.end()) {
        throw InputException(
          std::format(
            "Unknown vehicle id {} in vehicle_penalties for pickup {}.",
            vid,
            jobs.back().id));
      }
      penalties[search->second] = pen;
    }
    _job_vehicle_penalties.push_back(std::move(penalties));
  }

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
  // No penalties on delivery (counted once on pickup).
  _job_vehicle_penalties.push_back(std::vector<Cost>(vehicles.size(), 0));
  _has_shipments = true;
}

void Input::add_vehicle(const Vehicle& vehicle) {
  if (_vehicle_id_to_rank.contains(vehicle.id)) {
    throw InputException(
      std::format("Duplicate vehicle id: {}.", vehicle.id));
  }
  _vehicle_id_to_rank[vehicle.id] = vehicles.size();
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
  if (current_v.costs.per_km != 0) {
    _profiles_requiring_distances.insert(current_v.profile);
  }
  // First-leg distance limit requires distances for the relevant profile.
  if (current_v.has_start() && current_v.steps.empty() &&
      current_v.max_first_leg_distance != DEFAULT_MAX_DISTANCE) {
    _profiles_requiring_distances.insert(current_v.profile);
  }

  if (auto search = _max_cost_per_hour.find(current_v.profile);
      search == _max_cost_per_hour.end()) {
    _max_cost_per_hour.try_emplace(current_v.profile, current_v.costs.per_hour);
  } else {
    search->second = std::max(search->second, current_v.costs.per_hour);
  }

  // Store vehicle type stuff.
  const auto& type = current_v.type_str;
  if (auto search = _type_to_rank_in_vehicle_types.find(type);
      search != _type_to_rank_in_vehicle_types.end()) {
    // Already known type, only set vehicle type with known index.
    current_v.type = search->second;
  } else {
    const Index rank = _vehicle_types.size();
    [[maybe_unused]] const auto insertion_result =
      _type_to_rank_in_vehicle_types.try_emplace(type, rank);
    assert(insertion_result.second);
    _vehicle_types.push_back(type);
    current_v.type = rank;
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

  const UserCost jobs_bound =
    std::max(jobs_departure_bound, jobs_arrival_bound);

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

  const UserCost bound = utils::add_without_overflow(start_bound, jobs_bound);
  return utils::add_without_overflow(bound, end_bound);
}

void Input::set_skills_compatibility() {
  // Start with all compatible, then apply filters.
  _vehicle_to_job_compatibility = std::vector<
    std::vector<unsigned char>>(vehicles.size(),
                                std::vector<unsigned char>(jobs.size(), true));

  // Ensure pinned vector sized
  _pinned_vehicle_by_job.resize(jobs.size());

  for (std::size_t v = 0; v < vehicles.size(); ++v) {
    const auto& v_skills = vehicles[v].skills;

    for (std::size_t j = 0; j < jobs.size(); ++j) {
      bool is_compatible = true;

      // 1) Pinned filter: if job pinned to a specific vehicle, only allow that one
      if (_pinned_vehicle_by_job[j].has_value()) {
        is_compatible = (v == _pinned_vehicle_by_job[j].value());
      }

      // 2) allowed_vehicles filter regardless of _has_skills.
      if (is_compatible && !jobs[j].allowed_vehicles.empty()) {
        const auto v_id = vehicles[v].id;
        is_compatible = std::ranges::find(jobs[j].allowed_vehicles, v_id) !=
                        jobs[j].allowed_vehicles.end();
      }
      if (!is_compatible) {
        _vehicle_to_job_compatibility[v][j] = false;
        continue;
      }

      // 3) skills inclusion.
      if (is_compatible && _has_skills) {
        for (const auto& s : jobs[j].skills) {
          if (!v_skills.contains(s)) {
            is_compatible = false;
            break;
          }
        }
      }

      _vehicle_to_job_compatibility[v][j] = is_compatible;
    }
  }
}

void Input::set_extra_compatibility() {
  // Derive potential extra incompatibilities : jobs or shipments with
  // amount that does not fit into vehicle or that cannot be added to
  // an empty route for vehicle based on the timing constraints (when
  // they apply).
  compatible_vehicles_for_job = std::vector<std::vector<Index>>(jobs.size());

  for (std::size_t v = 0; v < vehicles.size(); ++v) {
    TWRoute probe_route(*this, v, _zero.size());
    Index insert_rank = 0;
    if (const auto pf = pinned_first_for_vehicle(v); pf.has_value()) {
      std::vector<Index> pinned_first;
      const auto& req = pf.value();
      if (req.job_rank.has_value()) {
        pinned_first.push_back(req.job_rank.value());
      } else if (req.pickup_rank.has_value() && req.delivery_rank.has_value()) {
        pinned_first.push_back(req.pickup_rank.value());
        pinned_first.push_back(req.delivery_rank.value());
      }
      if (!pinned_first.empty()) {
        probe_route.seed_relaxed_from_job_ranks(*this, _zero, pinned_first);
        insert_rank = static_cast<Index>(probe_route.route.size());
      }
    }
    for (Index j = 0; j < jobs.size(); ++j) {
      if (!_vehicle_to_job_compatibility[v][j]) {
        continue;
      }

      bool is_compatible =
        probe_route.is_valid_addition_for_capacity(*this,
                                                   jobs[j].pickup,
                                                   jobs[j].delivery,
                                                   insert_rank);

      const bool is_shipment_pickup = (jobs[j].type == JOB_TYPE::PICKUP);

      if (is_compatible && _has_TW) {
        if (jobs[j].type == JOB_TYPE::SINGLE) {
          is_compatible =
            is_compatible &&
            probe_route.is_valid_addition_for_tw_without_max_load(*this, j, insert_rank);
        } else {
          assert(is_shipment_pickup);
          std::vector<Index> p_d({j, static_cast<Index>(j + 1)});
          is_compatible =
            is_compatible && probe_route.is_valid_addition_for_tw(*this,
                                                                  _zero,
                                                                  p_d.begin(),
                                                                  p_d.end(),
                                                                  insert_rank,
                                                                  insert_rank);
        }
      }

      // Removed per-job budget compatibility prefilter: budgets are enforced
      // at the route level during insertions.

      _vehicle_to_job_compatibility[v][j] = is_compatible;
      if (is_shipment_pickup) {
        // Skipping matching delivery which is next in line in jobs.
        _vehicle_to_job_compatibility[v][j + 1] = is_compatible;
        ++j;
      }

      if (is_compatible) {
        compatible_vehicles_for_job[j].push_back(v);
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
    };

    // Store jobs ordered by increasing action time per vehicle type.
    std::vector<std::vector<JobTime>> job_times_per_type(_vehicle_types.size(),
                                                         std::vector<JobTime>(
                                                           jobs.size()));
    for (Index t = 0; t < _vehicle_types.size(); ++t) {
      for (Index j = 0; j < jobs.size(); ++j) {
        const auto action =
          jobs[j].services[t] +
          (is_used_several_times(jobs[j].location) ? 0 : jobs[j].setups[t]);
        job_times_per_type[t][j] = {j, action};
      }
      std::ranges::sort(job_times_per_type[t],
                        std::ranges::less{},
                        &JobTime::action);
    }

    for (Index v = 0; v < vehicles.size(); ++v) {
      auto& vehicle = vehicles[v];

      if (vehicle.tw.is_default()) {
        // No restriction will apply.
        continue;
      }

      const auto vehicle_duration = vehicle.available_duration();
      const auto t = vehicle.type;
      std::size_t doable_tasks = 0;
      Duration time_sum = 0;

      for (std::size_t j = 0; j < jobs.size(); ++j) {
        if (vehicle_ok_with_job(v, job_times_per_type[t][j].rank)) {
          time_sum += job_times_per_type[t][j].action;

          if (time_sum <= vehicle_duration) {
            ++doable_tasks;
          } else {
            break;
          }
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
    const Index j_index = jobs[j].index();
    const bool is_pickup = (jobs[j].type == JOB_TYPE::PICKUP);

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

      // Apply optional per-(job,vehicle) objective penalty. For shipments, this
      // is stored on pickup only and eval is shared with delivery.
      current_eval.cost += job_vehicle_penalty(j, v);

      // Enforce optional first-leg distance bound for empty routes only:
      // reject candidates that would exceed the start -> first job limit.
      if (vehicle.has_start() && vehicle.steps.empty() &&
          vehicle.max_first_leg_distance != DEFAULT_MAX_DISTANCE) {
        const auto first_leg_distance =
          vehicle.eval(vehicle.start.value().index(), j_index).distance;
        if (first_leg_distance > vehicle.max_first_leg_distance) {
          current_eval = NO_EVAL;
        }
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

void Input::set_jobs_durations_per_vehicle_type() {
  const auto nb_types = _vehicle_types.size();

  for (auto& job : jobs) {
    // Populate duration vectors with default values at first.
    job.setups = std::vector<Duration>(nb_types, job.default_setup);
    job.services = std::vector<Duration>(nb_types, job.default_service);

    // Iterate on all user-defined vehicle types to override relevant
    // setup and service values.
    for (std::size_t type_rank = 1; type_rank < nb_types; ++type_rank) {
      const auto& type = _vehicle_types[type_rank];

      if (const auto search = job.setup_per_type.find(type);
          search != job.setup_per_type.end()) {
        job.setups[type_rank] = search->second;
      }

      if (const auto search = job.service_per_type.find(type);
          search != job.service_per_type.end()) {
        job.services[type_rank] = search->second;
      }
    }
  }
}

void Input::set_vehicle_steps_ranks() {
  // Ensure pinned vector is sized before we record pinned vehicles
  if (_pinned_vehicle_by_job.size() != jobs.size()) {
    _pinned_vehicle_by_job = std::vector<std::optional<Index>>(jobs.size());
  }

  std::unordered_set<Id> planned_job_ids;
  std::unordered_set<Id> planned_pickup_ids;
  std::unordered_set<Id> planned_delivery_ids;

  // Track pinned presence: each pinned job must be present exactly once in exactly one vehicle
  std::vector<unsigned> pinned_appearances(jobs.size(), 0);

  // Prepare boundary anchors containers sized by vehicles
  _pinned_first_by_vehicle = std::vector<std::optional<PinnedBoundaryRequirement>>(vehicles.size());
  _pinned_last_by_vehicle = std::vector<std::optional<PinnedBoundaryRequirement>>(vehicles.size());

  for (auto& current_vehicle : vehicles) {
    const Index v_rank = &current_vehicle - &vehicles[0];
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

          // If pinned flag on this job, record pinned vehicle
          if (jobs[step.rank].pinned) {
            ++pinned_appearances[step.rank];
            if (_pinned_vehicle_by_job[step.rank].has_value() &&
                _pinned_vehicle_by_job[step.rank].value() != v_rank) {
              throw InputException(std::format(
                "Pinned task {} appears in multiple vehicle.steps.", step.id));
            }
            _pinned_vehicle_by_job[step.rank] = v_rank;

            // Record first/last anchors for single jobs
            const auto pos = jobs[step.rank].pinned_position;
            if (pos != PinnedPosition::NONE) {
              PinnedBoundaryRequirement req;
              req.job_rank = step.rank;
              auto& container =
                (pos == PinnedPosition::FIRST) ? _pinned_first_by_vehicle
                                               : _pinned_last_by_vehicle;
              if (container[v_rank].has_value()) {
                throw InputException(std::format(
                  "Multiple pinned_position '{}' constraints for vehicle {}.",
                  (pos == PinnedPosition::FIRST ? "first" : "last"),
                  current_vehicle.id));
              }
              container[v_rank] = req;
            }
          }
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

          if (jobs[step.rank].pinned) {
            ++pinned_appearances[step.rank];
            if (_pinned_vehicle_by_job[step.rank].has_value() &&
                _pinned_vehicle_by_job[step.rank].value() != v_rank) {
              throw InputException(std::format(
                "Pinned shipment pickup {} appears in multiple vehicle.steps.", step.id));
            }
            _pinned_vehicle_by_job[step.rank] = v_rank;
          }
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

          if (jobs[step.rank].pinned) {
            ++pinned_appearances[step.rank];
            if (_pinned_vehicle_by_job[step.rank].has_value() &&
                _pinned_vehicle_by_job[step.rank].value() != v_rank) {
              throw InputException(std::format(
                "Pinned shipment delivery {} appears in multiple vehicle.steps.", step.id));
            }
            _pinned_vehicle_by_job[step.rank] = v_rank;
          }
          break;
        }
        }
      }
    }
  }

  // Validate pinned presence and shipment steps on same vehicle
  for (Index j = 0; j < jobs.size(); ++j) {
    if (!jobs[j].pinned) {
      continue;
    }
    if (!pinned_appearances[j]) {
      const auto& job = jobs[j];
      const auto type_str = (job.type == JOB_TYPE::SINGLE)
                              ? "job"
                              : (job.type == JOB_TYPE::PICKUP ? "pickup"
                                                              : "delivery");
      throw InputException(std::format(
        "Pinned task {} ({} ) must be included in exactly one vehicle.steps.",
        job.id,
        type_str));
    }
    if (pinned_appearances[j] > 1) {
      const auto& job = jobs[j];
      throw InputException(std::format(
        "Pinned task {} appears multiple times in vehicle.steps.", job.id));
    }

    // For shipments, ensure pickup and delivery are pinned to same vehicle
    if (jobs[j].type == JOB_TYPE::PICKUP) {
      const Index d = j + 1;
      if (!jobs[d].pinned ||
          !_pinned_vehicle_by_job[j].has_value() ||
          !_pinned_vehicle_by_job[d].has_value() ||
          _pinned_vehicle_by_job[j].value() != _pinned_vehicle_by_job[d].value()) {
        throw InputException(std::format(
          "Pinned shipment steps {} and {} must be under the same vehicle steps.",
          jobs[j].id,
          jobs[d].id));
      }

      // If shipment has pinned_position, record boundary requirement
      const auto pos = jobs[j].pinned_position;
      if (pos != PinnedPosition::NONE || jobs[d].pinned_position != PinnedPosition::NONE) {
        // Both should match if provided
        if (jobs[d].pinned_position != pos) {
          throw InputException(std::format(
            "Pinned shipment steps {} and {} must share the same pinned_position.",
            jobs[j].id,
            jobs[d].id));
        }
        const auto v = _pinned_vehicle_by_job[j].value();
        PinnedBoundaryRequirement req;
        req.pickup_rank = j;
        req.delivery_rank = d;
        auto& container = (pos == PinnedPosition::FIRST) ? _pinned_first_by_vehicle
                                                         : _pinned_last_by_vehicle;
        if (container[v].has_value()) {
          throw InputException(std::format(
            "Multiple pinned_position '{}' constraints for vehicle {}.",
            (pos == PinnedPosition::FIRST ? "first" : "last"),
            vehicles[v].id));
        }
        container[v] = req;
      }
    }

    // Check allowed_vehicles conflict: pinned vehicle must be eligible
    if (!jobs[j].allowed_vehicles.empty()) {
      const auto pinned_v = _pinned_vehicle_by_job[j].value();
      const auto pinned_v_id = vehicles[pinned_v].id;
      if (std::ranges::find(jobs[j].allowed_vehicles, pinned_v_id) ==
          jobs[j].allowed_vehicles.end()) {
        throw InputException(std::format(
          "Pinned task {} conflicts with allowed_vehicles (vehicle {}).",
          jobs[j].id,
          pinned_v_id));
      }
    }
  }
}

void Input::init_exclusive_tags() {
  _exclusive_tag_to_rank.clear();
  _exclusive_tag_ids_by_job_rank.clear();
  _exclusive_tag_ids_by_job_rank.resize(jobs.size());
  _exclusive_tag_pinned_counts_by_vehicle.clear();

  if (jobs.empty()) {
    return;
  }

  const bool can_check_pinned =
    (_pinned_vehicle_by_job.size() == jobs.size()) && !vehicles.empty();
  std::vector<std::unordered_map<ExclusiveTag, Id>> pinned_tag_to_task_id;
  if (can_check_pinned) {
    pinned_tag_to_task_id.resize(vehicles.size());
  }

  for (Index j = 0; j < jobs.size(); ++j) {
    const auto& job = jobs[j];
    if (job.exclusive_tags.empty()) {
      continue;
    }

    // Ensure no duplicates within a single task.
    std::vector<ExclusiveTag> unique_tags = job.exclusive_tags;
    std::ranges::sort(unique_tags);
    for (std::size_t i = 1; i < unique_tags.size(); ++i) {
      if (unique_tags[i] == unique_tags[i - 1]) {
        throw InputException(std::format(
          "Duplicate exclusive tag {} for task {}.",
          unique_tags[i],
          job.id));
      }
    }

    // Reject pinned conflicts: solver can't fix two pinned tasks with same tag
    // on the same vehicle.
    if (!_exclusive_tags_allow_pinned_conflicts &&
        can_check_pinned && _pinned_vehicle_by_job[j].has_value()) {
      const auto v_rank = _pinned_vehicle_by_job[j].value();
      assert(v_rank < vehicles.size());
      for (const auto t : unique_tags) {
        auto& m = pinned_tag_to_task_id[v_rank];
        if (const auto it = m.find(t); it != m.end()) {
          throw InputException(std::format(
            "Pinned tasks {} and {} share exclusive tag {} on vehicle {}.",
            it->second,
            job.id,
            t,
            vehicles[v_rank].id));
        }
        m.emplace(t, job.id);
      }
    }

    // Normalize to compact ids.
    auto& out = _exclusive_tag_ids_by_job_rank[j];
    out.reserve(unique_tags.size());
    for (const auto t : unique_tags) {
      auto it = _exclusive_tag_to_rank.find(t);
      if (it == _exclusive_tag_to_rank.end()) {
        if (_exclusive_tag_to_rank.size() >
            static_cast<std::size_t>(std::numeric_limits<Index>::max())) {
          throw InputException("Too many exclusive tags, stopping.");
        }
        const Index next = static_cast<Index>(_exclusive_tag_to_rank.size());
        it = _exclusive_tag_to_rank.emplace(t, next).first;
      }
      out.push_back(it->second);
    }
  }

  // Compute per-vehicle pinned counts per normalized tag.
  _exclusive_tag_pinned_counts_by_vehicle.assign(
    vehicles.size(),
    std::vector<unsigned short>(exclusive_tag_count(), 0));
  if (can_check_pinned && exclusive_tag_count() > 0) {
    for (Index j = 0; j < jobs.size(); ++j) {
      if (!jobs[j].pinned || !_pinned_vehicle_by_job[j].has_value()) {
        continue;
      }
      const auto v_rank = _pinned_vehicle_by_job[j].value();
      for (const auto tid : _exclusive_tag_ids_by_job_rank[j]) {
        _exclusive_tag_pinned_counts_by_vehicle[v_rank][tid] += 1;
      }
    }
  }

  // When allowing pinned conflicts, we still need pinned counts to define
  // per-route limits. When not allowing, the earlier strict check already
  // rejected duplicates.
}

void Input::enforce_pinned_eligibility() {
  if (_pinned_vehicle_by_job.empty() || !_pinned_soft_timing) {
    return;
  }
  for (Index j = 0; j < jobs.size(); ++j) {
    if (_pinned_vehicle_by_job[j].has_value()) {
      const auto v_rank = _pinned_vehicle_by_job[j].value();
      _vehicle_to_job_compatibility[v_rank][j] = true;
      auto& list = compatible_vehicles_for_job[j];
      if (std::ranges::find(list, v_rank) == list.end()) {
        list.push_back(v_rank);
      }
    }
  }
}

void Input::validate_first_leg_limits(Solution&) const {
  // No-op: rank-0 violations are already prevented at seed and insertion time,
  // so the final solution should be valid without post-pass mutations.
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
      if (_geometry || _profiles_requiring_distances.contains(profile)) {
        // Get distances from routing engine later on since routing is
        // explicitly requested, or distances should be used in
        // optimization objective.
        create_routing_wrapper = true;
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
  if ((_durations_matrices.empty() && _distances_matrices.empty() &&
       _costs_matrices.empty()) &&
      _has_custom_location_index) {
    throw InputException(
      "Unexpected location index while no custom matrices provided.");
  }

  // Report distances either if geometry is explicitly requested, or
  // if distance matrices are manually provided or required in
  // optimization objective.
  _report_distances = _geometry || !_distances_matrices.empty() ||
                      !_profiles_requiring_distances.empty();

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
          const std::scoped_lock<std::mutex> lock(cost_bound_m);
          _cost_upper_bound =
            std::max(_cost_upper_bound,
                     utils::scale_from_user_cost(current_bound));
        } else {
          // Durations matrix will be used for costs.
          const UserCost current_bound = check_cost_bound(durations_m->second);

          auto search = _max_cost_per_hour.find(profile);
          assert(search != _max_cost_per_hour.end());
          const auto max_cost_per_hour_for_profile = search->second;

          const std::scoped_lock<std::mutex> lock(cost_bound_m);
          _cost_upper_bound =
            std::max(_cost_upper_bound,
                     max_cost_per_hour_for_profile *
                       utils::scale_from_user_duration(current_bound));
        }
      }
    } catch (...) {
      const std::scoped_lock<std::mutex> lock(ep_m);
      ep = std::current_exception();
    }
  };

  std::vector<std::thread> matrix_threads;
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

Solution Input::solve(const unsigned exploration_level,
                      const unsigned nb_thread,
                      const Timeout& timeout,
                      const std::vector<HeuristicParameters>& h_param) {
  return solve(utils::get_nb_searches(exploration_level),
               utils::get_depth(exploration_level),
               nb_thread,
               timeout,
               h_param);
}

Solution Input::solve(const unsigned nb_searches,
                      const unsigned depth,
                      const unsigned nb_thread,
                      const Timeout& timeout,
                      const std::vector<HeuristicParameters>& h_param) {
  run_basic_checks();

  // Pinned tasks require solving mode with vehicle.steps
  bool has_pinned = std::ranges::any_of(jobs, [](const Job& j) {
    return j.pinned;
  });
  if (has_pinned && !_has_initial_routes) {
    const auto it = std::ranges::find_if(jobs, [](const Job& j) {
      return j.pinned;
    });
    assert(it != jobs.end());
    throw InputException(std::format(
      "Pinned task {} must be included in exactly one vehicle.steps.",
      it->id));
  }

  if (_has_initial_routes) {
    set_vehicle_steps_ranks();
  }

  init_exclusive_tags();

  set_jobs_durations_per_vehicle_type();

  set_matrices(nb_thread);
  set_vehicles_costs();

  // Fill vehicle/job compatibility matrices.
  set_skills_compatibility();
  set_extra_compatibility();
  enforce_pinned_eligibility();
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

  // Post-solve verification: pinned tasks must still be on their pinned vehicle
  if (!_pinned_vehicle_by_job.empty()) {
    // Build assigned vehicle map: job id -> vehicle id in solution
    std::unordered_map<Id, Id> job_to_vehicle_id;
    for (const auto& route : sol.routes) {
      for (const auto& step : route.steps) {
        if (step.step_type == STEP_TYPE::JOB) {
          job_to_vehicle_id[step.id] = route.vehicle;
        }
      }
    }

    for (Index j = 0; j < jobs.size(); ++j) {
      if (!_pinned_vehicle_by_job[j].has_value()) {
        continue;
      }
      const auto pinned_v_rank = _pinned_vehicle_by_job[j].value();
      const auto expected_v_id = vehicles[pinned_v_rank].id;
      const auto& job = jobs[j];
      auto it = job_to_vehicle_id.find(job.id);
      if (it == job_to_vehicle_id.end() || it->second != expected_v_id) {
        throw InputException(std::format(
          "Pinned task {} not assigned to pinned vehicle {}.", job.id, expected_v_id));
      }
    }

    // Verify pinned_position constraints
    for (Index v = 0; v < vehicles.size(); ++v) {
      const auto& route_opt = std::ranges::find_if(sol.routes, [&](const auto& r){return r.vehicle == vehicles[v].id;});
      if (route_opt == sol.routes.end()) {
        continue;
      }
      const auto& steps = route_opt->steps;
      // Build list of job step ids in order
      std::vector<Id> job_ids;
      job_ids.reserve(steps.size());
      for (const auto& s : steps) {
        if (s.step_type == STEP_TYPE::JOB) {
          job_ids.push_back(s.id);
        }
      }
      if (const auto pf = pinned_first_for_vehicle(v); pf.has_value()) {
        const auto& req = pf.value();
        if (req.job_rank.has_value()) {
          if (job_ids.empty() || job_ids.front() != jobs[req.job_rank.value()].id) {
            throw InputException(std::format(
              "Pinned-first task {} not first on vehicle {}.",
              jobs[req.job_rank.value()].id,
              vehicles[v].id));
          }
        } else if (req.pickup_rank.has_value() && req.delivery_rank.has_value()) {
          if (job_ids.size() < 2 ||
              job_ids[0] != jobs[req.pickup_rank.value()].id ||
              job_ids[1] != jobs[req.delivery_rank.value()].id) {
            throw InputException(std::format(
              "Pinned-first shipment steps {} and {} not contiguous at start on vehicle {}.",
              jobs[req.pickup_rank.value()].id,
              jobs[req.delivery_rank.value()].id,
              vehicles[v].id));
          }
        }
      }
      if (const auto pl = pinned_last_for_vehicle(v); pl.has_value()) {
        const auto& req = pl.value();
        if (req.job_rank.has_value()) {
          if (job_ids.empty() || job_ids.back() != jobs[req.job_rank.value()].id) {
            throw InputException(std::format(
              "Pinned-last task {} not last on vehicle {}.",
              jobs[req.job_rank.value()].id,
              vehicles[v].id));
          }
        } else if (req.pickup_rank.has_value() && req.delivery_rank.has_value()) {
          if (job_ids.size() < 2 ||
              job_ids[job_ids.size() - 2] != jobs[req.pickup_rank.value()].id ||
              job_ids.back() != jobs[req.delivery_rank.value()].id) {
            throw InputException(std::format(
              "Pinned-last shipment steps {} and {} not contiguous at end on vehicle {}.",
              jobs[req.pickup_rank.value()].id,
              jobs[req.delivery_rank.value()].id,
              vehicles[v].id));
          }
        }
      }
    }
  }

  if (_geometry) {
    std::vector<std::thread> threads;
    threads.reserve(sol.routes.size());
    std::counting_semaphore<MAX_ROUTING_THREADS> semaphore(
      std::min(MAX_ROUTING_THREADS, nb_thread));

    auto run_routing = [this, &semaphore, &sol](std::size_t i) {
      semaphore.acquire();
      try {
        auto& route = sol.routes[i];
        const auto& profile = route.profile;
        auto rw = std::ranges::find_if(_routing_wrappers, [&](const auto& wr) {
          return wr->profile == profile;
        });
        if (rw == _routing_wrappers.end()) {
          std::cerr << "[Warning] Route geometry request with non-routable "
                       "profile "
                    << profile << ". Skipping geometry." << std::endl;
        } else {
          (*rw)->add_geometry(route);
        }
      } catch (const RoutingException& e) {
        std::cerr << "[Warning] Failed to get geometry for route "
                  << sol.routes[i].vehicle << ": " << e.message
                  << ". Route will lack geometry." << std::endl;
      } catch (const std::exception& e) {
        std::cerr << "[Warning] Error getting geometry for route "
                  << sol.routes[i].vehicle << ": " << e.what()
                  << ". Route will lack geometry." << std::endl;
      }
      semaphore.release();
    };

    for (std::size_t i = 0; i < sol.routes.size(); ++i) {
      threads.emplace_back(run_routing, i);
    }

    for (auto& t : threads) {
      t.join();
    }

    _end_routing = std::chrono::high_resolution_clock::now();
    auto routing = std::chrono::duration_cast<std::chrono::milliseconds>(
                     _end_routing - _end_solving)
                     .count();

    sol.summary.computing_times.routing = routing;

    // Drop routes that failed geometry and move their jobs to unassigned.
    std::vector<Route> geom_kept;
    geom_kept.reserve(sol.routes.size());
    std::vector<Job> geom_unassigned;

    for (const auto& route : sol.routes) {
      if (route.geometry.empty()) {
        for (const auto& st : route.steps) {
          if (st.step_type == STEP_TYPE::JOB) {
            Index r = 0;
            if (st.job_type.has_value() &&
                st.job_type.value() == JOB_TYPE::PICKUP) {
              r = pickup_id_to_rank.at(st.id);
            } else if (st.job_type.has_value() &&
                       st.job_type.value() == JOB_TYPE::DELIVERY) {
              r = delivery_id_to_rank.at(st.id);
            } else {
              r = job_id_to_rank.at(st.id);
            }
            geom_unassigned.push_back(jobs[r]);
          }
        }
      } else {
        geom_kept.push_back(route);
      }
    }

    if (geom_kept.size() != sol.routes.size()) {
      std::cerr << "[Warning] Dropped "
                << (sol.routes.size() - geom_kept.size())
                << " route(s) due to geometry failure; "
                << geom_unassigned.size()
                << " job(s) moved to unassigned." << std::endl;

      std::vector<Job> merged;
      merged.reserve(sol.unassigned.size() + geom_unassigned.size());
      for (const auto& j : sol.unassigned) {
        merged.push_back(j);
      }
      for (const auto& j : geom_unassigned) {
        merged.push_back(j);
      }

      const auto old_times = sol.summary.computing_times;
      sol.routes = std::move(geom_kept);
      sol.unassigned = std::move(merged);

      new (&sol.summary)
        Summary(static_cast<unsigned>(sol.routes.size()),
                static_cast<unsigned>(sol.unassigned.size()),
                zero_amount());
      for (const auto& route : sol.routes) {
        sol.summary.cost += route.cost;
        sol.summary.delivery += route.delivery;
        sol.summary.pickup += route.pickup;
        sol.summary.setup += route.setup;
        sol.summary.service += route.service;
        sol.summary.priority += route.priority;
        sol.summary.duration += route.duration;
        sol.summary.distance += route.distance;
        sol.summary.waiting_time += route.waiting_time;
        sol.summary.violations += route.violations;
      }
      sol.summary.computing_times = old_times;
    }
  }

  // Post-pass budget enforcement and repair.
  utils::repair_budget(*this, sol);

  // Final validation: ensure first-leg limit holds.
  validate_first_leg_limits(sol);

  return sol;
}

Solution Input::check(unsigned nb_thread) {
#if USE_LIBGLPK
  run_basic_checks();

  set_jobs_durations_per_vehicle_type();

  set_vehicle_steps_ranks();

  init_exclusive_tags();

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

  // Final validation: ensure first-leg limit holds.
  validate_first_leg_limits(sol);

  return sol;
#else
  // Attempt to use libglpk while compiling without it.
  throw InputException("VROOM compiled without libglpk installed.");
  // Silence -Wunused-parameter warning.
  (void)nb_thread;
#endif
}

} // namespace vroom
