/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/
#include <array>

#include <boost/optional.hpp>

#include "problems/cvrp/cvrp.h"
#include "problems/tsp/tsp.h"
#include "problems/vrp.h"
#include "problems/vrptw/vrptw.h"
#include "structures/vroom/input/input.h"
#include "utils/exception.h"
#include "utils/helpers.h"

namespace vroom {

Input::Input()
  : _start_loading(std::chrono::high_resolution_clock::now()),
    _has_TW(false),
    _homogeneous_locations(true),
    _geometry(false),
    _all_locations_have_coords(true) {
}

void Input::set_geometry(bool geometry) {
  _geometry = geometry;
}

void Input::set_routing(
  std::unique_ptr<routing::Wrapper<Cost>> routing_wrapper) {
  _routing_wrapper = std::move(routing_wrapper);
}

void Input::add_job(const Job& job) {
  jobs.push_back(job);

  auto& current_job = jobs.back();

  // Ensure amount size consistency.
  this->check_amount_size(current_job.amount.size());

  this->store_amount_lower_bound(current_job.amount);

  // Ensure that skills are either always or never provided.
  if (_locations.empty()) {
    _has_skills = !current_job.skills.empty();
  } else {
    if (_has_skills != !current_job.skills.empty()) {
      throw Exception(ERROR::INPUT, "Missing skills.");
    }
  }

  // Check for time-windows.
  _has_TW |= (!(job.tws.size() == 1) or !job.tws[0].is_default());

  if (!current_job.location.user_index()) {
    // Index of this job in the matrix was not specified upon job
    // creation, using current number of locations.
    current_job.location.set_index(_locations.size());
  }
  _matrix_used_index.insert(current_job.index());
  _all_locations_have_coords &= current_job.location.has_coordinates();

  _locations.push_back(current_job.location);
}

void Input::add_vehicle(const Vehicle& vehicle) {
  vehicles.push_back(vehicle);

  auto& current_v = vehicles.back();

  // Ensure amount size consistency.
  this->check_amount_size(current_v.capacity.size());

  // Ensure that skills are either always or never provided.
  if (_locations.empty()) {
    _has_skills = !current_v.skills.empty();
  } else {
    if (_has_skills != !current_v.skills.empty()) {
      throw Exception(ERROR::INPUT, "Missing skills.");
    }
  }

  // Check for time-windows.
  _has_TW |= !vehicle.tw.is_default();

  bool has_start = current_v.has_start();
  bool has_end = current_v.has_end();

  if (has_start) {
    if (!current_v.start.get().user_index()) {
      // Index of this start in the matrix was not specified upon
      // vehicle creation, using current number of locations.
      assert(current_v.start.get().has_coordinates());
      current_v.start.get().set_index(_locations.size());
    }

    _matrix_used_index.insert(current_v.start.get().index());
    _all_locations_have_coords &= current_v.start.get().has_coordinates();

    _locations.push_back(current_v.start.get());
  }

  if (has_end) {
    if (!current_v.end.get().user_index()) {
      // Index of this end in the matrix was not specified upon
      // vehicle creation, using current number of locations.
      assert(current_v.end.get().has_coordinates());
      current_v.end.get().set_index(_locations.size());
    }

    _matrix_used_index.insert(current_v.end.get().index());
    _all_locations_have_coords &= current_v.end.get().has_coordinates();

    _locations.push_back(current_v.end.get());
  }

  // Check for homogeneous locations among vehicles.
  if (vehicles.size() > 1) {
    _homogeneous_locations &=
      vehicles.front().has_same_locations(vehicles.back());
  }
}

void Input::store_amount_lower_bound(const Amount& amount) {
  if (_amount_lower_bound.empty()) {
    // Create on first call.
    _amount_lower_bound = amount;
  } else {
    for (std::size_t i = 0; i < _amount_size; ++i) {
      _amount_lower_bound[i] = std::min(_amount_lower_bound[i], amount[i]);
    }
  }
}

void Input::check_amount_size(unsigned size) {
  if (_locations.empty()) {
    // Updating real value on first call.
    _amount_size = size;
  } else {
    // Checking consistency for amount/capacity input lengths.
    if (size != _amount_size) {
      throw Exception(ERROR::INPUT,
                      "Inconsistent amount/capacity lengths: " +
                        std::to_string(size) + " and " +
                        std::to_string(_amount_size) + '.');
    }
  }
}

void Input::set_matrix(Matrix<Cost>&& m) {
  _matrix = std::move(m);
}

unsigned Input::amount_size() const {
  return _amount_size;
}

Amount Input::get_amount_lower_bound() const {
  return _amount_lower_bound;
}

bool Input::has_skills() const {
  return _has_skills;
}

bool Input::has_homogeneous_locations() const {
  return _homogeneous_locations;
}

bool Input::vehicle_ok_with_job(Index v_index, Index j_index) const {
  return _vehicle_to_job_compatibility[v_index][j_index];
}

bool Input::vehicle_ok_with_vehicle(Index v1_index, Index v2_index) const {
  return _vehicle_to_vehicle_compatibility[v1_index][v2_index];
}

const Matrix<Cost>& Input::get_matrix() const {
  return _matrix;
}

Matrix<Cost> Input::get_sub_matrix(const std::vector<Index>& indices) const {
  return _matrix.get_sub_matrix(indices);
}

void Input::check_cost_bound() const {
  // Check that we don't have any overflow while computing an upper
  // bound for solution cost.

  std::vector<Cost> max_cost_per_line(_matrix.size(), 0);
  std::vector<Cost> max_cost_per_column(_matrix.size(), 0);

  for (const auto i : _matrix_used_index) {
    for (const auto j : _matrix_used_index) {
      max_cost_per_line[i] = std::max(max_cost_per_line[i], _matrix[i][j]);
      max_cost_per_column[j] = std::max(max_cost_per_column[j], _matrix[i][j]);
    }
  }

  Cost jobs_departure_bound = 0;
  Cost jobs_arrival_bound = 0;
  for (const auto& j : jobs) {
    jobs_departure_bound =
      utils::add_without_overflow(jobs_departure_bound,
                                  max_cost_per_line[j.index()]);
    jobs_arrival_bound =
      utils::add_without_overflow(jobs_arrival_bound,
                                  max_cost_per_column[j.index()]);
  }

  Cost jobs_bound = std::max(jobs_departure_bound, jobs_arrival_bound);

  Cost start_bound = 0;
  Cost end_bound = 0;
  for (const auto& v : vehicles) {
    if (v.has_start()) {
      start_bound =
        utils::add_without_overflow(start_bound,
                                    max_cost_per_line[v.start.get().index()]);
    }
    if (v.has_end()) {
      end_bound =
        utils::add_without_overflow(end_bound,
                                    max_cost_per_column[v.end.get().index()]);
    }
  }

  Cost bound = utils::add_without_overflow(start_bound, jobs_bound);
  bound = utils::add_without_overflow(bound, end_bound);
}

void Input::set_compatibility() {
  // Default to no restriction when no skills are provided.
  _vehicle_to_job_compatibility =
    std::vector<std::vector<bool>>(vehicles.size(),
                                   std::vector<bool>(jobs.size(), true));
  if (_has_skills) {
    for (std::size_t v = 0; v < vehicles.size(); ++v) {
      const auto& v_skills = vehicles[v].skills;
      assert(!v_skills.empty());

      for (std::size_t j = 0; j < jobs.size(); ++j) {
        bool is_compatible = true;
        assert(!jobs[j].skills.empty());
        for (const auto& s : jobs[j].skills) {
          auto search = v_skills.find(s);
          is_compatible &= (search != v_skills.end());
          if (!is_compatible) {
            break;
          }
        }
        _vehicle_to_job_compatibility[v][j] = is_compatible;
      }
    }
  }

  // Derive potential extra incompatibilities : jobs with amount that
  // does not fit into vehicle or that cannot be added to an empty
  // route for vehicle based on the timing constraints (when they
  // apply).
  for (std::size_t v = 0; v < vehicles.size(); ++v) {
    TWRoute empty_route(*this, v);
    for (std::size_t j = 0; j < jobs.size(); ++j) {
      if (_vehicle_to_job_compatibility[v][j]) {
        auto is_compatible = (jobs[j].amount <= vehicles[v].capacity);
        if (_has_TW) {
          is_compatible &= empty_route.is_valid_addition_for_tw(*this, j, 0);
        }

        _vehicle_to_job_compatibility[v][j] = is_compatible;
      }
    }
  }

  _vehicle_to_vehicle_compatibility =
    std::vector<std::vector<bool>>(vehicles.size(),
                                   std::vector<bool>(vehicles.size(), false));
  for (std::size_t v1 = 0; v1 < vehicles.size(); ++v1) {
    _vehicle_to_vehicle_compatibility[v1][v1] = true;
    for (std::size_t v2 = v1 + 1; v2 < vehicles.size(); ++v2) {
      for (std::size_t j = 0; j < jobs.size(); ++j) {
        if (_vehicle_to_job_compatibility[v1][j] and
            _vehicle_to_job_compatibility[v2][j]) {
          _vehicle_to_vehicle_compatibility[v1][v2] = true;
          _vehicle_to_vehicle_compatibility[v2][v1] = true;
          break;
        }
      }
    }
  }
}

std::unique_ptr<VRP> Input::get_problem() const {
  if (_has_TW) {
    return std::make_unique<VRPTW>(*this);
  } else {
    return std::make_unique<CVRP>(*this);
  }
}

Solution Input::solve(unsigned exploration_level, unsigned nb_thread) {
  if (_geometry and !_all_locations_have_coords) {
    // Early abort when info is required with missing coordinates.
    throw Exception(ERROR::INPUT,
                    "Route geometry request with missing coordinates.");
  }

  if (_matrix.size() < 2) {
    // OSRM call if matrix not already provided.
    assert(_routing_wrapper);
    _matrix = _routing_wrapper->get_matrix(_locations);
  }

  // Check for potential overflow in solution cost.
  this->check_cost_bound();

  // Fill vehicle/job compatibility matrix.
  this->set_compatibility();

  // Load relevant problem.
  auto instance = this->get_problem();
  _end_loading = std::chrono::high_resolution_clock::now();

  auto loading = std::chrono::duration_cast<std::chrono::milliseconds>(
                   _end_loading - _start_loading)
                   .count();

  // Solve.
  auto sol = instance->solve(exploration_level, nb_thread);

  // Update timing info.
  sol.summary.computing_times.loading = loading;

  _end_solving = std::chrono::high_resolution_clock::now();
  sol.summary.computing_times.solving =
    std::chrono::duration_cast<std::chrono::milliseconds>(_end_solving -
                                                          _end_loading)
      .count();

  if (_geometry) {
    for (auto& route : sol.routes) {
      _routing_wrapper->add_route_info(route);
      sol.summary.distance += route.distance;
    }

    _end_routing = std::chrono::high_resolution_clock::now();
    auto routing = std::chrono::duration_cast<std::chrono::milliseconds>(
                     _end_routing - _end_solving)
                     .count();

    sol.summary.computing_times.routing = routing;
  }

  return sol;
}

} // namespace vroom
