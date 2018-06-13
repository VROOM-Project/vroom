/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/
#include <array>

#include <boost/log/trivial.hpp>
#include <boost/optional.hpp>

#include "problems/cvrp/cvrp.h"
#include "problems/tsp/tsp.h"
#include "problems/vrp.h"
#include "structures/vroom/input/input.h"
#include "utils/exceptions.h"
#include "utils/helpers.h"

input::input(std::unique_ptr<routing_io<cost_t>> routing_wrapper, bool geometry)
  : _start_loading(std::chrono::high_resolution_clock::now()),
    _routing_wrapper(std::move(routing_wrapper)),
    _geometry(geometry),
    _all_locations_have_coords(true) {
}

void input::add_job(const job_t& job) {
  _jobs.push_back(job);

  auto& current_job = _jobs.back();

  // Ensure amount size consistency.
  this->check_amount_size(current_job.amount.size());

  this->store_amount_lower_bound(current_job.amount);

  // Ensure that skills are either always or never provided.
  if (_locations.empty()) {
    _has_skills = !current_job.skills.empty();
  } else {
    if (_has_skills != !current_job.skills.empty()) {
      throw custom_exception("Missing skills.");
    }
  }

  if (!current_job.location.user_index()) {
    // Index of this job in the matrix was not specified upon job
    // creation, using current number of locations.
    current_job.location.set_index(_locations.size());
  }
  _matrix_used_index.insert(current_job.index());
  _all_locations_have_coords &= current_job.location.has_coordinates();

  _locations.push_back(current_job.location);
}

void input::add_vehicle(const vehicle_t& vehicle) {
  _vehicles.push_back(vehicle);

  auto& current_v = _vehicles.back();

  // Ensure amount size consistency.
  this->check_amount_size(current_v.capacity.size());

  // Ensure that skills are either always or never provided.
  if (_locations.empty()) {
    _has_skills = !current_v.skills.empty();
  } else {
    if (_has_skills != !current_v.skills.empty()) {
      throw custom_exception("Missing skills.");
    }
  }

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
}

void input::store_amount_lower_bound(const amount_t& amount) {
  if (_amount_lower_bound.empty()) {
    // Create on first call.
    _amount_lower_bound = amount;
  } else {
    for (std::size_t i = 0; i < _amount_size; ++i) {
      _amount_lower_bound[i] = std::min(_amount_lower_bound[i], amount[i]);
    }
  }
}

void input::check_amount_size(unsigned size) {
  if (_locations.empty()) {
    // Updating real value on first call.
    _amount_size = size;
  } else {
    // Checking consistency for amount/capacity input lengths.
    if (size != _amount_size) {
      throw custom_exception("Inconsistent amount/capacity lengths: " +
                             std::to_string(size) + " and " +
                             std::to_string(_amount_size) + '.');
    }
  }
}

void input::set_matrix(matrix<cost_t>&& m) {
  _matrix = std::move(m);
}

unsigned input::amount_size() const {
  return _amount_size;
}

amount_t input::get_amount_lower_bound() const {
  return _amount_lower_bound;
}

bool input::vehicle_ok_with_job(index_t v_index, index_t j_index) const {
  return _vehicle_to_job_compatibility[v_index][j_index];
}

const matrix<cost_t>& input::get_matrix() const {
  return _matrix;
}

matrix<cost_t>
input::get_sub_matrix(const std::vector<index_t>& indices) const {
  return _matrix.get_sub_matrix(indices);
}

void input::check_cost_bound() const {
  // Check that we don't have any overflow while computing an upper
  // bound for solution cost.

  std::vector<cost_t> max_cost_per_line(_matrix.size(), 0);
  std::vector<cost_t> max_cost_per_column(_matrix.size(), 0);

  for (const auto i : _matrix_used_index) {
    for (const auto j : _matrix_used_index) {
      max_cost_per_line[i] = std::max(max_cost_per_line[i], _matrix[i][j]);
      max_cost_per_column[j] = std::max(max_cost_per_column[j], _matrix[i][j]);
    }
  }

  cost_t jobs_departure_bound = 0;
  cost_t jobs_arrival_bound = 0;
  for (const auto& j : _jobs) {
    jobs_departure_bound =
      add_without_overflow(jobs_departure_bound, max_cost_per_line[j.index()]);
    jobs_arrival_bound =
      add_without_overflow(jobs_arrival_bound, max_cost_per_column[j.index()]);
  }

  cost_t jobs_bound = std::max(jobs_departure_bound, jobs_arrival_bound);

  cost_t start_bound = 0;
  cost_t end_bound = 0;
  for (const auto& v : _vehicles) {
    if (v.has_start()) {
      start_bound =
        add_without_overflow(start_bound,
                             max_cost_per_line[v.start.get().index()]);
    }
    if (v.has_end()) {
      end_bound =
        add_without_overflow(end_bound,
                             max_cost_per_column[v.end.get().index()]);
    }
  }

  cost_t bound = add_without_overflow(start_bound, jobs_bound);
  bound = add_without_overflow(bound, end_bound);

  BOOST_LOG_TRIVIAL(info) << "[Loading] solution cost upper bound: " << bound
                          << ".";
}

void input::set_vehicle_to_job_compatibility() {
  // Default to no restriction when no skills are provided.
  _vehicle_to_job_compatibility =
    std::vector<std::vector<bool>>(_vehicles.size(),
                                   std::vector<bool>(_jobs.size(), true));
  if (_has_skills) {
    for (std::size_t v = 0; v < _vehicles.size(); ++v) {
      const auto& v_skills = _vehicles[v].skills;
      assert(!v_skills.empty());

      for (std::size_t j = 0; j < _jobs.size(); ++j) {
        bool is_compatible = true;
        assert(!_jobs[j].skills.empty());
        for (const auto& s : _jobs[j].skills) {
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
}

std::unique_ptr<vrp> input::get_problem() const {
  return std::make_unique<cvrp>(*this);
}

solution input::format_solution(const raw_solution& raw_routes) const {
  std::vector<route_t> routes;
  cost_t total_cost = 0;
  duration_t total_service = 0;
  amount_t total_amount(_amount_size);

  // All job ranks start with unassigned status.
  std::unordered_set<index_t> unassigned_ranks;
  for (unsigned i = 0; i < _jobs.size(); ++i) {
    unassigned_ranks.insert(i);
  }

  for (std::size_t i = 0; i < raw_routes.size(); ++i) {
    const auto& route = raw_routes[i];
    if (route.empty()) {
      continue;
    }
    const auto& v = _vehicles[i];
    cost_t cost = 0;
    duration_t service = 0;
    amount_t amount(_amount_size);

    // Steps for current route.
    std::vector<step> steps;

    // Handle start.
    if (v.has_start()) {
      steps.emplace_back(TYPE::START, v.start.get());
      cost += _matrix[v.start.get().index()][_jobs[route.front()].index()];
    }

    // Handle jobs.
    index_t previous = route.front();
    assert(vehicle_ok_with_job(i, previous));
    steps.emplace_back(_jobs[previous]);
    service += steps.back().service;
    amount += steps.back().amount;
    unassigned_ranks.erase(previous);

    for (auto it = ++route.cbegin(); it != route.cend(); ++it) {
      cost += _matrix[_jobs[previous].index()][_jobs[*it].index()];
      assert(vehicle_ok_with_job(i, *it));
      steps.emplace_back(_jobs[*it]);
      service += steps.back().service;
      amount += steps.back().amount;
      unassigned_ranks.erase(*it);
      previous = *it;
    }

    // Handle end.
    if (v.has_end()) {
      steps.emplace_back(TYPE::END, v.end.get());
      cost += _matrix[_jobs[route.back()].index()][v.end.get().index()];
    }

    assert(amount <= _vehicles[i].capacity);
    routes.emplace_back(_vehicles[i].id,
                        std::move(steps),
                        cost,
                        service,
                        amount);

    total_cost += cost;
    total_service += service;
    total_amount += amount;
  }

  // Handle unassigned jobs.
  std::vector<job_t> unassigned_jobs;
  std::transform(unassigned_ranks.begin(),
                 unassigned_ranks.end(),
                 std::back_inserter(unassigned_jobs),
                 [&](auto j) { return _jobs[j]; });

  return solution(0,
                  total_cost,
                  std::move(routes),
                  std::move(unassigned_jobs),
                  total_service,
                  std::move(total_amount));
}

solution input::solve(unsigned nb_thread) {
  if (_geometry and !_all_locations_have_coords) {
    // Early abort when info is required with missing coordinates.
    throw custom_exception("Option -g is invalid with missing coordinates.");
  }

  if (_matrix.size() < 2) {
    // OSRM call if matrix not already provided.
    assert(_routing_wrapper);
    BOOST_LOG_TRIVIAL(info) << "[Loading] Start matrix computing.";
    _matrix = _routing_wrapper->get_matrix(_locations);
  }

  // Check for potential overflow in solution cost.
  this->check_cost_bound();

  // Fill vehicle/job compatibility matrix.
  this->set_vehicle_to_job_compatibility();

  // Load relevant problem.
  auto instance = this->get_problem();
  _end_loading = std::chrono::high_resolution_clock::now();

  auto loading = std::chrono::duration_cast<std::chrono::milliseconds>(
                   _end_loading - _start_loading)
                   .count();

  BOOST_LOG_TRIVIAL(info) << "[Loading] Done, took " << loading << " ms.";

  // Solve.
  auto sol = format_solution(instance->solve(nb_thread));

  // Update timing info.
  sol.summary.computing_times.loading = loading;

  _end_solving = std::chrono::high_resolution_clock::now();
  sol.summary.computing_times.solving =
    std::chrono::duration_cast<std::chrono::milliseconds>(_end_solving -
                                                          _end_loading)
      .count();

  if (_geometry) {
    // Routing stuff.
    BOOST_LOG_TRIVIAL(info) << "[Route] Start computing detailed route.";

    for (auto& route : sol.routes) {
      _routing_wrapper->add_route_info(route);
      sol.summary.duration += route.duration;
      sol.summary.distance += route.distance;
    }

    _end_routing = std::chrono::high_resolution_clock::now();
    auto routing = std::chrono::duration_cast<std::chrono::milliseconds>(
                     _end_routing - _end_solving)
                     .count();

    sol.summary.computing_times.routing = routing;

    BOOST_LOG_TRIVIAL(info) << "[Route] Done, took " << routing << " ms.";
  }

  return sol;
}
