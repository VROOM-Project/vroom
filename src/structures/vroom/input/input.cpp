/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "./input.h"
#include "../../../problems/vrp.h"

input::input(std::unique_ptr<routing_io<cost_t>> routing_wrapper, bool geometry)
  : _start_loading(std::chrono::high_resolution_clock::now()),
    _problem_type(PROBLEM_T::TSP),
    _routing_wrapper(std::move(routing_wrapper)),
    _geometry(geometry) {
}

void input::add_job(const job_t& job) {
  _jobs.push_back(job);

  auto& current_job = _jobs.back();

  if (!current_job.user_index()) {
    // Index of this job in the matrix was not specified upon job
    // creation, using current number of locations.
    current_job.set_index(_locations.size());
  }

  _type_with_ids.push_back({TYPE::JOB, current_job.id});
  _locations.push_back(current_job);
}

void input::add_vehicle(const vehicle_t& vehicle) {
  _vehicles.push_back(vehicle);

  auto& current_v = _vehicles.back();

  bool has_start = current_v.has_start();
  bool has_end = current_v.has_end();

  if (has_start) {
    if (!current_v.start.get().user_index()) {
      // Index of this start in the matrix was not specified upon
      // vehicle creation, using current number of locations.
      current_v.start.get().set_index(_locations.size());
    }

    _type_with_ids.push_back({TYPE::START, current_v.id});
    _locations.push_back(current_v.start.get());
  }

  if (has_end) {
    if (!current_v.end.get().user_index()) {
      // Index of this end in the matrix was not specified upon
      // vehicle creation, using current number of locations.
      current_v.end.get().set_index(_locations.size());
    }

    if (!has_start or
        (current_v.start.get().index() != current_v.end.get().index())) {
      // Avoiding duplicate for start/end location.
      _type_with_ids.push_back({TYPE::END, current_v.id});
      _locations.push_back(current_v.end.get());
    }
  }
}

void input::set_matrix(matrix<cost_t>&& m) {
  _matrix = std::move(m);
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

  for (std::size_t i = 0; i < _matrix.size(); ++i) {
    for (std::size_t j = 0; j < _matrix.size(); ++j) {
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

PROBLEM_T input::get_problem_type() const {
  return _problem_type;
}

std::unique_ptr<vrp> input::get_problem() const {
  std::vector<index_t> problem_indices;
  std::transform(_locations.begin(),
                 _locations.end(),
                 std::back_inserter(problem_indices),
                 [](const auto& loc) { return loc.index(); });

  return std::make_unique<tsp>(*this, problem_indices, 0);
}

solution input::solve(unsigned nb_thread) {
  if (_matrix.size() < 2) {
    // OSRM call if matrix not already provided.
    assert(_routing_wrapper);
    BOOST_LOG_TRIVIAL(info) << "[Loading] Start matrix computing.";
    _matrix = _routing_wrapper->get_matrix(_locations);
  }

  // Check for potential overflow in solution cost.
  this->check_cost_bound();

  // Load relevant problem.
  auto instance = this->get_problem();
  _end_loading = std::chrono::high_resolution_clock::now();

  auto loading = std::chrono::duration_cast<std::chrono::milliseconds>(
                   _end_loading - _start_loading)
                   .count();

  BOOST_LOG_TRIVIAL(info) << "[Loading] Done, took " << loading << " ms.";

  // Solve.
  solution sol = instance->solve(nb_thread);

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
      _routing_wrapper->add_route_geometry(route);
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
