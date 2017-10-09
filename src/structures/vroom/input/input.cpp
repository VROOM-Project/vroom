/*

This file is part of VROOM.

Copyright (c) 2015-2017, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <iostream>

#include "../../../problems/vrp.h"
#include "./input.h"

input::input(std::unique_ptr<routing_io<distance_t>> routing_wrapper,
             bool geometry)
  : _start_loading(std::chrono::high_resolution_clock::now()),
    _problem_type(PROBLEM_T::TSP),
    _routing_wrapper(std::move(routing_wrapper)),
    _geometry(geometry) {}

void input::add_job(ID_t id, const optional_coords_t& coords) {
  // Using current number of locations as index of this job in the
  // matrix.
  add_job(id, coords, _locations.size());
}

void input::add_job(ID_t id,
                    const optional_coords_t& coords,
                    index_t index) {
  if (coords == boost::none) {
    _jobs.emplace_back(id, index);
  }
  else {
    _jobs.emplace_back(id,
                      index,
                      coords.get()[0],
                      coords.get()[1]);
  }

  // Remember mapping between the job index in the matrix and its rank
  // in _jobs.
  _index_to_job_rank.insert({index, _jobs.size() - 1});
  _all_indices.insert(index);

  _locations.push_back(_jobs.back());
  _index_to_loc_rank.insert({index, _locations.size() - 1});
}

void input::add_vehicle(ID_t id,
                        const optional_coords_t& start_coords,
                        const optional_coords_t& end_coords) {
  // Using current number of locations as index of start and end in
  // the matrix.
  boost::optional<index_t> start_index;
  if (start_coords) {
    start_index = _locations.size();
  }
  boost::optional<index_t> end_index;
  if (end_coords) {
    end_index = (start_coords) ?
      _locations.size() + 1:
      _locations.size();
  }

  add_vehicle(id, start_coords, end_coords, start_index, end_index);
}

void input::add_vehicle(ID_t id,
                        const optional_coords_t& start_coords,
                        const optional_coords_t& end_coords,
                        boost::optional<index_t> start_index,
                        boost::optional<index_t> end_index) {

  if ((start_index == boost::none ) and (end_index == boost::none)) {
    throw custom_exception("No start or end specified for vehicle " +
                           std::to_string(id) + '.');
  }
  boost::optional<location_t> start = (start_index == boost::none) ?
    boost::none:
    ((start_coords == boost::none) ?
      boost::optional<location_t>(start_index.get()):
      boost::optional<location_t>({start_index.get(), (*start_coords)[0], (*start_coords)[1]})
    );

  if (start_index != boost::none) {
    _all_indices.insert(start_index.get());
  }

  boost::optional<location_t> end = (end_index == boost::none) ?
    boost::none:
    ((end_coords == boost::none) ?
      boost::optional<location_t>(end_index.get()):
      boost::optional<location_t>({end_index.get(), (*end_coords)[0], (*end_coords)[1]})
    );

  if (end_index != boost::none) {
    _all_indices.insert(end_index.get());
  }

  _vehicles.emplace_back(id, start, end);

  if (start) {
    _locations.push_back(_vehicles.back().start.get());
    _index_to_loc_rank.insert({_vehicles.back().start.get().index , _locations.size() - 1});
  }
  if (end) {
    _locations.push_back(_vehicles.back().end.get());
    _index_to_loc_rank.insert({_vehicles.back().end.get().index , _locations.size() - 1});
  }
}

void input::set_matrix() {
  //Don't call osrm, if matrix is already provided.
  if (_matrix.size() < 2) {
    assert(_routing_wrapper);
    BOOST_LOG_TRIVIAL(info) << "[Loading] Start matrix computing.";
    _matrix = _routing_wrapper->get_matrix(_locations);
  }
  // Distances on the diagonal are never used except in the minimum
  // weight perfect matching (munkres call during the TSP
  // heuristic). This makes sure no node will be matched with itself
  // at that time.
  for (index_t i = 0; i < _matrix.size(); ++i) {
    _matrix[i][i] = INFINITE_DISTANCE;
  }
}

index_t input::get_location_rank_from_index(index_t index) const {
  auto result = _index_to_loc_rank.find(index);
  assert(result != _index_to_loc_rank.end());
  return result->second;
}

index_t input::get_job_rank_from_index(index_t index) const {
  auto result = _index_to_job_rank.find(index);
  assert(result != _index_to_job_rank.end());
  return result->second;
}

PROBLEM_T input::get_problem_type() const {
  return _problem_type;
}

std::unique_ptr<vrp> input::get_problem() const {
  std::vector<index_t> problem_indices;
  for (const auto& i: _all_indices) {
    problem_indices.push_back(i);
  }

  return std::make_unique<tsp>(*this,
                               problem_indices,
                               0);
}

solution input::solve(unsigned nb_thread) {
  // Compute matrix and load relevant problem.
  this->set_matrix();
  auto instance = this->get_problem();
  _end_loading = std::chrono::high_resolution_clock::now();

  auto loading = std::chrono::duration_cast<std::chrono::milliseconds>
    (_end_loading - _start_loading).count();

  BOOST_LOG_TRIVIAL(info) << "[Loading] Done, took "
                          << loading << " ms.";

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
    BOOST_LOG_TRIVIAL(info)
      << "[Route] Start computing detailed route.";

    for (auto& route : sol.routes) {
      _routing_wrapper->add_route_geometry(route);
      sol.summary.duration += route.duration;
      sol.summary.distance += route.distance;
    }

    _end_routing = std::chrono::high_resolution_clock::now();
    auto routing = std::chrono::duration_cast<std::chrono::milliseconds>
      (_end_routing - _end_solving).count();

    sol.summary.computing_times.routing = routing;

    BOOST_LOG_TRIVIAL(info) << "[Route] Done, took "
                            << routing
                            << " ms.";
  }

  return sol;
}
