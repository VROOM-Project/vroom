/*

This file is part of VROOM.

Copyright (c) 2015-2016, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <iostream>
#include "./input.h"
#include "../../../problems/vrp.h"

input::input(std::unique_ptr<routing_io<distance_t>> routing_wrapper,
             bool geometry):
  _start_loading(std::chrono::high_resolution_clock::now()),
  _location_number(0),
  _problem_type(PROBLEM_T::TSP),
  _routing_wrapper(std::move(routing_wrapper)),
  _geometry(geometry){}

void input::add_job(index_t id, const optional_coords_t& coords){
  // Using current number of locations as index of this job in the
  // matrix.
  if(coords == boost::none){
    _jobs.emplace_back(id, _location_number++);
  }
  else{
    _jobs.emplace_back(id,
                       _location_number++,
                       coords.get()[0],
                       coords.get()[1]);
  }

  // Remember mapping between the job index in the matrix and its rank
  // in _jobs.
  _index_to_job_rank.insert({_location_number - 1, _jobs.size() - 1});

  _ordered_locations.push_back(_jobs.back());
}

void input::add_vehicle(index_t id,
                        const optional_coords_t& start_coords,
                        const optional_coords_t& end_coords){
  // Using current number of locations as index of start and end in
  // the matrix.
  if((!start_coords) and (!end_coords)){
    throw custom_exception("No start or end specified for vehicle "
                           + std::to_string(id)
                           + '.');
  }

  boost::optional<location_t> start = (start_coords == boost::none) ?
    boost::none:
    boost::optional<location_t>(
      {_location_number++, (*start_coords)[0], (*start_coords)[1]}
      );

  boost::optional<location_t> end = (end_coords == boost::none) ?
    boost::none:
    boost::optional<location_t>(
      {_location_number++, (*end_coords)[0], (*end_coords)[1]}
      );

  _vehicles.emplace_back(id, start, end);

  if(start_coords){
    _ordered_locations.push_back(_vehicles.back().start.get());
  }
  if(end_coords){
    _ordered_locations.push_back(_vehicles.back().end.get());
  }
}

void input::set_matrix(){
  assert(_routing_wrapper);
  BOOST_LOG_TRIVIAL(info) << "[Loading] Start matrix computing.";
  _matrix = _routing_wrapper->get_matrix(_ordered_locations);
}

index_t input::get_location_number() const{
  return _location_number;
}

location_t input::get_location_at(index_t index) const{
  return _ordered_locations[index];
}

index_t input::get_job_rank_from_index(index_t index) const{
  auto result = _index_to_job_rank.find(index);
  assert(result != _index_to_job_rank.end());
  return result->second;
}

PROBLEM_T input::get_problem_type() const{
  return _problem_type;
}

std::unique_ptr<vrp> input::get_problem() const{
  return std::make_unique<tsp>(*this, 0);
}

solution input::solve(unsigned nb_thread){
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
  sol.summary.computing_times.solving
    = std::chrono::duration_cast<std::chrono::milliseconds>
      (_end_solving - _end_loading).count();

  if(_geometry){
    // Routing stuff.
    BOOST_LOG_TRIVIAL(info)
      << "[Route] Start computing detailed route.";

    for(auto& route: sol.routes){
      _routing_wrapper->add_route_geometry(route);
      sol.summary.duration += route.duration;
      sol.summary.distance += route.distance;
    }

    _end_routing = std::chrono::high_resolution_clock::now();
    auto routing
      = std::chrono::duration_cast<std::chrono::milliseconds>
      (_end_routing - _end_solving).count();

    sol.summary.computing_times.routing = routing;

    BOOST_LOG_TRIVIAL(info) << "[Route] Done, took "
                            << routing
                            << " ms.";
  }

  return sol;
}
