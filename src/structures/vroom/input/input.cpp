/*

This file is part of VROOM.

Copyright (c) 2015-2016, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <iostream>
#include "./input.h"
#include "../../../problems/vrp.h"

input::input():
  _location_number(0),
  _problem_type(PROBLEM_T::TSP){}

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

void input::set_routing(std::unique_ptr<routing_io<distance_t>> routing_wrapper){
  _routing_wrapper = std::move(routing_wrapper);
}

void input::set_matrix(){
  assert(_routing_wrapper);
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
  std::unique_ptr<vrp> problem;
  return problem;
}
