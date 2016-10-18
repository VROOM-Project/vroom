/*

This file is part of VROOM.

Copyright (c) 2015-2016, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "./input.h"

input::input(): _location_number(0){}

void input::add_job(index_t id, optional_coords_t coords){
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
  _ordered_locations.push_back(_jobs.back());
}

void input::add_vehicle(index_t id,
                        optional_coords_t start_coords,
                        optional_coords_t end_coords){
  // Using current number of locations as index of start and end in
  // the matrix.
  if((!start_coords) and (!end_coords)){
    throw custom_exception("No start or end specified for vehicle "
                           + std::to_string(id)
                           + '.');
  }

  boost::optional<location> start = (start_coords == boost::none) ?
    boost::none:
    boost::optional<location>(
      {++_location_number, (*start_coords)[0], (*start_coords)[1]}
      );

  boost::optional<location> end = (end_coords == boost::none) ?
    boost::none:
    boost::optional<location>(
      {++_location_number, (*end_coords)[0], (*end_coords)[1]}
      );

  _vehicles.emplace_back(id, start, end);

  if(start_coords){
    _ordered_locations.push_back(_vehicles.back().start.value());
  }
  if(end_coords){
    _ordered_locations.push_back(_vehicles.back().end.value());
  }
}

index_t input::get_location_number(){
  return _location_number;
}

void input::set_matrix(){
  _matrix = _routing_wrapper->get_matrix(_ordered_locations);
}
