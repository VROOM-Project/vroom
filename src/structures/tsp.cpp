/*
VROOM (Vehicle Routing Open-source Optimization Machine)
Copyright (C) 2015, Julien Coupey

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or (at
your option) any later version.

This program is distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "tsp.h"

tsp::tsp(const cl_args_t& cl_args): 
  _matrix(0),
  _symmetrized_matrix(0),
  _is_symmetric(true),
  _cl_args(cl_args){

  // Only use euclidean loader.
  _loader = std::make_unique<euclidean>(cl_args.input);
  _matrix = _loader->get_matrix();

  _is_symmetric = _matrix.is_symmetric();
}

const matrix<distance_t>& tsp::get_matrix() const{
  return _matrix;
}

const matrix<distance_t>& tsp::get_symmetrized_matrix() const{
  return _symmetrized_matrix;
}

const undirected_graph<distance_t>& tsp::get_symmetrized_graph() const{
  return _symmetrized_graph;
}

const bool tsp::is_symmetric() const{
  return _is_symmetric;
}

std::size_t tsp::size(){
  return _matrix.size();
}

distance_t tsp::cost(const std::list<index_t>& tour) const{
  distance_t cost = 0;
  index_t init_step = 0;        // Initialization actually never used.

  auto step = tour.cbegin();
  if(tour.size() > 0){
    init_step = *step;
  }

  index_t previous_step = init_step;
  ++step;
  for(; step != tour.cend(); ++step){
    cost += _matrix[previous_step][*step];
    previous_step = *step;
  }
  if(tour.size() > 0){
    cost += _matrix[previous_step][init_step];
  }
  return cost;
}

distance_t tsp::symmetrized_cost(const std::list<index_t>& tour) const{
  distance_t cost = 0;
  index_t init_step = 0;        // Initialization actually never used.

  auto step = tour.cbegin();
  if(tour.size() > 0){
    init_step = *step;
  }

  index_t previous_step = init_step;
  ++step;
  for(; step != tour.cend(); ++step){
    cost += _symmetrized_matrix[previous_step][*step];
    previous_step = *step;
  }
  if(tour.size() > 0){
    cost += _symmetrized_matrix[previous_step][init_step];
  }
  return cost;
}

void tsp::get_route(const std::list<index_t>& tour,
                    rapidjson::Value& value,
                    rapidjson::Document::AllocatorType& allocator) const{
  assert(tour.size() == _matrix.size());
  return _loader->get_route(tour, value, allocator);
}

void tsp::get_tour(const std::list<index_t>& tour,
                   rapidjson::Value& value,
                   rapidjson::Document::AllocatorType& allocator) const{
  assert(tour.size() == _matrix.size());
  return _loader->get_tour(tour, value, allocator);
}

void tsp::get_route_infos(const std::list<index_t>& tour,
                          rapidjson::Document& output) const{
  assert(tour.size() == _matrix.size());

  if(_cl_args.force_start or _cl_args.force_end){
    // Open tour, getting direct geometry.
    return _loader->get_route_infos(tour, output);
  }
  else{
    // Back to the starting location when the trip is a loop.
    std::list<index_t> actual_trip (tour);
    actual_trip.push_back(actual_trip.front());
    return _loader->get_route_infos(actual_trip, output);
  }
}
