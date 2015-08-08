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

tsp::tsp(const cl_args_t& cl_args){
  std::string places = cl_args.places;
  
  // Poorly filtered for now.
  std::size_t start = 4;
  std::size_t end = places.find("&", start);
  while(end != std::string::npos){
    std::size_t separator_rank = places.find(",", start);
    std::string lat = places.substr(start,
                                    separator_rank - start);
    std::string lon = places.substr(separator_rank + 1,
                                          end - separator_rank - 1);
    _places.emplace_back(std::stod(lat, nullptr),
                         std::stod(lon, nullptr));
       
    start = end + 5;
    end = places.find("&", start);
  }
  // Adding last element, after last "&".
  end = places.length();
  std::size_t separator_rank = places.find(",", start);
  std::string lat = places.substr(start,
                                  separator_rank - start);
  std::string lon = places.substr(separator_rank + 1,
                                  end - separator_rank - 1);
  _places.emplace_back(std::stod(lat, nullptr),
                       std::stod(lon, nullptr));

  // Computing matrix.
  matrix_loader<distance_t, double>* loader;
  switch(cl_args.loader){
  case 1:
    // Using plain euclidean distance.
    loader = new euc_2d_matrix_loader();
    break;
  case 0:
    // Using OSRM.
    loader = new osrm_wrapper(cl_args.osrm_address, cl_args.osrm_port);
    break;
  default:
    // Should not happen!
    loader = new osrm_wrapper(cl_args.osrm_address, cl_args.osrm_port);
    break;
  }
  _matrix = loader->load_matrix(_places);
}

tsp::tsp(matrix<distance_t> m)
  :_matrix(m) {}

const matrix<distance_t>& tsp::get_matrix() const{
  return _matrix;
}

const std::vector<std::pair<double, double>>& tsp::get_places() const{
  return _places;
}

const matrix<distance_t> tsp::get_symmetrized_matrix() const{
  matrix<distance_t> matrix = _matrix;
  for(index_t i = 0; i < matrix.size(); ++i){
    for(index_t j = i + 1; j < matrix.size(); ++j){
      distance_t max = std::max(matrix(i, j), matrix(j, i));
      matrix.set(i, j, max);
      matrix.set(j, i, max);
    }
  }
  return matrix;
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
    cost += _matrix(previous_step, *step);
    previous_step = *step;
  }
  if(tour.size() > 0){
    cost += _matrix(previous_step, init_step);
  }
  return cost;
}

std::string tsp::get_route_summary(const std::list<index_t>& tour) const{
  // Ordering places for this tour.
  std::vector<std::pair<double, double>> ordered_places;
  for(auto step = tour.cbegin(); step != tour.cend(); ++step){
    ordered_places.push_back(_places[*step]);
  }
  // Back to the starting place.
  if(tour.size() > 0){
    ordered_places.push_back(_places[tour.front()]);
  }

  // Selected information from OSRM viaroute request.
  osrm_wrapper loader ("0.0.0.0", 5000);
  return loader.viaroute_summary(ordered_places);
}
