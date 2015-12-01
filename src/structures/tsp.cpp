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
  _cl_args(cl_args){
  
  // Computing matrix with the right tool.
  assert((!cl_args.use_osrm) or (!cl_args.use_tsplib));
  
  // Exactly one of the two following is true.
  if(cl_args.use_osrm){
    _loader 
      = std::make_unique<osrm_wrapper>(cl_args.osrm_address, 
                                       cl_args.osrm_port,
                                       cl_args.input);
  }
  if(cl_args.use_tsplib){
    _loader = std::make_unique<tsplib_loader>(cl_args.input);
  }

  _matrix = _loader->get_matrix();

  // Dealing with open tour cases. At most one of the following
  // occurs.
  if(cl_args.force_start and !cl_args.force_end){
    // Forcing first location as start, end location decided during
    // optimization.
    for(index_t i = 1; i < _matrix.size(); ++i){
      _matrix[i][0] = 0;
    }
  }
  if(!cl_args.force_start and cl_args.force_end){
    // Forcing last location as end, start location decided during
    // optimization.
    index_t last_index = _matrix.size() - 1;
    for(index_t j = 0; j < last_index; ++j){
      _matrix[last_index][j] = 0;
    }
  }
  if(cl_args.force_start and cl_args.force_end){
    // Forcing first location as start, last location as end to
    // produce an open tour.
    index_t last_index = _matrix.size() - 1;
    _matrix[last_index][0] = 0;
    for(index_t j = 1; j < last_index; ++j){
      _matrix[last_index][j] = (std::numeric_limits<distance_t>::max() / 2);
    }
  }

  _is_symmetric = _matrix.is_symmetric();
}

tsp::tsp(const matrix<distance_t>& m):
  _matrix(m),
  _is_symmetric(_matrix.is_symmetric()) {}

const matrix<distance_t>& tsp::get_matrix() const{
  return _matrix;
}

const matrix<distance_t> tsp::get_symmetrized_matrix() const{
  if(_is_symmetric){
    return _matrix;
  }
  else{
    const distance_t& (*sym_f) (const distance_t&, const distance_t&) 
      = std::min<distance_t>;
    if((_cl_args.force_start and !_cl_args.force_end)
       or (!_cl_args.force_start and _cl_args.force_end)){
      // Using symmetrization with max as when only start or only end
      // is forced, the matrix has a line or a column filled with
      // zeros.
      sym_f = std::max<distance_t>;
    }
    matrix<distance_t> m {_matrix.size()};
    for(index_t i = 0; i < m.size(); ++i){
      m[i][i] = _matrix[i][i];
      for(index_t j = i + 1; j < m.size(); ++j){
        distance_t val = sym_f(_matrix[i][j], _matrix[j][i]);
        m[i][j] = val;
        m[j][i] = val;
      }
    }
    return m;
  }
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

std::string tsp::get_route(const std::list<index_t>& tour) const{
  std::string type = "\"route_type\":";
  if(_cl_args.force_start or _cl_args.force_end){
    type += "\"open\",";
  }
  else{
    type += "\"loop\",";
  }
  return type + _loader->get_route(tour);
}

std::string tsp::get_route_geometry(const std::list<index_t>& tour) const{
  assert(tour.size() >= 2);

  if(_cl_args.force_start or _cl_args.force_end){
    // Open tour, getting direct geometry.
    return _loader->get_route_geometry(tour);
  }
  else{
    // Back to the starting location when the trip is a loop.
    std::list<index_t> actual_trip (tour);
    actual_trip.push_back(actual_trip.front());
    return _loader->get_route_geometry(actual_trip);
  }
}
