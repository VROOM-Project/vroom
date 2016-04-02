/*

This file is part of VROOM.

Copyright (c) 2015-2016, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "tsp.h"

tsp::tsp(const problem_io<distance_t>& loader,
         bool force_start,
         bool force_end):
  _matrix(loader.get_matrix()),
  _symmetrized_matrix(0),
  _is_symmetric(true),
  _force_start(force_start),
  _force_end(force_end){
  
  // Distances on the diagonal are never used except in the minimum
  // weight perfect matching during the heuristic. This makes sure
  // each node will be impossible to match with itself at that time.
  for(index_t i = 0; i < _matrix.size(); ++i){
    _matrix[i][i] = INFINITE_DISTANCE;
  }

  // Dealing with open tour cases. At most one of the following
  // occurs.
  if(_force_start and !_force_end){
    // Forcing first location as start, end location decided during
    // optimization.
    for(index_t i = 1; i < _matrix.size(); ++i){
      _matrix[i][0] = 0;
    }
  }
  if(!_force_start and _force_end){
    // Forcing last location as end, start location decided during
    // optimization.
    index_t last_index = _matrix.size() - 1;
    for(index_t j = 0; j < last_index; ++j){
      _matrix[last_index][j] = 0;
    }
  }
  if(_force_start and _force_end){
    // Forcing first location as start, last location as end to
    // produce an open tour.
    index_t last_index = _matrix.size() - 1;
    _matrix[last_index][0] = 0;
    for(index_t j = 1; j < last_index; ++j){
      _matrix[last_index][j] = (std::numeric_limits<distance_t>::max() / 2);
    }
  }

  // Compute symmetrized matrix and update _is_symmetric flag.
  const distance_t& (*sym_f) (const distance_t&, const distance_t&) 
    = std::min<distance_t>;
  if((_force_start and !_force_end)
     or (!_force_start and _force_end)){
    // Using symmetrization with max as when only start or only end is
    // forced, the matrix has a line or a column filled with zeros.
    sym_f = std::max<distance_t>;
  }
  matrix<distance_t> m {_matrix.size()};
  for(index_t i = 0; i < m.size(); ++i){
    m[i][i] = _matrix[i][i];
    for(index_t j = i + 1; j < m.size(); ++j){
      _is_symmetric &= (_matrix[i][j] == _matrix[j][i]);
      distance_t val = sym_f(_matrix[i][j], _matrix[j][i]);
      m[i][j] = val;
      m[j][i] = val;
    }
  }
  _symmetrized_matrix = m;
  
  // Compute graph for symmetrized problem.
  _symmetrized_graph = undirected_graph<distance_t>(_symmetrized_matrix);
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

const bool tsp::force_start() const{
  return _force_start;
}

const bool tsp::force_end() const{
  return _force_end;
}

std::size_t tsp::size() const{
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
