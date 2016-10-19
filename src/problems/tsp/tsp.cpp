/*

This file is part of VROOM.

Copyright (c) 2015-2016, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "tsp.h"

tsp::tsp(const std::vector<job>& jobs,
         const std::vector<vehicle>& vehicles,
         matrix<distance_t> matrix):
  vrp(jobs, vehicles, matrix)
  _symmetrized_matrix(_matrix.size()),
  _is_symmetric(true),
  _force_start(_vehicles[0].has_start()),
  _force_end(_vehicles[0].has_end()),
{
  if(_force_start){
    _start = _vehicles[0].start.get().index;
    assert(_start < _matrix.size());
  }
  if(_force_end){
    _end = _vehicles[0].end.get().index;
    assert(_end < _matrix.size());
  }

  // Distances on the diagonal are never used except in the minimum
  // weight perfect matching during the heuristic. This makes sure
  // each node will be impossible to match with itself at that time.

  // TODO change munkres implementation to avoid having to do this
  // here.
  for(index_t i = 0; i < _matrix.size(); ++i){
    _matrix[i][i] = INFINITE_DISTANCE;
  }

  // Dealing with open tour cases. At most one of the following
  // occurs.
  if(_force_start and !_force_end){
    // Forcing first location as start, end location decided during
    // optimization.
    for(index_t i = 0; i < _matrix.size(); ++i){
      if(i != _start){
        _matrix[i][_start] = 0;
      }
    }
  }
  if(!_force_start and _force_end){
    // Forcing last location as end, start location decided during
    // optimization.
    for(index_t j = 0; j < _matrix.size(); ++j){
      if(j != _end){
        _matrix[_end][j] = 0;
      }
    }
  }
  if(_force_start and _force_end){
    // Forcing first location as start, last location as end to
    // produce an open tour.
    assert(_start != _end);
    _matrix[_end][_start] = 0;
    for(index_t j = 0; j < _matrix.size(); ++j){
      if((j != _start) and (j != _end)){
        _matrix[_end][j] = INFINITE_DISTANCE;
      }
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
  for(index_t i = 0; i < matrix.size(); ++i){
    _symmetrized_matrix[i][i] = _matrix[i][i];
    for(index_t j = i + 1; j < matrix.size(); ++j){
      _is_symmetric &= (_matrix[i][j] == _matrix[j][i]);
      distance_t val = sym_f(_matrix[i][j], _matrix[j][i]);
      _symmetrized_matrix[i][j] = val;
      _symmetrized_matrix[j][i] = val;
    }
  }

  // Compute graph for symmetrized problem.
  _symmetrized_graph = undirected_graph<distance_t>(_symmetrized_matrix);
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
