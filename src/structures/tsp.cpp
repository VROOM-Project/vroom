/*

This file is part of VROOM.

Copyright (c) 2015-2016, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "tsp.h"

tsp::tsp(const pbl_context_t& pbl_context, const matrix<distance_t>& m):
  _pbl_context(pbl_context),
  _matrix(m),
  _symmetrized_matrix(0),
  _is_symmetric(true){

  if(_pbl_context.force_start){
    assert(_pbl_context.start < _matrix.size());
  }
  if(_pbl_context.force_end){
    assert(_pbl_context.end < _matrix.size());
  }

  // Distances on the diagonal are never used except in the minimum
  // weight perfect matching during the heuristic. This makes sure
  // each node will be impossible to match with itself at that time.
  for(index_t i = 0; i < _matrix.size(); ++i){
    _matrix[i][i] = INFINITE_DISTANCE;
  }

  // Dealing with open tour cases. At most one of the following
  // occurs.
  if(_pbl_context.force_start and !_pbl_context.force_end){
    // Forcing first location as start, end location decided during
    // optimization.
    for(index_t i = 0; i < _matrix.size(); ++i){
      if(i != _pbl_context.start){
        _matrix[i][_pbl_context.start] = 0;
      }
    }
  }
  if(!_pbl_context.force_start and _pbl_context.force_end){
    // Forcing last location as end, start location decided during
    // optimization.
    for(index_t j = 0; j < _matrix.size(); ++j){
      if(j != _pbl_context.end){
        _matrix[_pbl_context.end][j] = 0;
      }
    }
  }
  if(_pbl_context.force_start and _pbl_context.force_end){
    // Forcing first location as start, last location as end to
    // produce an open tour.
    assert(_pbl_context.start != _pbl_context.end);
    index_t last_index = _matrix.size() - 1;
    _matrix[_pbl_context.end][_pbl_context.start] = 0;
    for(index_t j = 1; j < last_index; ++j){
      if((j != _pbl_context.start) and (j != _pbl_context.end)){
        _matrix[_pbl_context.end][j] = INFINITE_DISTANCE;
      }
    }
  }

  // Compute symmetrized matrix and update _is_symmetric flag.
  const distance_t& (*sym_f) (const distance_t&, const distance_t&) 
    = std::min<distance_t>;
  if((_pbl_context.force_start and !_pbl_context.force_end)
     or (!_pbl_context.force_start and _pbl_context.force_end)){
    // Using symmetrization with max as when only start or only end is
    // forced, the matrix has a line or a column filled with zeros.
    sym_f = std::max<distance_t>;
  }
  matrix<distance_t> mat {_matrix.size()};
  for(index_t i = 0; i < mat.size(); ++i){
    mat[i][i] = _matrix[i][i];
    for(index_t j = i + 1; j < mat.size(); ++j){
      _is_symmetric &= (_matrix[i][j] == _matrix[j][i]);
      distance_t val = sym_f(_matrix[i][j], _matrix[j][i]);
      mat[i][j] = val;
      mat[j][i] = val;
    }
  }
  _symmetrized_matrix = mat;
  
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

bool tsp::is_symmetric() const{
  return _is_symmetric;
}

bool tsp::force_start() const{
  return _pbl_context.force_start;
}

index_t tsp::get_start() const{
  return _pbl_context.start;
}

bool tsp::force_end() const{
  return _pbl_context.force_end;
}

index_t tsp::get_end() const{
  return _pbl_context.end;
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
