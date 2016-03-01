#ifndef UNDIRECTED_GRAPH_H
#define UNDIRECTED_GRAPH_H

/*

This file is part of VROOM.

Copyright (c) 2015-2016, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <vector>
#include <unordered_map>
#include <cassert>
#include "edge.h"
#include "matrix.h"

template <class T> class undirected_graph{

private:
  unsigned _size;
  // Embedding two representations for different uses depending on
  // context.
  std::vector<edge<T>> _edges;
  std::unordered_map<index_t, std::list<index_t>> _adjacency_list;

public:
  undirected_graph() {}
  
  undirected_graph(const matrix<T>& m):
    _size(m.size())
  {
    bool matrix_ok = true;
    for(index_t i = 0; i < _size; ++i){
      matrix_ok &= (m[i][i] == INFINITE_DISTANCE);
      for(index_t j = i + 1; j < _size; ++j){
        matrix_ok &= (m[i][j] == m[j][i]);
        _edges.emplace_back(i, j, m[i][j]);
        _adjacency_list[i].push_back(j);
        _adjacency_list[j].push_back(i);
      }
    }
    assert(matrix_ok);
  }

  undirected_graph(std::vector<edge<T>> edges):
    _edges{std::move(edges)}{
    for(auto const& edge: _edges){
      index_t first = edge.get_first_vertex();
      index_t second = edge.get_second_vertex();
    
      _adjacency_list[first].push_back(second);
      _adjacency_list[second].push_back(first);
    }
    _size = _adjacency_list.size();
  }

  std::size_t size() const{
    return _size;
  }

  std::vector<edge<T>> get_edges() const{
    return _edges;
  }

  std::unordered_map<index_t, std::list<index_t>> get_adjacency_list() const{
    return _adjacency_list;
  }
};

#endif
