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

#ifndef UNDIRECTED_GRAPH_H
#define UNDIRECTED_GRAPH_H
#include <list>
#include <unordered_map>
#include <cassert>
#include "edge.h"
#include "matrix.h"

template <class T> class undirected_graph{

private:
  unsigned _size;
  // Embedding two representations for different uses depending on
  // context.
  std::list<edge<T>> _edges;
  std::unordered_map<index_t, std::list<index_t>> _adjacency_list;

public:
  undirected_graph() {}
  
  undirected_graph(const matrix<T>& m):
    _size(m.size())
  {
    bool matrix_ok = true;
    for(index_t i = 0; i < _size; ++i){
      matrix_ok &= (m[i][i] == 0);
      for(index_t j = i + 1; j < _size; ++j){
        matrix_ok &= (m[i][j] == m[j][i]);
        _edges.emplace_front(i, j, m[i][j]);
        _adjacency_list[i].push_back(j);
        _adjacency_list[j].push_back(i);
      }
    }
    assert(matrix_ok);
  }

  undirected_graph(std::list<edge<T>> edges):
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

  std::list<edge<T>> get_edges() const{
    return _edges;
  }

  std::unordered_map<index_t, std::list<index_t>> get_adjacency_list() const{
    return _adjacency_list;
  }

  void print_edges() const{
    for(auto const& edge: _edges){
      edge.log();
      std::cout << " ; ";
    }
    std::cout << std::endl;
  }

  void print_adjacency_list() const{
    for(auto const& adj: _adjacency_list){
      std::cout << adj.first << "->(";
      for(auto& element: adj.second){
        std::cout << element << " ; ";
      }
      std::cout << ") \n";
    }
  }

};

#endif
