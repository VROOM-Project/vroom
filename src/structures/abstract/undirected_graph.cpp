/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/
#include <cassert>

#include "structures/abstract/undirected_graph.h"

template <class T> undirected_graph<T>::undirected_graph() {
}

template <class T>
undirected_graph<T>::undirected_graph(const matrix<T>& m) : _size(m.size()) {
  bool matrix_ok = true;
  _edges.reserve(_size * _size);
  _adjacency_list.reserve(_size);
  for (index_t i = 0; i < _size; ++i) {
    _adjacency_list[i].reserve(_size);
  }
  for (index_t i = 0; i < _size; ++i) {
    matrix_ok &= (m[i][i] == INFINITE_COST);
    for (index_t j = i + 1; j < _size; ++j) {
      matrix_ok &= (m[i][j] == m[j][i]);
      _edges.emplace_back(i, j, m[i][j]);
      _adjacency_list[i].push_back(j);
      _adjacency_list[j].push_back(i);
    }
  }
  assert(matrix_ok);
}

template <class T>
undirected_graph<T>::undirected_graph(std::vector<edge<T>> edges)
  : _edges{std::move(edges)} {
  for (auto const& edge : _edges) {
    index_t first = edge.get_first_vertex();
    index_t second = edge.get_second_vertex();

    _adjacency_list[first].push_back(second);
    _adjacency_list[second].push_back(first);
  }
  _size = _adjacency_list.size();
}

template <class T> std::size_t undirected_graph<T>::size() const {
  return _size;
}

template <class T> std::vector<edge<T>> undirected_graph<T>::get_edges() const {
  return _edges;
}

template <class T>
std::unordered_map<index_t, std::list<index_t>>
undirected_graph<T>::get_adjacency_list() const {
  std::unordered_map<index_t, std::list<index_t>> result;
  for (const auto& pair : _adjacency_list) {
    std::copy(pair.second.begin(),
              pair.second.end(),
              std::back_inserter(result[pair.first]));
  }
  return result;
}

template class undirected_graph<cost_t>;
