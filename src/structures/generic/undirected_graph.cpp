/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <algorithm>
#include <cassert>
#include <iterator>

#include "structures/generic/undirected_graph.h"

namespace vroom::utils {

template <class T> UndirectedGraph<T>::UndirectedGraph() = default;

template <class T>
UndirectedGraph<T>::UndirectedGraph(const Matrix<T>& m) : _size(m.size()) {
#ifndef NDEBUG
  bool matrix_ok = true;
#endif
  _edges.reserve(_size * _size);
  _adjacency_list.reserve(_size);
  for (Index i = 0; i < _size; ++i) {
    _adjacency_list[i].reserve(_size);
  }
  for (Index i = 0; i < _size; ++i) {
#ifndef NDEBUG
    matrix_ok = matrix_ok && (m[i][i] == INFINITE_USER_COST);
#endif
    for (Index j = i + 1; j < _size; ++j) {
#ifndef NDEBUG
      matrix_ok = matrix_ok && (m[i][j] == m[j][i]);
#endif
      _edges.emplace_back(i, j, m[i][j]);
      _adjacency_list[i].push_back(j);
      _adjacency_list[j].push_back(i);
    }
  }
  assert(matrix_ok);
}

template <class T>
UndirectedGraph<T>::UndirectedGraph(std::vector<Edge<T>>&& edges)
  : _edges(std::move(edges)) {
  for (auto const& edge : _edges) {
    const Index first = edge.get_first_vertex();
    const Index second = edge.get_second_vertex();

    _adjacency_list[first].push_back(second);
    _adjacency_list[second].push_back(first);
  }
  _size = _adjacency_list.size();
}

template <class T> std::size_t UndirectedGraph<T>::size() const {
  return _size;
}

template <class T> std::vector<Edge<T>> UndirectedGraph<T>::get_edges() const {
  return _edges;
}

template <class T>
std::unordered_map<Index, std::list<Index>>
UndirectedGraph<T>::get_adjacency_list() const {
  std::unordered_map<Index, std::list<Index>> result;
  for (const auto& [index, vector] : _adjacency_list) {
    std::ranges::copy(vector, std::back_inserter(result[index]));
  }
  return result;
}

template class UndirectedGraph<UserCost>;

} // namespace vroom::utils
