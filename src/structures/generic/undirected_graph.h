#ifndef UNDIRECTED_GRAPH_H
#define UNDIRECTED_GRAPH_H

/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <unordered_map>

#include "structures/generic/edge.h"
#include "structures/generic/matrix.h"

namespace vroom::utils {

template <class T> class UndirectedGraph {

private:
  unsigned _size;
  // Embedding two representations for different uses depending on
  // context.
  std::vector<Edge<T>> _edges;
  std::unordered_map<Index, std::vector<Index>> _adjacency_list;

public:
  UndirectedGraph();

  explicit UndirectedGraph(const Matrix<T>& m);

  explicit UndirectedGraph(std::vector<Edge<T>>&& edges);

  std::size_t size() const;

  std::vector<Edge<T>> get_edges() const;

  std::unordered_map<Index, std::list<Index>> get_adjacency_list() const;
};

} // namespace vroom::utils

#endif
