#ifndef KRUSKAL_H
#define KRUSKAL_H

/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/generic/undirected_graph.h"

namespace vroom::utils {

template <class T>
UndirectedGraph<T> minimum_spanning_tree(const UndirectedGraph<T>& graph);

} // namespace vroom::utils

#endif
