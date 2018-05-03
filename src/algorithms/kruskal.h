#ifndef KRUSKAL_H
#define KRUSKAL_H

/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "../structures/abstract/undirected_graph.h"

template <class T>
undirected_graph<T> minimum_spanning_tree(const undirected_graph<T>& graph);

#endif
