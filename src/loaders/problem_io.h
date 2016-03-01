#ifndef PROBLEM_IO_H
#define PROBLEM_IO_H

/*

This file is part of VROOM.

Copyright (c) 2015-2016, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include<string>
#include<vector>
#include "../../include/rapidjson/document.h"
#include "../structures/matrix.h"

template <class T> 
class problem_io{

public:
  virtual matrix<T> get_matrix() const = 0;

  virtual void get_route(const std::list<index_t>& tour,
                         rapidjson::Value& value,
                         rapidjson::Document::AllocatorType& allocator) const = 0;

  virtual void get_tour(const std::list<index_t>& tour,
                        rapidjson::Value& value,
                        rapidjson::Document::AllocatorType& allocator) const = 0;

  virtual void get_route_infos(const std::list<index_t>& tour,
                               rapidjson::Document& output) const = 0;

  virtual ~problem_io() {}

protected:
  problem_io() {}
};

#endif
