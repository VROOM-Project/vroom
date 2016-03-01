#ifndef LOGGER_H
#define LOGGER_H

/*

This file is part of VROOM.

Copyright (c) 2015-2016, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <iostream>
#include <fstream>
#include <string>
#include <list>
#include <chrono>
#include <boost/log/trivial.hpp>
#include "../../include/rapidjson/document.h"
#include "../../include/rapidjson/writer.h"
#include "../../include/rapidjson/stringbuffer.h"
#include "../structures/typedefs.h"
#include "../structures/tsp.h"

class logger{
private:
  cl_args_t _cl_args;
  
public:
  logger(const cl_args_t& cl_args);
  
  void write_solution(const tsp& instance,
                      const std::list<index_t>& tour,
                      const timing_t& computing_times) const;
};

#endif
