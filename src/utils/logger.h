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

void write_solution(const cl_args_t& cl_args,
                    const problem_io<distance_t>& loader,
                    const tsp& instance,
                    const std::list<index_t>& tour,
                    distance_t sol_cost,                    
                    const timing_t& computing_times);

#endif
