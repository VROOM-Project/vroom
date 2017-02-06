#ifndef OUTPUT_JSON_H
#define OUTPUT_JSON_H

/*

This file is part of VROOM.

Copyright (c) 2015-2016, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <iostream>
#include <fstream>
#include <string>
#include <chrono>
#include <boost/log/trivial.hpp>
#include "../../include/rapidjson/document.h"
#include "../../include/rapidjson/writer.h"
#include "../../include/rapidjson/stringbuffer.h"
#include "../structures/vroom/solution/solution.h"
#include "./version.h"

void write_to_json(const solution& sol,
                   bool geometry,
                   const std::string& output_file);

#endif
