#ifndef OUTPUT_JSON_H
#define OUTPUT_JSON_H

/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "../include/rapidjson/include/rapidjson/document.h"
#include "structures/vroom/solution/solution.h"
#include "utils/exception.h"

namespace vroom::io {

rapidjson::Document to_json(const Solution& sol, bool report_distances);

rapidjson::Document to_json(const vroom::Exception& e);

rapidjson::Value to_json(const Summary& summary,
                         bool report_distances,
                         rapidjson::Document::AllocatorType& allocator);

rapidjson::Value to_json(const ComputingTimes& ct,
                         rapidjson::Document::AllocatorType& allocator);

rapidjson::Value to_json(const Route& route,
                         bool report_distances,
                         rapidjson::Document::AllocatorType& allocator);

rapidjson::Value to_json(const Step& s,
                         bool report_distances,
                         rapidjson::Document::AllocatorType& allocator);

rapidjson::Value to_json(const Location& loc,
                         rapidjson::Document::AllocatorType& allocator);

void write_to_json(const vroom::Exception& e,
                   const std::string& output_file = "");

void write_to_json(const Solution& sol,
                   const std::string& output_file = "",
                   bool report_distances = false);
} // namespace vroom::io

#endif
