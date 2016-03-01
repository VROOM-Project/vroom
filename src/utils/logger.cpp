/*

This file is part of VROOM.

Copyright (c) 2015-2016, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "logger.h"

logger::logger(const cl_args_t& cl_args):
  _cl_args(cl_args) {}

void logger::write_solution(const tsp& instance,
                            const std::list<index_t>& tour,
                            const timing_t& computing_times) const{
  rapidjson::Document output;
  output.SetObject();
  rapidjson::Document::AllocatorType& allocator = output.GetAllocator();

  unsigned long route_geometry_duration = 0;
  if(_cl_args.use_osrm and _cl_args.geometry){
    // Get route informations geometry (only when using OSRM).
    BOOST_LOG_TRIVIAL(info) 
      << "[Route] Start computing detailed route.";

    auto start_route_geometry = std::chrono::high_resolution_clock::now();
    instance.get_route_infos(tour, output);
    auto end_route_geometry = std::chrono::high_resolution_clock::now();
    route_geometry_duration =
      std::chrono::duration_cast<std::chrono::milliseconds>
      (end_route_geometry - start_route_geometry).count();
    BOOST_LOG_TRIVIAL(info) << "[Route] Done, took "
                            << route_geometry_duration 
                            << " ms.";
  }

  // Timing informations.
  rapidjson::Value c_t(rapidjson::kObjectType);
  c_t.AddMember("matrix_loading", computing_times.matrix_loading, allocator);

  rapidjson::Value c_t_route(rapidjson::kObjectType);
  c_t_route.AddMember("heuristic", computing_times.heuristic, allocator);
  c_t_route.AddMember("local_search", computing_times.local_search, allocator);
  if(_cl_args.use_osrm and _cl_args.geometry){
    // Log route information timing when using OSRM.
    c_t.AddMember("detailed_geometry", route_geometry_duration, allocator);
  }
  c_t.AddMember("route", c_t_route, allocator);
  output.AddMember("computing_times", c_t, allocator);

  // Solution description.
  std::string route_type 
    = (_cl_args.force_start or _cl_args.force_end) ? "open": "loop";
  output.AddMember("route_type", rapidjson::Value(), allocator);
  output["route_type"].SetString(route_type.c_str(), route_type.size());

  output.AddMember("solution_cost", instance.cost(tour), allocator);
  // Create route and tour keys with null values and pass them as
  // references to populate them.
  output.AddMember("route", rapidjson::Value(rapidjson::kArrayType).Move(), allocator);
  instance.get_route(tour, output["route"], allocator);
  // Avoid empty "route" value (e.g. for explicit TSPLIB).
  if(output["route"].Empty()){
    output.RemoveMember("route");
  }
  output.AddMember("tour", rapidjson::Value(rapidjson::kArrayType).Move(), allocator);
  instance.get_tour(tour, output["tour"], allocator);

  rapidjson::StringBuffer s;
  rapidjson::Writer<rapidjson::StringBuffer> r_writer(s);
  output.Accept(r_writer);

  auto start_output = std::chrono::high_resolution_clock::now();
  std::string out 
    = _cl_args.output_file.empty() ? "standard output": _cl_args.output_file;
  BOOST_LOG_TRIVIAL(info) << "[Output] Write solution to "
                          << out << ".";

  if(_cl_args.output_file.empty()){
    // Log to standard output.
    std::cout << s.GetString() << std::endl;
  }
  else{
    // Log to file given as command-line option.
    std::ofstream out_stream (_cl_args.output_file, std::ofstream::out);
    out_stream << s.GetString();
    out_stream.close();
  }
  auto end_output = std::chrono::high_resolution_clock::now();
  auto output_duration =
    std::chrono::duration_cast<std::chrono::milliseconds>
    (end_output - start_output).count();

  BOOST_LOG_TRIVIAL(info) << "[Output] Done, took "
                          << output_duration  << " ms.";
}
