/*

This file is part of VROOM.

Copyright (c) 2015-2016, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "logger.h"

void write_solution(const cl_args_t& cl_args,
                    const problem_io<distance_t>& loader,
                    const std::list<index_t>& steps,
                    distance_t sol_cost,
                    const timing_t& computing_times){
  rapidjson::Document json_output;
  json_output.SetObject();
  rapidjson::Document::AllocatorType& allocator = json_output.GetAllocator();
  rapidjson::Value json_route(rapidjson::kObjectType);

  duration_t route_geometry_duration = 0;
  if(cl_args.use_osrm and cl_args.geometry){
    // Get route informations geometry (only when using OSRM).
    BOOST_LOG_TRIVIAL(info)
      << "[Route] Start computing detailed route.";

    auto start_route_geometry = std::chrono::high_resolution_clock::now();
    loader.get_route_infos(steps, json_route, allocator);
    auto end_route_geometry = std::chrono::high_resolution_clock::now();
    route_geometry_duration =
      std::chrono::duration_cast<std::chrono::milliseconds>
      (end_route_geometry - start_route_geometry).count();
    BOOST_LOG_TRIVIAL(info) << "[Route] Done, took "
                            << route_geometry_duration
                            << " ms.";
  }

  // Set output.
  auto start_output = std::chrono::high_resolution_clock::now();
  std::string out
    = cl_args.output_file.empty() ? "standard output": cl_args.output_file;
  BOOST_LOG_TRIVIAL(info) << "[Output] Write solution to "
                          << out << ".";

  // Timing informations.
  rapidjson::Value json_c_t(rapidjson::kObjectType);
  json_c_t.AddMember("loading", computing_times.matrix_loading, allocator);

  rapidjson::Value json_c_t_solving(rapidjson::kObjectType);
  json_c_t_solving.AddMember("heuristic", computing_times.heuristic, allocator);
  json_c_t_solving.AddMember("local_search", computing_times.local_search, allocator);
  json_c_t.AddMember("solving", json_c_t_solving, allocator);

  if(cl_args.use_osrm and cl_args.geometry){
    // Log route information timing when using OSRM.
    json_c_t.AddMember("routing", route_geometry_duration, allocator);
  }

  // Create "steps" key as an empty array and pass it as a reference
  // for step objects additions.
  json_route.AddMember("steps", rapidjson::Value(rapidjson::kArrayType).Move(), allocator);
  loader.get_steps(steps, json_route["steps"], allocator);
  json_route.AddMember("cost", sol_cost, allocator);
  json_route.AddMember("vehicle", loader.get_vehicle_id(), allocator);

  // Routes.
  rapidjson::Value json_routes_array(rapidjson::kArrayType);
  json_routes_array.PushBack(json_route, allocator);

  // Global indicators.
  distance_t global_cost = 0;
  distance_t global_duration = 0;
  distance_t global_distance = 0;

  for(rapidjson::SizeType i = 0; i < json_routes_array.Size(); ++i){
    global_cost += json_routes_array[i]["cost"].GetUint();
    if(cl_args.use_osrm and cl_args.geometry){
      global_duration += json_routes_array[i]["duration"].GetUint();
      global_distance += json_routes_array[i]["distance"].GetUint();
    }
  }

  // Solution description.
  rapidjson::Value json_solution(rapidjson::kObjectType);
  json_solution.AddMember("computing_times", json_c_t, allocator);
  if(cl_args.use_osrm and cl_args.geometry){
    json_solution.AddMember("distance", global_distance, allocator);
    json_solution.AddMember("duration", global_duration, allocator);
  }
  json_solution.AddMember("cost", global_cost, allocator);

  json_output.AddMember("routes", json_routes_array, allocator);
  json_output.AddMember("solution", json_solution, allocator);
  json_output.AddMember("code", rapidjson::Value(0), allocator);

  rapidjson::StringBuffer s;
  rapidjson::Writer<rapidjson::StringBuffer> r_writer(s);
  json_output.Accept(r_writer);

  // Write to relevant output.
  if(cl_args.output_file.empty()){
    // Log to standard output.
    std::cout << s.GetString() << std::endl;
  }
  else{
    // Log to file given as command-line option.
    std::ofstream out_stream (cl_args.output_file, std::ofstream::out);
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

void write_error(const std::string& output,
                 const std::string& message){
  rapidjson::Document json_output;
  json_output.SetObject();
  rapidjson::Document::AllocatorType& allocator = json_output.GetAllocator();

  json_output.AddMember("code", rapidjson::Value(1), allocator);

  json_output.AddMember("error", rapidjson::Value(), allocator);
  json_output["error"].SetString(message.c_str(), message.size());

  rapidjson::StringBuffer s;
  rapidjson::Writer<rapidjson::StringBuffer> r_writer(s);
  json_output.Accept(r_writer);

  // Write to relevant output.
  if(output.empty()){
    // Log to standard output.
    std::cout << s.GetString() << std::endl;
  }
  else{
    // Log to file given as command-line option.
    std::ofstream out_stream (output, std::ofstream::out);
    out_stream << s.GetString();
    out_stream.close();
  }
}
