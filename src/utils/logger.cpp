/*
VROOM (Vehicle Routing Open-source Optimization Machine)
Copyright (C) 2015, Julien Coupey

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or (at
your option) any later version.

This program is distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
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
    auto start_route_geometry = std::chrono::high_resolution_clock::now();
    instance.get_route_infos(tour, output);
    auto end_route_geometry = std::chrono::high_resolution_clock::now();
    route_geometry_duration =
      std::chrono::duration_cast<std::chrono::milliseconds>
      (end_route_geometry - start_route_geometry).count();
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
}
