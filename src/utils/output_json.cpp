/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <fstream>
#include <iostream>

#include "../include/rapidjson/stringbuffer.h"
#include "../include/rapidjson/writer.h"

#include "utils/output_json.h"
#include "utils/version.h"

namespace vroom {
namespace io {

rapidjson::Document to_json(const Solution& sol, bool geometry) {
  rapidjson::Document json_output;
  json_output.SetObject();
  rapidjson::Document::AllocatorType& allocator = json_output.GetAllocator();

  json_output.AddMember("code", sol.code, allocator);
  if (sol.code != 0) {
    json_output.AddMember("error", rapidjson::Value(), allocator);
    json_output["error"].SetString(sol.error.c_str(), sol.error.size());
  } else {
    json_output.AddMember("summary",
                          to_json(sol.summary, geometry, allocator),
                          allocator);

    rapidjson::Value json_unassigned(rapidjson::kArrayType);
    for (const auto& job : sol.unassigned) {
      rapidjson::Value json_job(rapidjson::kObjectType);
      json_job.AddMember("id", job.id, allocator);
      if (job.location.has_coordinates()) {
        json_job.AddMember("location",
                           to_json(job.location, allocator),
                           allocator);
      }
      json_unassigned.PushBack(json_job, allocator);
    }

    json_output.AddMember("unassigned", json_unassigned, allocator);

    rapidjson::Value json_routes(rapidjson::kArrayType);
    for (const auto& route : sol.routes) {
      json_routes.PushBack(to_json(route, geometry, allocator), allocator);
    }

    json_output.AddMember("routes", json_routes, allocator);
  }

  return json_output;
}

rapidjson::Value to_json(const Summary& summary,
                         bool geometry,
                         rapidjson::Document::AllocatorType& allocator) {
  rapidjson::Value json_summary(rapidjson::kObjectType);

  json_summary.AddMember("cost", summary.cost, allocator);
  json_summary.AddMember("unassigned", summary.unassigned, allocator);

  if (summary.amount.size() > 0) {
    rapidjson::Value json_amount(rapidjson::kArrayType);
    for (std::size_t i = 0; i < summary.amount.size(); ++i) {
      json_amount.PushBack(summary.amount[i], allocator);
    }
    json_summary.AddMember("amount", json_amount, allocator);
  }

  json_summary.AddMember("service", summary.service, allocator);
  json_summary.AddMember("duration", summary.duration, allocator);
  json_summary.AddMember("waiting_time", summary.waiting_time, allocator);

  if (geometry) {
    json_summary.AddMember("distance", summary.distance, allocator);
  }

  json_summary.AddMember("computing_times",
                         to_json(summary.computing_times, geometry, allocator),
                         allocator);

  return json_summary;
}

rapidjson::Value to_json(const Route& route,
                         bool geometry,
                         rapidjson::Document::AllocatorType& allocator) {
  rapidjson::Value json_route(rapidjson::kObjectType);

  json_route.AddMember("vehicle", route.vehicle, allocator);
  json_route.AddMember("cost", route.cost, allocator);

  if (route.amount.size() > 0) {
    rapidjson::Value json_amount(rapidjson::kArrayType);
    for (std::size_t i = 0; i < route.amount.size(); ++i) {
      json_amount.PushBack(route.amount[i], allocator);
    }
    json_route.AddMember("amount", json_amount, allocator);
  }

  json_route.AddMember("service", route.service, allocator);
  json_route.AddMember("duration", route.duration, allocator);
  json_route.AddMember("waiting_time", route.waiting_time, allocator);

  if (geometry) {
    json_route.AddMember("distance", route.distance, allocator);
  }

  rapidjson::Value json_steps(rapidjson::kArrayType);
  for (const auto& step : route.steps) {
    json_steps.PushBack(to_json(step, geometry, allocator), allocator);
  }

  json_route.AddMember("steps", json_steps, allocator);

  if (!route.geometry.empty()) {
    json_route.AddMember("geometry", rapidjson::Value(), allocator);
    json_route["geometry"].SetString(route.geometry.c_str(),
                                     route.geometry.size());
  }

  return json_route;
}

rapidjson::Value to_json(const ComputingTimes& ct,
                         bool geometry,
                         rapidjson::Document::AllocatorType& allocator) {
  rapidjson::Value json_ct(rapidjson::kObjectType);

  json_ct.AddMember("loading", ct.loading, allocator);
  json_ct.AddMember("solving", ct.solving, allocator);

  if (geometry) {
    // Log route information timing when using OSRM.
    json_ct.AddMember("routing", ct.routing, allocator);
  }

  return json_ct;
}

rapidjson::Value to_json(const Step& s,
                         bool geometry,
                         rapidjson::Document::AllocatorType& allocator) {
  rapidjson::Value json_step(rapidjson::kObjectType);

  json_step.AddMember("type", rapidjson::Value(), allocator);
  std::string str_type;
  switch (s.type) {
  case STEP_TYPE::START:
    str_type = "start";
    break;
  case STEP_TYPE::END:
    str_type = "end";
    break;
  case STEP_TYPE::JOB:
    str_type = "job";
    break;
  }
  json_step["type"].SetString(str_type.c_str(), str_type.size(), allocator);

  if (s.location.has_coordinates()) {
    json_step.AddMember("location", to_json(s.location, allocator), allocator);
  }

  if (s.type == STEP_TYPE::JOB) {
    json_step.AddMember("job", s.job, allocator);
    json_step.AddMember("service", s.service, allocator);
    json_step.AddMember("waiting_time", s.waiting_time, allocator);
  }

  json_step.AddMember("arrival", s.arrival, allocator);
  json_step.AddMember("duration", s.duration, allocator);

  if (geometry) {
    json_step.AddMember("distance", s.distance, allocator);
  }

  return json_step;
}

rapidjson::Value to_json(const Location& loc,
                         rapidjson::Document::AllocatorType& allocator) {
  rapidjson::Value json_coords(rapidjson::kArrayType);

  json_coords.PushBack(loc.lon(), allocator);
  json_coords.PushBack(loc.lat(), allocator);

  return json_coords;
}

void write_to_json(const Solution& sol,
                   bool geometry,
                   const std::string& output_file) {
  auto json_output = to_json(sol, geometry);

  // Rapidjson writing process.
  rapidjson::StringBuffer s;
  rapidjson::Writer<rapidjson::StringBuffer> r_writer(s);
  json_output.Accept(r_writer);

  // Write to relevant output.
  if (output_file.empty()) {
    // Log to standard output.
    std::cout << s.GetString() << std::endl;
  } else {
    // Log to file.
    std::ofstream out_stream(output_file, std::ofstream::out);
    out_stream << s.GetString();
    out_stream.close();
  }
}

} // namespace io
} // namespace vroom
