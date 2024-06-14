/*

This file is part of VROOM.

Copyright (c) 2015-2024, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <fstream>
#include <iostream>

#include "../include/rapidjson/include/rapidjson/stringbuffer.h"
#include "../include/rapidjson/include/rapidjson/writer.h"

#include "utils/output_json.h"

namespace vroom::io {

inline boost::json::value get_violations(const Violations& violations) {
  boost::json::array json_violations;

  for (const auto type : violations.types) {
    boost::json::object json_violation;
    std::string cause;
    switch (type) {
      using enum VIOLATION;
    case LEAD_TIME:
      cause = "lead_time";
      json_violation["duration"] = violations.lead_time;
      break;
    case DELAY:
      cause = "delay";
      json_violation["duration"] = violations.delay;
      break;
    case LOAD:
      cause = "load";
      break;
    case MAX_TASKS:
      cause = "max_tasks";
      break;
    case SKILLS:
      cause = "skills";
      break;
    case PRECEDENCE:
      cause = "precedence";
      break;
    case MISSING_BREAK:
      cause = "missing_break";
      break;
    case MAX_TRAVEL_TIME:
      cause = "max_travel_time";
      break;
    case MAX_LOAD:
      cause = "max_load";
      break;
    case MAX_DISTANCE:
      cause = "max_distance";
      break;
    default:
      assert(false);
    }

    json_violation["cause"] = cause;
    json_violations.emplace_back(json_violation);
  }

  return json_violations;
}

boost::json::object to_json(const Solution& sol, bool report_distances) {
  boost::json::object json_output;
  
  json_output["code"] = 0;
  json_output["summary"] = to_json(sol.summary, report_distances);

  boost::json::array json_unassigned;
  for (const auto& job : sol.unassigned) {
    boost::json::object json_job;
    json_job["id"] = job.id;
    if (job.location.has_coordinates()) {
      json_job["location"] = to_json(job.location);
    }
    if (job.location.user_index()) {
      json_job["location_index"] = job.location.index();
    }
    json_job["type"] = rapidjson::Value();
    std::string str_type;
    switch (job.type) {
      using enum JOB_TYPE;
    case SINGLE:
      str_type = "job";
      break;
    case PICKUP:
      str_type = "pickup";
      break;
    case DELIVERY:
      str_type = "delivery";
      break;
    }
    json_job["type"] = str_type;

    if (!job.description.empty()) {
      json_job["description"] = job.description;
    }

    json_unassigned.emplace_back(json_job);
  }

  json_output["unassigned"] = json_unassigned;

  boost::json::array json_routes;
  for (const auto& route : sol.routes) {
    json_routes.emplace_back(to_json(route, report_distances);
  }

  json_output["routes"] = json_routes;
  return json_output;
}

boost::json::object to_json(const vroom::Exception& e) {
  boost::json::object json_output;
  
  json_output["code"] = e.error_code;
  json_output["error"] = e.message;
  return json_output;
}

boost::json::object to_json(const Summary& summary,
                         bool report_distances) {
  boost::json::object json_summary;

  json_summary["cost"] = summary.cost;
  json_summary["routes"] = summary.routes;
  json_summary["unassigned"] = summary.unassigned;

  if (!summary.delivery.empty()) {
    boost::json::array json_delivery;
    for (std::size_t i = 0; i < summary.delivery.size(); ++i) {
      json_delivery.emplace_back(summary.delivery[i]);
    }
    json_summary["delivery"] = json_delivery;

    // Support for deprecated "amount" key.
    json_summary["amount"] =  json_delivery;
  }

  if (!summary.pickup.empty()) {
    boost::json::array json_pickup;
    for (std::size_t i = 0; i < summary.pickup.size(); ++i) {
      json_pickup.emplace_back(summary.pickup[i]);
    }
    json_summary["pickup"] = json_pickup;
  }

  json_summary["setup"] = summary.setup;
  json_summary["service"] = summary.service;
  json_summary["duration"] = summary.duration;
  json_summary["waiting_time"] = summary.waiting_time;
  json_summary["priority"] = summary.priority;

  if (report_distances) {
    json_summary["distance"] = summary.distance;
  }

  json_summary["violations"] = get_violations(summary.violations);

  json_summary["computing_times"] = to_json(summary.computing_times);

  return json_summary;
}

rapidjson::Value to_json(const Route& route,
                         bool report_distances,
                         rapidjson::Document::AllocatorType& allocator) {
  rapidjson::Value json_route(rapidjson::kObjectType);

  json_route.AddMember("vehicle", route.vehicle, allocator);
  json_route.AddMember("cost", route.cost, allocator);

  if (!route.description.empty()) {
    json_route.AddMember("description", rapidjson::Value(), allocator);
    json_route["description"].SetString(route.description.c_str(),
                                        route.description.size(),
                                        allocator);
  }

  if (!route.delivery.empty()) {
    rapidjson::Value json_delivery(rapidjson::kArrayType);
    for (std::size_t i = 0; i < route.delivery.size(); ++i) {
      json_delivery.PushBack(route.delivery[i], allocator);
    }
    json_route.AddMember("delivery", json_delivery, allocator);

    // Support for deprecated "amount" key.
    rapidjson::Value json_amount(rapidjson::kArrayType);
    for (std::size_t i = 0; i < route.delivery.size(); ++i) {
      json_amount.PushBack(route.delivery[i], allocator);
    }
    json_route.AddMember("amount", json_amount, allocator);
  }

  if (!route.pickup.empty()) {
    rapidjson::Value json_pickup(rapidjson::kArrayType);
    for (std::size_t i = 0; i < route.pickup.size(); ++i) {
      json_pickup.PushBack(route.pickup[i], allocator);
    }
    json_route.AddMember("pickup", json_pickup, allocator);
  }

  json_route.AddMember("setup", route.setup, allocator);
  json_route.AddMember("service", route.service, allocator);
  json_route.AddMember("duration", route.duration, allocator);
  json_route.AddMember("waiting_time", route.waiting_time, allocator);
  json_route.AddMember("priority", route.priority, allocator);

  if (report_distances) {
    json_route.AddMember("distance", route.distance, allocator);
  }

  rapidjson::Value json_steps(rapidjson::kArrayType);
  for (const auto& step : route.steps) {
    json_steps.PushBack(to_json(step, report_distances, allocator), allocator);
  }

  json_route.AddMember("steps", json_steps, allocator);

  json_route.AddMember("violations",
                       get_violations(route.violations, allocator),
                       allocator);

  if (!route.geometry.empty()) {
    json_route.AddMember("geometry", rapidjson::Value(), allocator);
    json_route["geometry"].SetString(route.geometry.c_str(),
                                     route.geometry.size());
  }

  return json_route;
}

rapidjson::Value to_json(const ComputingTimes& ct,
                         rapidjson::Document::AllocatorType& allocator) {
  rapidjson::Value json_ct(rapidjson::kObjectType);

  json_ct.AddMember("loading", ct.loading, allocator);
  json_ct.AddMember("solving", ct.solving, allocator);
  json_ct.AddMember("routing", ct.routing, allocator);

  return json_ct;
}

rapidjson::Value to_json(const Step& s,
                         bool report_distances,
                         rapidjson::Document::AllocatorType& allocator) {
  rapidjson::Value json_step(rapidjson::kObjectType);

  json_step.AddMember("type", rapidjson::Value(), allocator);
  std::string str_type;
  switch (s.step_type) {
    using enum STEP_TYPE;
  case START:
    str_type = "start";
    break;
  case END:
    str_type = "end";
    break;
  case BREAK:
    str_type = "break";
    break;
  case JOB: {
    assert(s.job_type.has_value());
    switch (s.job_type.value()) {
      using enum JOB_TYPE;
    case SINGLE:
      str_type = "job";
      break;
    case PICKUP:
      str_type = "pickup";
      break;
    case DELIVERY:
      str_type = "delivery";
      break;
    }
    break;
  }
  }
  json_step["type"].SetString(str_type.c_str(), str_type.size(), allocator);

  if (!s.description.empty()) {
    json_step.AddMember("description", rapidjson::Value(), allocator);
    json_step["description"].SetString(s.description.c_str(),
                                       s.description.size(),
                                       allocator);
  }

  if (s.location.has_value()) {
    const auto& loc = s.location.value();
    if (loc.has_coordinates()) {
      json_step.AddMember("location", to_json(loc, allocator), allocator);
    }

    if (loc.user_index()) {
      json_step.AddMember("location_index", loc.index(), allocator);
    }
  }

  if (s.step_type == STEP_TYPE::JOB || s.step_type == STEP_TYPE::BREAK) {
    json_step.AddMember("id", s.id, allocator);
  }

  json_step.AddMember("setup", s.setup, allocator);
  json_step.AddMember("service", s.service, allocator);
  json_step.AddMember("waiting_time", s.waiting_time, allocator);

  // Should be removed at some point as step.job is deprecated.
  if (s.step_type == STEP_TYPE::JOB) {
    json_step.AddMember("job", s.id, allocator);
  }

  if (!s.load.empty()) {
    rapidjson::Value json_load(rapidjson::kArrayType);
    for (std::size_t i = 0; i < s.load.size(); ++i) {
      json_load.PushBack(s.load[i], allocator);
    }
    json_step.AddMember("load", json_load, allocator);
  }

  json_step.AddMember("arrival", s.arrival, allocator);
  json_step.AddMember("duration", s.duration, allocator);

  json_step.AddMember("violations",
                      get_violations(s.violations, allocator),
                      allocator);

  if (report_distances) {
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

void write_to_output(const rapidjson::Document& json_output,
                     const std::string& output_file) {
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

void write_to_json(const vroom::Exception& e, const std::string& output_file) {
  const auto json_output = to_json(e);

  write_to_output(json_output, output_file);
}

void write_to_json(const Solution& sol,
                   const std::string& output_file,
                   bool report_distances) {
  const auto json_output = to_json(sol, report_distances);

  write_to_output(json_output, output_file);
}

} // namespace vroom::io
