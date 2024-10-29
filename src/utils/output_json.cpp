/*

This file is part of VROOM.

Copyright (c) 2015-2024, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <fstream>
#include <iostream>

#include "../include/rapidjson/include/rapidjson/stringbuffer.h"
#include "../include/rapidjson/include/rapidjson/writer.h"

#include "structures/typedefs.h"
#include "utils/output_json.h"

namespace vroom::io {

inline rapidjson::Value
get_violations(const Violations& violations,
               rapidjson::Document::AllocatorType& allocator) {
  rapidjson::Value json_violations(rapidjson::kArrayType);

  for (const auto type : violations.types) {
    rapidjson::Value json_violation(rapidjson::kObjectType);
    std::string cause;
    switch (type) {
      using enum VIOLATION;
    case LEAD_TIME:
      cause = "lead_time";
      json_violation.AddMember("duration", violations.lead_time, allocator);
      break;
    case DELAY:
      cause = "delay";
      json_violation.AddMember("duration", violations.delay, allocator);
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

    json_violation.AddMember("cause", rapidjson::Value(), allocator);
    json_violation["cause"].SetString(cause.c_str(), cause.size(), allocator);

    json_violations.PushBack(json_violation, allocator);
  }

  return json_violations;
}

rapidjson::Document to_json(const Solution& sol, bool report_distances) {
  rapidjson::Document json_output;
  json_output.SetObject();
  rapidjson::Document::AllocatorType& allocator = json_output.GetAllocator();

  json_output.AddMember("code", 0, allocator);
  json_output.AddMember("summary",
                        to_json(sol.summary, report_distances, allocator),
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
    if (job.location.user_index()) {
      json_job.AddMember("location_index", job.location.index(), allocator);
    }
    json_job.AddMember("type", rapidjson::Value(), allocator);
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
    json_job["type"].SetString(str_type.c_str(), str_type.size(), allocator);

    if (!job.description.empty()) {
      json_job.AddMember("description", rapidjson::Value(), allocator);
      json_job["description"].SetString(job.description.c_str(),
                                        job.description.size(),
                                        allocator);
    }

    json_unassigned.PushBack(json_job, allocator);
  }

  json_output.AddMember("unassigned", json_unassigned, allocator);

  rapidjson::Value json_routes(rapidjson::kArrayType);
  for (const auto& route : sol.routes) {
    json_routes.PushBack(to_json(route, report_distances, allocator),
                         allocator);
  }

  json_output.AddMember("routes", json_routes, allocator);

  return json_output;
}

rapidjson::Document to_json(const vroom::Exception& e) {
  rapidjson::Document json_output;
  json_output.SetObject();
  rapidjson::Document::AllocatorType& allocator = json_output.GetAllocator();

  json_output.AddMember("code", e.error_code, allocator);
  json_output.AddMember("error", rapidjson::Value(), allocator);
  json_output["error"].SetString(e.message.c_str(), e.message.size());

  return json_output;
}

rapidjson::Value to_json(const Summary& summary,
                         bool report_distances,
                         rapidjson::Document::AllocatorType& allocator) {
  rapidjson::Value json_summary(rapidjson::kObjectType);

  json_summary.AddMember("cost", summary.cost, allocator);
  json_summary.AddMember("routes", summary.routes, allocator);
  json_summary.AddMember("unassigned", summary.unassigned, allocator);

  if (!summary.delivery.empty()) {
    rapidjson::Value json_delivery(rapidjson::kArrayType);
    for (std::size_t i = 0; i < summary.delivery.size(); ++i) {
      json_delivery.PushBack(summary.delivery[i], allocator);
    }
    json_summary.AddMember("delivery", json_delivery, allocator);

    // Support for deprecated "amount" key.
    rapidjson::Value json_amount(rapidjson::kArrayType);
    for (std::size_t i = 0; i < summary.delivery.size(); ++i) {
      json_amount.PushBack(summary.delivery[i], allocator);
    }
    json_summary.AddMember("amount", json_amount, allocator);
  }

  if (!summary.pickup.empty()) {
    rapidjson::Value json_pickup(rapidjson::kArrayType);
    for (std::size_t i = 0; i < summary.pickup.size(); ++i) {
      json_pickup.PushBack(summary.pickup[i], allocator);
    }
    json_summary.AddMember("pickup", json_pickup, allocator);
  }

  json_summary.AddMember("setup", summary.setup, allocator);
  json_summary.AddMember("service", summary.service, allocator);
  json_summary.AddMember("duration", summary.duration, allocator);
  json_summary.AddMember("waiting_time", summary.waiting_time, allocator);
  json_summary.AddMember("priority", summary.priority, allocator);

  if (report_distances) {
    json_summary.AddMember("distance", summary.distance, allocator);
  }

  json_summary.AddMember("violations",
                         get_violations(summary.violations, allocator),
                         allocator);

  json_summary.AddMember("computing_times",
                         to_json(summary.computing_times, allocator),
                         allocator);

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

#ifdef LOG_LS
rapidjson::Value to_json(const std::vector<ls::log::Step>& steps,
                         rapidjson::Document::AllocatorType& allocator) {
  rapidjson::Value json_LS_steps(rapidjson::kArrayType);

  assert(steps.front().event == ls::log::EVENT::START);
  const auto start_time = steps.front().time_point;

  for (const auto& step : steps) {
    rapidjson::Value json_step(rapidjson::kObjectType);

    const auto delta = std::chrono::duration_cast<std::chrono::microseconds>(
      step.time_point - start_time);
    json_step.AddMember("time", delta.count(), allocator);

    std::string event;
    switch (step.event) {
      using enum ls::log::EVENT;
    case START:
      event = "Start";
      break;
    case OPERATOR:
      event = OPERATOR_NAMES[step.operator_name];
      break;
    case LOCAL_MINIMA:
      event = "LocalMinima";
      break;
    case JOB_ADDITION:
      event = "JobAddition";
      break;
    case RUIN:
      event = "Ruin";
      break;
    case RECREATE:
      event = "Recreate";
      break;
    case ROLLBACK:
      event = "Rollback";
      break;
    default:
      assert(false);
    }
    json_step.AddMember("event", rapidjson::Value(), allocator);
    json_step["event"].SetString(event.c_str(), event.size(), allocator);

    rapidjson::Value json_score(rapidjson::kObjectType);
    json_score.AddMember("priority", step.indicators.priority_sum, allocator);
    json_score.AddMember("assigned", step.indicators.assigned, allocator);
    json_score.AddMember("cost",
                         utils::scale_to_user_cost(step.indicators.eval.cost),
                         allocator);

    json_step.AddMember("score", json_score, allocator);

    if (step.solution.has_value()) {
      rapidjson::Value step_solution(rapidjson::kObjectType);
      auto json_solution = to_json(step.solution.value(), false);
      step_solution.CopyFrom(json_solution, allocator);

      json_step.AddMember("solution", step_solution, allocator);
    }

    json_LS_steps.PushBack(json_step, allocator);
  }

  return json_LS_steps;
}

rapidjson::Value to_json(const ls::log::Dump& dump,
                         rapidjson::Document::AllocatorType& allocator) {
  rapidjson::Value json_parameters(rapidjson::kObjectType);

  std::string heuristic;
  switch (dump.heuristic_parameters.heuristic) {
    using enum HEURISTIC;
  case BASIC:
    heuristic = "BASIC";
    break;
  case DYNAMIC:
    heuristic = "DYNAMIC";
    break;
  default:
    assert(false);
  }
  json_parameters.AddMember("heuristic", rapidjson::Value(), allocator);
  json_parameters["heuristic"].SetString(heuristic.c_str(),
                                         heuristic.size(),
                                         allocator);

  std::string init;
  switch (dump.heuristic_parameters.init) {
    using enum INIT;
  case NONE:
    init = "NONE";
    break;
  case HIGHER_AMOUNT:
    init = "HIGHER_AMOUNT";
    break;
  case NEAREST:
    init = "NEAREST";
    break;
  case FURTHEST:
    init = "FURTHEST";
    break;
  case EARLIEST_DEADLINE:
    init = "EARLIEST_DEADLINE";
    break;
  default:
    assert(false);
  }
  json_parameters.AddMember("init", rapidjson::Value(), allocator);
  json_parameters["init"].SetString(init.c_str(), init.size(), allocator);

  json_parameters.AddMember("regret",
                            dump.heuristic_parameters.regret_coeff,
                            allocator);

  std::string sort;
  switch (dump.heuristic_parameters.sort) {
    using enum SORT;
  case AVAILABILITY:
    sort = "AVAILABILITY";
    break;
  case COST:
    sort = "COST";
    break;
  default:
    assert(false);
  }
  json_parameters.AddMember("sort", rapidjson::Value(), allocator);
  json_parameters["sort"].SetString(sort.c_str(), sort.size(), allocator);

  json_parameters.AddMember("steps", to_json(dump.steps, allocator), allocator);

  return json_parameters;
}

void write_LS_logs_to_json(const std::vector<ls::log::Dump>& dumps) {
  rapidjson::Document json_log;
  json_log.SetArray();
  rapidjson::Document::AllocatorType& allocator = json_log.GetAllocator();

  for (const auto& dump : dumps) {
    json_log.PushBack(to_json(dump, allocator), allocator);
  }

  write_to_output(json_log, "vroom_ls_log.json");
}

#endif

} // namespace vroom::io
