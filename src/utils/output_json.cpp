/*

This file is part of VROOM.

Copyright (c) 2015-2024, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <fstream>
#include <iostream>

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
    json_routes.emplace_back(to_json(route, report_distances));
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

boost::json::object to_json(const Summary& summary, bool report_distances) {
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
    json_summary["amount"] = json_delivery;
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

boost::json::object to_json(const Route& route, bool report_distances) {
  boost::json::object json_route;

  json_route["vehicle"] = route.vehicle;
  json_route["cost"] = route.cost;

  if (!route.description.empty()) {
    json_route["description"] = route.description;
  }

  if (!route.delivery.empty()) {
    boost::json::array json_delivery;
    for (std::size_t i = 0; i < route.delivery.size(); ++i) {
      json_delivery.emplace_back(route.delivery[i]);
    }
    json_route["delivery"] = json_delivery;

    // Support for deprecated "amount" key.
    boost::json::array json_amount;
    for (std::size_t i = 0; i < route.delivery.size(); ++i) {
      json_amount.emplace_back(route.delivery[i]);
    }
    json_route["amount"] = json_amount;
  }

  if (!route.pickup.empty()) {
    boost::json::array json_pickup;
    for (std::size_t i = 0; i < route.pickup.size(); ++i) {
      json_pickup.emplace_back(route.pickup[i]);
    }
    json_route["pickup"] = json_pickup;
  }

  json_route["setup"] = route.setup;
  json_route["service"] = route.service;
  json_route["duration"] = route.duration;
  json_route["waiting_time"] = route.waiting_time;
  json_route["priority"] = route.priority;

  if (report_distances) {
    json_route["distance"] = route.distance;
  }

  boost::json::array json_steps;
  for (const auto& step : route.steps) {
    json_steps.emplace_back(to_json(step, report_distances));
  }

  json_route["steps"] = json_steps;

  json_route["violations"] = get_violations(route.violations);

  if (!route.geometry.empty()) {
    json_route["geometry"] = route.geometry;
  }

  return json_route;
}

boost::json::object to_json(const ComputingTimes& ct) {
  boost::json::object json_ct;

  json_ct["loading"] = ct.loading;
  json_ct["solving"] = ct.solving;
  json_ct["routing"] = ct.routing;

  return json_ct;
}

boost::json::object to_json(const Step& s, bool report_distances) {
  boost::json::object json_step;

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
  json_step["type"] = str_type;

  if (!s.description.empty()) {
    json_step["description"] = s.description;
  }

  if (s.location.has_value()) {
    const auto& loc = s.location.value();
    if (loc.has_coordinates()) {
      json_step["location"] = to_json(loc);
    }

    if (loc.user_index()) {
      json_step["location_index"] = loc.index();
    }
  }

  if (s.step_type == STEP_TYPE::JOB || s.step_type == STEP_TYPE::BREAK) {
    json_step["id"] = s.id;
  }

  json_step["setup"] = s.setup;
  json_step["service"] = s.service;
  json_step["waiting_time"] = s.waiting_time;

  // Should be removed at some point as step.job is deprecated.
  if (s.step_type == STEP_TYPE::JOB) {
    json_step["job"] = s.id;
  }

  if (!s.load.empty()) {
    boost::json::array json_load;
    for (std::size_t i = 0; i < s.load.size(); ++i) {
      json_load.emplace_back(s.load[i]);
    }
    json_step["load"] = json_load;
  }

  json_step["arrival"] = s.arrival;
  json_step["duration"] = s.duration;
  json_step["violations"] = get_violations(s.violations);

  if (report_distances) {
    json_step["distance"] = s.distance;
  }

  return json_step;
}

boost::json::array to_json(const Location& loc) {
  boost::json::array json_coords;

  json_coords.emplace_back(loc.lon());
  json_coords.emplace_back(loc.lat());

  return json_coords;
}

void write_to_output(const boost::json::value& json_output,
                     const std::string& output_file) {

  // Write to relevant output.
  if (output_file.empty()) {
    // Log to standard output.
    std::cout << boost::json::serialize(json_output) << std::endl;
  } else {
    // Log to file.
    std::ofstream out_stream(output_file, std::ofstream::out);
    out_stream << boost::json::serialize(json_output);
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
