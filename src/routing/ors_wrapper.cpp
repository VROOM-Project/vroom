/*

This file is part of VROOM.

Copyright (c) 2015-2020, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "routing/ors_wrapper.h"
#include "utils/exception.h"

namespace vroom {
namespace routing {

OrsWrapper::OrsWrapper(const std::string& profile, const Server& server)
  : HttpWrapper(profile,
                server,
                "matrix",
                "directions",
                "\"geometry_simplify\":\"false\",\"continue_straight\":"
                "\"false\"") {
}

std::string OrsWrapper::build_query(const std::vector<Location>& locations,
                                    const std::string& service,
                                    const std::string& extra_args) const {
  // Adding locations.
  std::string body = "{\"";
  if (service == "directions") {
    body += "coordinates";
  } else {
    body += "locations";
  }
  body += "\":[";
  for (auto const& location : locations) {
    body += "[" + std::to_string(location.lon()) + "," +
            std::to_string(location.lat()) + "],";
  }
  body.pop_back(); // Remove trailing ','.
  body += "]";
  if (!extra_args.empty()) {
    body += "," + extra_args;
  }
  body += "}";

  // Building query for ORS
  std::string query = "POST /ors/v2/" + service + "/" + profile;

  query += " HTTP/1.0\r\n";
  query += "Accept: */*\r\n";
  query += "Content-Type: application/json\r\n";
  query += "Content-Length: " + std::to_string(body.size()) + "\r\n";
  query += "Host: " + _server.host + ":" + _server.port + "\r\n";
  query += "Connection: close\r\n";
  query += "\r\n" + body;

  return query;
}

void OrsWrapper::parse_response(rapidjson::Document& infos,
                                const std::string& json_content) const {
#ifdef NDEBUG
  infos.Parse(json_content.c_str());
#else
  assert(!infos.Parse(json_content.c_str()).HasParseError());
#endif
  if (infos.HasMember("error")) {
    throw Exception(ERROR::ROUTING,
                    std::string(infos["error"]["message"].GetString()));
  }
}

void OrsWrapper::add_route_info(Route& route) const {
  // Ordering locations for the given steps.
  std::vector<Location> ordered_locations;
  for (const auto& step : route.steps) {
    ordered_locations.push_back(step.location);
  }

  std::string query =
    this->build_query(ordered_locations, _route_service, _extra_args);
  std::string json_content = this->run_query(query);

  rapidjson::Document infos;
  this->parse_response(infos, json_content);

  // Total distance and route geometry.
  route.distance =
    round_cost(infos["routes"][0]["summary"]["distance"].GetDouble());
  route.geometry = std::move(infos["routes"][0]["geometry"].GetString());

  auto nb_legs = infos["routes"][0]["segments"].Size();
  assert(nb_legs == route.steps.size() - 1);

  // Accumulated travel distance stored for each step.
  double current_distance = 0;

  route.steps[0].distance = 0;

  for (rapidjson::SizeType i = 0; i < nb_legs; ++i) {
    // Update distance for step after current route leg.
    current_distance +=
      infos["routes"][0]["segments"][i]["distance"].GetDouble();
    route.steps[i + 1].distance = round_cost(current_distance);
  }
}

} // namespace routing
} // namespace vroom
