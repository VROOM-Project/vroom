/*

This file is part of VROOM.

Copyright (c) 2015-2020, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "routing/osrm_routed_wrapper.h"
#include "utils/exception.h"

namespace vroom {
namespace routing {

OsrmRoutedWrapper::OsrmRoutedWrapper(const std::string& profile,
                                     const Server& server)
  : HttpWrapper(profile,
                server,
                "table",
                "route",
                "alternatives=false&steps=false&overview=full&continue_"
                "straight=false") {
}

std::string
OsrmRoutedWrapper::build_query(const std::vector<Location>& locations,
                               const std::string& service,
                               const std::string& extra_args) const {
  // Building query for osrm-routed
  std::string query = "GET /" + service;

  query += "/v1/" + profile + "/";

  // Adding locations.
  for (auto const& location : locations) {
    query += std::to_string(location.lon()) + "," +
             std::to_string(location.lat()) + ";";
  }
  query.pop_back(); // Remove trailing ';'.

  if (!extra_args.empty()) {
    query += "?" + extra_args;
  }

  query += " HTTP/1.1\r\n";
  query += "Host: " + _server.host + "\r\n";
  query += "Accept: */*\r\n";
  query += "Connection: close\r\n\r\n";

  return query;
}

void OsrmRoutedWrapper::parse_response(rapidjson::Document& infos,
                                       const std::string& json_content) const {
#ifdef NDEBUG
  infos.Parse(json_content.c_str());
#else
  assert(!infos.Parse(json_content.c_str()).HasParseError());
  assert(infos.HasMember("code"));
#endif
  if (infos["code"] != "Ok") {
    throw Exception(ERROR::ROUTING, std::string(infos["message"].GetString()));
  }
}

void OsrmRoutedWrapper::add_route_info(Route& route) const {
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
  route.distance = round_cost(infos["routes"][0]["distance"].GetDouble());
  route.geometry = std::move(infos["routes"][0]["geometry"].GetString());

  auto nb_legs = infos["routes"][0]["legs"].Size();
  assert(nb_legs == route.steps.size() - 1);

  // Accumulated travel distance stored for each step.
  double current_distance = 0;

  route.steps[0].distance = 0;

  for (rapidjson::SizeType i = 0; i < nb_legs; ++i) {
    // Update distance for step after current route leg.
    current_distance += infos["routes"][0]["legs"][i]["distance"].GetDouble();
    route.steps[i + 1].distance = round_cost(current_distance);
  }
}

} // namespace routing
} // namespace vroom
