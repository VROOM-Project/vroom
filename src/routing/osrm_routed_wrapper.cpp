/*

This file is part of VROOM.

Copyright (c) 2015-2020, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "../include/rapidjson/document.h"
#include "../include/rapidjson/error/en.h"

#include "routing/osrm_routed_wrapper.h"
#include "utils/exception.h"

namespace vroom {
namespace routing {

OsrmRoutedWrapper::OsrmRoutedWrapper(const std::string& profile,
                                     const Server& server)
  : RoutingWrapper(profile), HttpWrapper(server) {
}

std::string
OsrmRoutedWrapper::build_query(const std::vector<Location>& locations,
                               std::string service,
                               std::string extra_args = "") const {
  // Building query for osrm-routed
  std::string query = "GET /" + service;

  query += "/v1/" + _profile + "/";

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

Matrix<Cost>
OsrmRoutedWrapper::get_matrix(const std::vector<Location>& locs) const {
  std::string query = this->build_query(locs, "table");

  std::string json_content = this->run_query(query);

  // Expected matrix size.
  std::size_t m_size = locs.size();

  // Checking everything is fine in the response.
  rapidjson::Document infos;
#ifdef NDEBUG
  infos.Parse(json_content.c_str());
#else
  assert(!infos.Parse(json_content.c_str()).HasParseError());
  assert(infos.HasMember("code"));
#endif
  if (infos["code"] != "Ok") {
    throw Exception(ERROR::ROUTING,
                    "OSRM table: " + std::string(infos["message"].GetString()));
  }
  assert(infos["durations"].Size() == m_size);

  // Build matrix while checking for unfound routes to avoid
  // unexpected behavior (OSRM raises 'null').
  Matrix<Cost> m(m_size);

  std::vector<unsigned> nb_unfound_from_loc(m_size, 0);
  std::vector<unsigned> nb_unfound_to_loc(m_size, 0);

  for (rapidjson::SizeType i = 0; i < infos["durations"].Size(); ++i) {
    const auto& line = infos["durations"][i];
    assert(line.Size() == m_size);
    for (rapidjson::SizeType j = 0; j < line.Size(); ++j) {
      if (line[j].IsNull()) {
        // No route found between i and j. Just storing info as we
        // don't know yet which location is responsible between i
        // and j.
        ++nb_unfound_from_loc[i];
        ++nb_unfound_to_loc[j];
      } else {
        auto cost = round_cost(line[j].GetDouble());
        m[i][j] = cost;
      }
    }
  }

  check_unfound(locs, nb_unfound_from_loc, nb_unfound_to_loc);

  return m;
}

void OsrmRoutedWrapper::add_route_info(Route& route) const {
  // Ordering locations for the given steps.
  std::vector<Location> ordered_locations;
  for (const auto& step : route.steps) {
    ordered_locations.push_back(step.location);
  }

  std::string extra_args =
    "alternatives=false&steps=false&overview=full&continue_straight=false";

  std::string query = this->build_query(ordered_locations, "route", extra_args);

  std::string json_content = this->run_query(query);

  // Checking everything is fine in the response.
  rapidjson::Document infos;
#ifdef NDEBUG
  infos.Parse(json_content.c_str());
#else
  assert(!infos.Parse(json_content.c_str()).HasParseError());
  assert(infos.HasMember("code"));
#endif
  if (infos["code"] != "Ok") {
    throw Exception(ERROR::ROUTING,
                    "OSRM route: " + std::string(infos["message"].GetString()));
  }

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
