/*

This file is part of VROOM.

Copyright (c) 2015-2021, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "routing/osrm_routed_wrapper.h"

namespace vroom {
namespace routing {

OsrmRoutedWrapper::OsrmRoutedWrapper(const std::string& profile,
                                     const Server& server)
  : HttpWrapper(profile,
                server,
                "table",
                "durations",
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

void OsrmRoutedWrapper::check_response(const rapidjson::Document& json_result,
                                       const std::string&) const {
  assert(json_result.HasMember("code"));
  if (json_result["code"] != "Ok") {
    throw Exception(ERROR::ROUTING,
                    std::string(json_result["message"].GetString()));
  }
}

bool OsrmRoutedWrapper::duration_value_is_null(
  const rapidjson::Value& matrix_entry) const {
  return matrix_entry.IsNull();
}

Cost OsrmRoutedWrapper::get_duration_value(
  const rapidjson::Value& matrix_entry) const {
  return round_cost(matrix_entry.GetDouble());
}

double
OsrmRoutedWrapper::get_total_distance(const rapidjson::Value& result) const {
  return result["routes"][0]["distance"].GetDouble();
}

unsigned
OsrmRoutedWrapper::get_legs_number(const rapidjson::Value& result) const {
  return result["routes"][0]["legs"].Size();
}

double OsrmRoutedWrapper::get_distance_for_leg(const rapidjson::Value& result,
                                               rapidjson::SizeType i) const {
  return result["routes"][0]["legs"][i]["distance"].GetDouble();
}

std::string OsrmRoutedWrapper::get_geometry(rapidjson::Value& result) const {
  return result["routes"][0]["geometry"].GetString();
}

} // namespace routing
} // namespace vroom
