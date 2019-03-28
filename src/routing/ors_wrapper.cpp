/*

This file is part of VROOM.

Copyright (c) 2015-2022, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "routing/ors_wrapper.h"
#include "utils/helpers.h"

namespace vroom::routing {

OrsWrapper::OrsWrapper(const std::string& profile, const Server& server)
  : HttpWrapper(profile,
                server,
                "matrix",
                "durations",
                "distances",
                "directions",
                "\"geometry_simplify\":\"false\",\"continue_straight\":"
                "\"false\"") {
}

std::string OrsWrapper::build_query(const std::vector<Location>& locations,
                                    const std::string& service) const {
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
  if (service == _route_service) {
    body += "," + _routing_args;
  } else {
    assert(service == _matrix_service);
    body += ",\"metrics\":[\"duration\",\"distance\"]";
  }
  body += "}";

  // Building query for ORS
  std::string query = "POST /" + _server.path + service + "/" + profile;

  query += " HTTP/1.0\r\n";
  query += "Accept: */*\r\n";
  query += "Content-Type: application/json\r\n";
  query += "Content-Length: " + std::to_string(body.size()) + "\r\n";
  query += "Host: " + _server.host + ":" + _server.port + "\r\n";
  query += "Connection: close\r\n";
  query += "\r\n" + body;

  return query;
}

void OrsWrapper::check_response(const rapidjson::Document& json_result,
                                const std::vector<Location>&,
                                const std::string&) const {
  if (json_result.HasMember("error")) {
    throw RoutingException(
      std::string(json_result["error"]["message"].GetString()));
  }
}

bool OrsWrapper::duration_value_is_null(
  const rapidjson::Value& matrix_entry) const {
  return matrix_entry.IsNull();
}

bool OrsWrapper::distance_value_is_null(
  const rapidjson::Value& matrix_entry) const {
  return matrix_entry.IsNull();
}

UserDuration
OrsWrapper::get_duration_value(const rapidjson::Value& matrix_entry) const {
  return utils::round<UserDuration>(matrix_entry.GetDouble());
}

UserDistance
OrsWrapper::get_distance_value(const rapidjson::Value& matrix_entry) const {
  return utils::round<UserDistance>(matrix_entry.GetDouble());
}

unsigned OrsWrapper::get_legs_number(const rapidjson::Value& result) const {
  return result["routes"][0]["segments"].Size();
}

std::string OrsWrapper::get_geometry(rapidjson::Value& result) const {
  return result["routes"][0]["geometry"].GetString();
}

} // namespace vroom::routing
