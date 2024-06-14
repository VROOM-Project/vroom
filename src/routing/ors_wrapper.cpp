/*

This file is part of VROOM.

Copyright (c) 2015-2024, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "routing/ors_wrapper.h"
#include "utils/helpers.h"

#include <iostream>

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
    body += std::format("[{},{}],", location.lon(), location.lat());
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
  query += std::format("Content-Length: {}\r\n", body.size());
  query += "Host: " + _server.host + ":" + _server.port + "\r\n";
  query += "Connection: close\r\n";
  query += "\r\n" + body;

  return query;
}

void OrsWrapper::check_response(const boost::json::object& json_result,
                                const std::vector<Location>&,
                                const std::string&) const {
  if (json_result.contains("error")) {
    throw RoutingException(
      json_result.at("error").at("message").as_string().subview());
  }
}

bool OrsWrapper::duration_value_is_null(
  const boost::json::value& matrix_entry) const {
  return matrix_entry.is_null();
}

bool OrsWrapper::distance_value_is_null(
  const boost::json::value& matrix_entry) const {
  return matrix_entry.is_null();
}

UserDuration
OrsWrapper::get_duration_value(const boost::json::value& matrix_entry) const {
  return utils::round<UserDuration>(matrix_entry.to_number<double>());
}

UserDistance
OrsWrapper::get_distance_value(const boost::json::value& matrix_entry) const {
  return utils::round<UserDuration>(matrix_entry.to_number<double>());
}

unsigned OrsWrapper::get_legs_number(const boost::json::object& result) const {
  return result.at("routes").at(0).at("segments").as_array().size();
}

std::string OrsWrapper::get_geometry(boost::json::object& result) const {
  return result.at("routes").at(0).at("geometry").as_string().subview();
}

} // namespace vroom::routing
