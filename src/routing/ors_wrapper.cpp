/*

This file is part of VROOM.

Copyright (c) 2015-2024, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "routing/ors_wrapper.h"

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
    body += std::format("[{:.6f},{:.6f}],", location.lon(), location.lat());
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

void OrsWrapper::check_response(const rapidjson::Document& json_result,
                                const std::vector<Location>&,
                                const std::string&) const {
  if (json_result.HasMember("error")) {
    if (json_result["error"].IsObject() &&
        json_result["error"].HasMember("message") &&
        json_result["error"]["message"].IsString()) {
      // Normal ORS error syntax.
      throw RoutingException(
        std::string(json_result["error"]["message"].GetString()));
    }

    if (json_result["error"].IsString()) {
      // Web framework error uses another convention, see #1083.
      auto error = std::string(json_result["error"].GetString());

      if (json_result.HasMember("path") && json_result["path"].IsString()) {
        error += " " + std::string(json_result["path"].GetString());
      }
      throw RoutingException(error);
    }
  }
}

const rapidjson::Value&
OrsWrapper::get_legs(const rapidjson::Value& result) const {
  assert(result.HasMember("routes") && result["routes"].IsArray() &&
         !result["routes"].Empty() &&
         result["routes"][0].HasMember("segments") &&
         result["routes"][0]["segments"].IsArray());

  return result["routes"][0]["segments"];
}

} // namespace vroom::routing
