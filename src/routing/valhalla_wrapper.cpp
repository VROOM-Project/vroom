/*

This file is part of VROOM.

Copyright (c) 2015-2021, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "routing/valhalla_wrapper.h"

namespace vroom {
namespace routing {

ValhallaWrapper::ValhallaWrapper(const std::string& profile,
                                 const Server& server)
  : HttpWrapper(profile,
                server,
                "sources_to_targets",
                "sources_to_targets",
                "route",
                "\"directions_type\":\"none\"") {
}

std::string ValhallaWrapper::get_matrix_query(
  const std::vector<Location>& locations) const {
  // Building matrix query for Valhalla.
  std::string query = "GET /" + _matrix_service + "?json=";

  // List locations.
  std::string all_locations;
  for (auto const& location : locations) {
    all_locations += "{\"lon\":" + std::to_string(location.lon()) + "," +
                     "\"lat\":" + std::to_string(location.lat()) + "},";
  }
  all_locations.pop_back(); // Remove trailing ','.

  query += "{\"sources\":[" + all_locations;
  query += "],\"targets\":[" + all_locations;
  query += "],\"costing\":\"" + profile + "\"}";

  query += " HTTP/1.1\r\n";
  query += "Host: " + _server.host + "\r\n";
  query += "Accept: */*\r\n";
  query += "Connection: close\r\n\r\n";

  return query;
}

std::string
ValhallaWrapper::get_route_query(const std::vector<Location>& locations,
                                 const std::string& extra_args) const {
  // Building matrix query for Valhalla.
  std::string query = "GET /" + _route_service + "?json={\"locations\":[";

  for (auto const& location : locations) {
    query += "{\"lon\":" + std::to_string(location.lon()) + "," +
             "\"lat\":" + std::to_string(location.lat()) +
             ",\"type\":\"break\"},";
  }
  query.pop_back(); // Remove trailing ','.

  query += "],\"costing\":\"" + profile + "\"";
  if (!extra_args.empty()) {
    query += "," + extra_args;
  }
  query += "}";

  query += " HTTP/1.1\r\n";
  query += "Host: " + _server.host + "\r\n";
  query += "Accept: */*\r\n";
  query += "Connection: close\r\n\r\n";

  return query;
}

std::string ValhallaWrapper::build_query(const std::vector<Location>& locations,
                                         const std::string& service,
                                         const std::string& extra_args) const {
  assert(service == _matrix_service or service == _route_service);

  return (service == _matrix_service) ? get_matrix_query(locations)
                                      : get_route_query(locations, extra_args);
}

void ValhallaWrapper::check_response(const rapidjson::Document& json_result,
                                     const std::string& service) const {
  if (service == _route_service) {
    assert(json_result.HasMember("trip") and
           json_result["trip"].HasMember("status"));
    if (json_result["trip"]["status"] != 0) {
      throw Exception(ERROR::ROUTING,
                      std::string(
                        json_result["trip"]["status_message"].GetString()));
    }
  }
}

bool ValhallaWrapper::duration_value_is_null(
  const rapidjson::Value& matrix_entry) const {
  assert(matrix_entry.HasMember("time"));
  return matrix_entry["time"].IsNull();
}

Cost ValhallaWrapper::get_duration_value(
  const rapidjson::Value& matrix_entry) const {
  assert(matrix_entry["time"].IsUint());
  return matrix_entry["time"].GetUint();
}

double
ValhallaWrapper::get_total_distance(const rapidjson::Value& result) const {
  return 100 * result["trip"]["summary"]["length"].GetDouble();
}

unsigned
ValhallaWrapper::get_legs_number(const rapidjson::Value& result) const {
  return result["trip"]["legs"].Size();
}

double ValhallaWrapper::get_distance_for_leg(const rapidjson::Value& result,
                                             rapidjson::SizeType i) const {
  return 100 * result["trip"]["legs"][i]["summary"]["length"].GetDouble();
}

} // namespace routing
} // namespace vroom
