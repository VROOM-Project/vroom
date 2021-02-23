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
                "") {
}

std::string ValhallaWrapper::build_query(const std::vector<Location>& locations,
                                         const std::string& service,
                                         const std::string& extra_args) const {
  // Building query for Valhalla
  std::string query = "GET /" + service + "?json=";

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

void ValhallaWrapper::parse_response(rapidjson::Document& json_result,
                                     const std::string& json_content) const {
#ifdef NDEBUG
  json_result.Parse(json_content.c_str());
#else
  assert(!json_result.Parse(json_content.c_str()).HasParseError());
#endif
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
ValhallaWrapper::get_total_distance(const rapidjson::Value& route) const {
  // TODO implement
  return 0;
}

unsigned ValhallaWrapper::get_legs_number(const rapidjson::Value& route) const {
  // TODO implement
  return 0;
}

double ValhallaWrapper::get_distance_for_leg(const rapidjson::Value& route,
                                             rapidjson::SizeType i) const {
  // TODO implement
  return 0;
}

} // namespace routing
} // namespace vroom
