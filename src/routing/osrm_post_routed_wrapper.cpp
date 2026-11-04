/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "routing/osrm_post_routed_wrapper.h"
#include "utils/helpers.h"
#include "utils/output_json.h"

namespace vroom::routing {

OsrmPostRoutedWrapper::OsrmPostRoutedWrapper(const std::string& profile,
                                             const Server& server)
  : HttpWrapper(profile,
                server,
                "table",
                "durations",
                "distances",
                "route",
                "alternatives=false&steps=false&overview=full&continue_"
                "straight=false") {
}

std::string
OsrmPostRoutedWrapper::build_query(const std::vector<Location>& locations,
                                   const std::string& service) const {
  std::string body = build_request_body(locations, service);
  std::string query = "POST /" + _server.path + service + "/v1" + profile;
  query += " HTTP/1.0\r\n";
  query += "Accept: */*\r\n";
  query += "Content-Type: application/json\r\n";
  query += std::format("Content-Length: {}\r\n", body.size());
  query += "Host: " + _server.host + ":" + _server.port + "\r\n";
  query += "Connection: close\r\n";
  query += "\r\n" + body;

  return query;
}

std::string OsrmPostRoutedWrapper::build_request_body(
  const std::vector<Location>& locations,
  const std::string& service) const {
  rapidjson::Document docs;
  auto& allocator = docs.GetAllocator();
  std::map<std::string, rapidjson::Value> args_mapping =
    get_routing_args_as_map();
  rapidjson::Value body(rapidjson::kObjectType);
  rapidjson::Value coordinates(rapidjson::kArrayType);
  rapidjson::Value radiuses(rapidjson::kArrayType);
  for (const auto& locs : locations) {
    rapidjson::Value coord(rapidjson::kArrayType);
    coord.PushBack(locs.lon(), allocator);
    coord.PushBack(locs.lat(), allocator);
    coordinates.PushBack(coord, allocator);
    radiuses.PushBack(rapidjson::Value(DEFAULT_LIBOSRM_SNAPPING_RADIUS),
                      allocator);
  }
  body.AddMember("coordinates", coordinates, allocator);
  body.AddMember("radiuses", radiuses, allocator);
  if (service == "route") {
    for (auto& [k, v] : args_mapping) {
      body.AddMember(rapidjson::Value(k.c_str(), allocator), v, allocator);
    }
  }
  docs.Swap(body);
  return io::json_to_string(docs);
}

void OsrmPostRoutedWrapper::check_response(
  const rapidjson::Document& json_result,
  const std::vector<Location>& locs,
  const std::string&) const {
  assert(json_result.HasMember("code"));
  const std::string code = json_result["code"].GetString();
  if (code != "Ok") {
    const std::string message = json_result["message"].GetString();
    if (const std::string snapping_error_base =
          "Could not find a matching segment for coordinate ";
        code == "NoSegment" && message.starts_with(snapping_error_base)) {
      const auto error_loc =
        std::stoul(message.substr(snapping_error_base.size(),
                                  message.size() - snapping_error_base.size()));
      const auto coordinates = std::format("[{:.6f},{:.6f}]",
                                           locs[error_loc].lon(),
                                           locs[error_loc].lat());
      throw RoutingException("Could not find route near location " +
                             coordinates);
    }

    // Other error in response.
    throw RoutingException(message);
  }
}

const rapidjson::Value&
OsrmPostRoutedWrapper::get_legs(const rapidjson::Value& result) const {
  assert(result.HasMember("routes") && result["routes"].IsArray() &&
         !result["routes"].Empty() && result["routes"][0].HasMember("legs") &&
         result["routes"][0]["legs"].IsArray());

  return result["routes"][0]["legs"];
}

std::map<std::string, rapidjson::Value>
OsrmPostRoutedWrapper::get_routing_args_as_map() const {
  std::map<std::string, rapidjson::Value> mapping;
  std::vector<std::string> keys_values =
    vroom::utils::split(this->_routing_args, '&');
  for (const auto& k_v : keys_values) {
    std::vector<std::string> key_and_value = vroom::utils::split(k_v, '=');
    std::string& key = key_and_value[0];
    std::string& value = key_and_value[1];
    mapping[key] = value == "true" ? rapidjson::Value(true)
                   : value == "false"
                     ? rapidjson::Value(false)
                     : rapidjson::Value(value.c_str(), value.length());
  }
  return mapping;
}

} // namespace vroom::routing
