/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "routing/ors_wrapper.h"
#include "utils/output_json.h"

namespace vroom::routing {

OrsWrapper::OrsWrapper(const std::string& profile, const Server& server)
  : HttpWrapper(profile,
                server,
                "matrix",
                "durations",
                "distances",
                "directions",
                "") {
}

rapidjson::Document
OrsWrapper::build_request_body(const std::vector<Location>& locations,
                               const std::string& service) const {
  rapidjson::Document body;
  body.SetObject();
  auto& allocator = body.GetAllocator();
  rapidjson::Value coordinates(rapidjson::kArrayType);
  for (const auto& location : locations) {
    rapidjson::Value coord(rapidjson::kArrayType);
    coord.PushBack(location.lon(), allocator);
    coord.PushBack(location.lat(), allocator);
    coordinates.PushBack(coord, allocator);
  }

  if (service == _route_service) {
    body.AddMember("coordinates", coordinates, allocator);
    body.AddMember("geometry_simplify", "false", allocator);
    body.AddMember("continue_straight", "false", allocator);
  } else {
    assert(service == _matrix_service);
    body.AddMember("locations", coordinates, allocator);
    rapidjson::Value metrics(rapidjson::kArrayType);
    metrics.PushBack("duration", allocator);
    metrics.PushBack("distance", allocator);
    body.AddMember("metrics", metrics, allocator);
  }
  return body;
}

std::string OrsWrapper::build_query(const std::vector<Location>& locations,
                                    const std::string& service) const {
  return build_post_query(service + "/" + profile,
                          io::json_to_string(build_request_body(locations,
                                                                service),
                                             COORDINATES_PRECISION));
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
