/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "routing/osrm_post_routed_wrapper.h"
#include "utils/output_json.h"

namespace vroom::routing {

OsrmPostRoutedWrapper::OsrmPostRoutedWrapper(const std::string& profile,
                                             const Server& server)
  : OsrmRoutedWrapper(profile, server) {
}

std::string
OsrmPostRoutedWrapper::build_query(const std::vector<Location>& locations,
                                   const std::string& service) const {
  return build_post_query(service + "/v1/" + profile,
                          build_request_body(locations, service));
}

std::string OsrmPostRoutedWrapper::build_request_body(
  const std::vector<Location>& locations,
  const std::string& service) const {
  rapidjson::Document body;
  body.SetObject();
  auto& allocator = body.GetAllocator();
  rapidjson::Value coordinates(rapidjson::kArrayType);
  rapidjson::Value radiuses(rapidjson::kArrayType);
  for (const auto& location : locations) {
    rapidjson::Value coord(rapidjson::kArrayType);
    coord.PushBack(location.lon(), allocator);
    coord.PushBack(location.lat(), allocator);
    coordinates.PushBack(coord, allocator);
    radiuses.PushBack(DEFAULT_LIBOSRM_SNAPPING_RADIUS, allocator);
  }
  body.AddMember("coordinates", coordinates, allocator);
  body.AddMember("radiuses", radiuses, allocator);
  if (service == _route_service) {
    body.AddMember("alternatives", false, allocator);
    body.AddMember("steps", false, allocator);
    body.AddMember("overview", "full", allocator);
    body.AddMember("continue_straight", false, allocator);
  } else if (service == _matrix_service) {
    rapidjson::Value annotations(rapidjson::kArrayType);
    annotations.PushBack("duration", allocator);
    annotations.PushBack("distance", allocator);
    body.AddMember("annotations", annotations, allocator);
  }
  return io::json_to_string(body);
}

} // namespace vroom::routing
