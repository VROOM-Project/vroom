/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "routing/osrm_get_wrapper.h"

namespace vroom::routing {

OsrmGetWrapper::OsrmGetWrapper(const std::string& profile, const Server& server)
  : HttpWrapper(profile,
                server,
                "table",
                "durations",
                "distances",
                "route",
                "alternatives=false&steps=false&overview=full&continue_"
                "straight=false") {
}

std::string OsrmGetWrapper::build_query(const std::vector<Location>& locations,
                                        const std::string& service) const {

  return this->build_get_query(std::string{service + "/v1/" + profile + '/'},
                               this->build_query_body(locations, service));
}

std::string
OsrmGetWrapper::build_query_body(const std::vector<Location>& locations,
                                 const std::string& service) const {
  std::string query_body;
  std::string radiuses = "radiuses=";
  radiuses.reserve(radiuses.size() +
                   locations.size() *
                     (DEFAULT_OSRM_SNAPPING_RADIUS.size() + 1));

  // Adding locations and radiuses values.
  for (const Location& location : locations) {
    query_body += std::format("{:.6f},{:.6f};", location.lon(), location.lat());
    radiuses += DEFAULT_OSRM_SNAPPING_RADIUS + ';';
  }
  query_body.pop_back();
  radiuses.pop_back();

  if (service == _route_service) {
    query_body += "?" + _routing_args;
  } else {
    assert(service == _matrix_service);
    query_body += "?annotations=duration,distance";
  }
  query_body += "&" + radiuses;
  return query_body;
}

void OsrmGetWrapper::check_response(const rapidjson::Document& json_result,
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
OsrmGetWrapper::get_legs(const rapidjson::Value& result) const {
  assert(result.HasMember("routes") && result["routes"].IsArray() &&
         !result["routes"].Empty() && result["routes"][0].HasMember("legs") &&
         result["routes"][0]["legs"].IsArray());

  return result["routes"][0]["legs"];
}

} // namespace vroom::routing
