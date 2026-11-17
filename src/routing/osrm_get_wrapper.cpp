/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "routing/osrm_get_wrapper.h"

namespace vroom::routing {

OsrmGetWrapper::OsrmGetWrapper(const std::string& profile, const Server& server)
  : OsrmRoutedWrapper(profile, server) {
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

} // namespace vroom::routing
