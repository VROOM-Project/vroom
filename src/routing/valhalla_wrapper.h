#ifndef VALHALLA_WRAPPER_H
#define VALHALLA_WRAPPER_H

/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "routing/http_wrapper.h"

namespace vroom::routing {

class ValhallaWrapper : public HttpWrapper {
private:
  // Empty sources or destinations means using all locations.
  rapidjson::Document
  build_matrix_body(const std::vector<Location>& locations,
                    const std::vector<std::size_t>& sources,
                    const std::vector<std::size_t>& destinations) const;

  rapidjson::Document
  build_route_body(const std::vector<Location>& locations) const;

  std::string build_query(const std::vector<Location>& locations,
                          const std::string& service) const override;

  void check_response(const rapidjson::Document& json_result,
                      const std::vector<Location>& locs,
                      const std::string& service) const override;

  bool
  duration_value_is_null(const rapidjson::Value& matrix_entry) const override;

  bool
  distance_value_is_null(const rapidjson::Value& matrix_entry) const override;

  UserDuration
  get_duration_value(const rapidjson::Value& matrix_entry) const override;

  UserDistance
  get_distance_value(const rapidjson::Value& matrix_entry) const override;

  const rapidjson::Value&
  get_legs(const rapidjson::Value& result) const override;

  UserDuration get_leg_duration(const rapidjson::Value& leg) const override;

  UserDistance get_leg_distance(const rapidjson::Value& leg) const override;

  std::string get_geometry(rapidjson::Value& result) const override;

public:
  ValhallaWrapper(const std::string& profile, const Server& server);
};

} // namespace vroom::routing

#endif
