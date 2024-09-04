#ifndef OSRM_ROUTED_WRAPPER_H
#define OSRM_ROUTED_WRAPPER_H

/*

This file is part of VROOM.

Copyright (c) 2015-2022, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "routing/http_wrapper.h"

namespace vroom::routing {

class OsrmRoutedWrapper : public HttpWrapper {
private:
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

  unsigned get_legs_number(const rapidjson::Value& result) const override;

  std::string get_geometry(rapidjson::Value& result) const override;
  
  rapidjson::Value get_legs(rapidjson::Value& result, rapidjson::Document::AllocatorType& allocator) const override;
 

public:

  OsrmRoutedWrapper(const std::string& profile, const Server& server);
};

} // namespace vroom::routing

#endif
