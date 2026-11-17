#ifndef OSRM_ROUTED_WRAPPER_H
#define OSRM_ROUTED_WRAPPER_H

/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "routing/http_wrapper.h"

namespace vroom::routing {
class OsrmRoutedWrapper : public HttpWrapper {

public:
  OsrmRoutedWrapper(const std::string& profile, const Server& server);

protected:
  void check_response(const rapidjson::Document& json_result,
                      const std::vector<Location>& locs,
                      const std::string& service) const override;

  const rapidjson::Value&
  get_legs(const rapidjson::Value& result) const override;
};
} // namespace vroom::routing

#endif