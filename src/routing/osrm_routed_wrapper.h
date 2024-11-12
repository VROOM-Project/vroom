#ifndef OSRM_ROUTED_WRAPPER_H
#define OSRM_ROUTED_WRAPPER_H

/*

This file is part of VROOM.

Copyright (c) 2015-2024, Julien Coupey.
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

  const rapidjson::Value&
  get_legs(const rapidjson::Value& result) const override;

public:
  OsrmRoutedWrapper(const std::string& profile, const Server& server);
};

} // namespace vroom::routing

#endif
