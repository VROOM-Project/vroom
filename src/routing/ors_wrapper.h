#ifndef ORS_WRAPPER_H
#define ORS_WRAPPER_H

/*

This file is part of VROOM.

Copyright (c) 2015-2020, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "routing/http_wrapper.h"

namespace vroom {
namespace routing {

class OrsWrapper : public HttpWrapper {
private:
  virtual std::string build_query(const std::vector<Location>& locations,
                                  const std::string& service,
                                  const std::string& extra_args) const override;

  virtual void parse_response(rapidjson::Document& input,
                              const std::string& json_content) const override;

public:
  OrsWrapper(const std::string& profile, const Server& server);

  virtual void add_route_info(Route& route) const override;
};

} // namespace routing
} // namespace vroom

#endif
