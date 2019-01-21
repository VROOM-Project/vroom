#ifndef ORSHTTP_WRAPPER_H
#define ORSHTTP_WRAPPER_H

/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "routing/osrm_wrapper.h"

namespace vroom {
namespace routing {

class OrsHttpWrapper : public OSRMWrapper {

private:
  Server _server;

  void update_server();

  std::string build_query(const std::vector<Location>& locations,
                          std::string service,
                          std::string extra_args) const;

  std::string send_then_receive(std::string query) const;

public:
  OrsHttpWrapper(const std::string& profile, const Server& server);

  virtual Matrix<Cost>
  get_matrix(const std::vector<Location>& locs) const override;

  virtual void add_route_info(Route& route) const override;
};

} // namespace routing
} // namespace vroom

#endif
