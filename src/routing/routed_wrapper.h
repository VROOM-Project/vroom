#ifndef ROUTED_WRAPPER_H
#define ROUTED_WRAPPER_H

/*

This file is part of VROOM.

Copyright (c) 2015-2017, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <boost/asio.hpp>

#include "./osrm_wrapper.h"

using boost::asio::ip::tcp;

class routed_wrapper : public osrm_wrapper {

private:
  std::string _address;
  std::string _port;

  std::string build_query(const std::vector<location_t>& locations,
                          std::string service,
                          std::string extra_args) const;

  std::string send_then_receive(std::string query) const;

public:
  routed_wrapper(const std::string& address,
                 const std::string& port,
                 const std::string& osrm_profile);

  virtual matrix<cost_t>
  get_matrix(const std::vector<location_t>& locs,
             std::vector<cost_t>& max_cost_per_line,
             std::vector<cost_t>& max_cost_per_column) const override;

  virtual void add_route_geometry(route_t& rte) const override;
};

#endif
