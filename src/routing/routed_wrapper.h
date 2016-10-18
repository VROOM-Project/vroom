#ifndef ROUTED_WRAPPER_H
#define ROUTED_WRAPPER_H

/*

This file is part of VROOM.

Copyright (c) 2015-2016, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "./osrm_wrapper.h"
#include <boost/asio.hpp>

using boost::asio::ip::tcp;

class routed_wrapper : public osrm_wrapper{

private:
  std::string _address;            // OSRM server adress
  std::string _port;               // OSRM server listening port

  std::string build_query(const std::vector<std::reference_wrapper<location>>& locations,
                          std::string service,
                          std::string extra_args) const;

  std::string send_then_receive(std::string query) const;

public:
  routed_wrapper(const std::string& address,
                 const std::string& port,
                 const std::string& osrm_profile);

  virtual matrix<distance_t> get_matrix(const std::vector<std::reference_wrapper<location>>& locs) const override;

  virtual void get_route_infos(const std::vector<std::reference_wrapper<location>>& locs,
                               const std::list<index_t>& steps,
                               rapidjson::Value& value,
                               rapidjson::Document::AllocatorType& allocator) const override;
};

#endif
