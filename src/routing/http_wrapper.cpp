/*

This file is part of VROOM.

Copyright (c) 2015-2019, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <boost/asio.hpp>

#include "routing/http_wrapper.h"
#include "utils/exception.h"

using boost::asio::ip::tcp;

namespace vroom {
namespace routing {

HttpWrapper::HttpWrapper(const Server& server) : _server(server) {
}

std::string HttpWrapper::send_then_receive(std::string query) const {
  std::string response;

  try {
    boost::asio::io_service io_service;

    tcp::resolver r(io_service);

    tcp::resolver::query q(_server.host, _server.port);

    tcp::socket s(io_service);
    boost::asio::connect(s, r.resolve(q));

    boost::asio::write(s, boost::asio::buffer(query));

    char buf[512];
    boost::system::error_code error;
    for (;;) {
      std::size_t len = s.read_some(boost::asio::buffer(buf), error);
      response.append(buf, len);
      if (error == boost::asio::error::eof) {
        // Connection closed cleanly.
        break;
      } else {
        if (error) {
          throw boost::system::system_error(error);
        }
      }
    }
  } catch (boost::system::system_error& e) {
    throw Exception(ERROR::ROUTING,
                    "Failed to connect to " + _server.host + ":" +
                      _server.port);
  }
  return response;
}

} // namespace routing
} // namespace vroom
