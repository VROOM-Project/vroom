/*

This file is part of VROOM.

Copyright (c) 2015-2020, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <boost/asio.hpp>
#include <boost/asio/ssl.hpp>

#include "routing/http_wrapper.h"
#include "utils/exception.h"

using boost::asio::ip::tcp;

namespace vroom {
namespace routing {

const std::string HttpWrapper::HTTPS_PORT = "443";

HttpWrapper::HttpWrapper(const Server& server) : _server(server) {
}

std::string HttpWrapper::send_then_receive(const std::string& query) const {
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

  // Removing headers.
  auto start = response.find("{");
  assert(start != std::string::npos);
  auto end = response.rfind("}");
  assert(end != std::string::npos);

  std::string json_content = response.substr(start, end - start + 1);

  return json_content;
}

std::string HttpWrapper::ssl_send_then_receive(const std::string& query) const {
  std::string response;

  try {
    boost::asio::io_service io_service;

    boost::asio::ssl::context ctx(
      boost::asio::ssl::context::method::sslv23_client);
    boost::asio::ssl::stream<boost::asio::ip::tcp::socket> ssock(io_service,
                                                                 ctx);

    tcp::resolver r(io_service);

    tcp::resolver::query q(_server.host, _server.port);

    boost::asio::connect(ssock.lowest_layer(), r.resolve(q));
    ssock.handshake(boost::asio::ssl::stream_base::handshake_type::client);

    boost::asio::write(ssock, boost::asio::buffer(query));

    char buf[512];
    boost::system::error_code error;
    for (;;) {
      std::size_t len = ssock.read_some(boost::asio::buffer(buf), error);
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

  // Removing headers.
  auto start = response.find("{");
  assert(start != std::string::npos);
  auto end = response.rfind("}");
  assert(end != std::string::npos);

  std::string json_content = response.substr(start, end - start + 1);

  return json_content;
}

std::string HttpWrapper::run_query(const std::string& query) const {
  return (_server.port == HTTPS_PORT) ? ssl_send_then_receive(query)
                                      : send_then_receive(query);
}

} // namespace routing
} // namespace vroom
