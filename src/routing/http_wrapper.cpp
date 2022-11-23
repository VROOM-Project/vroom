/*

This file is part of VROOM.

Copyright (c) 2015-2022, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <asio.hpp>
#include <asio/ssl.hpp>

#include "routing/http_wrapper.h"

using asio::ip::tcp;

namespace vroom {
namespace routing {

const std::string HttpWrapper::HTTPS_PORT = "443";

HttpWrapper::HttpWrapper(const std::string& profile,
                         const Server& server,
                         const std::string& matrix_service,
                         const std::string& matrix_durations_key,
                         const std::string& route_service,
                         const std::string& extra_args)
  : Wrapper(profile),
    _server(server),
    _matrix_service(matrix_service),
    _matrix_durations_key(matrix_durations_key),
    _route_service(route_service),
    _extra_args(extra_args) {
}

std::string HttpWrapper::send_then_receive(const std::string& query) const {
  std::string response;

  try {
    asio::io_service io_service;

    tcp::resolver r(io_service);

    tcp::resolver::query q(_server.host, _server.port);

    tcp::socket s(io_service);
    asio::connect(s, r.resolve(q));

    asio::write(s, asio::buffer(query));

    char buf[512];
    std::error_code error;
    for (;;) {
      std::size_t len = s.read_some(asio::buffer(buf), error);
      response.append(buf, len);
      if (error == asio::error::eof) {
        // Connection closed cleanly.
        break;
      } else {
        if (error) {
          throw std::system_error(error);
        }
      }
    }
  } catch (std::system_error& e) {
    throw RoutingException("Failed to connect to " + _server.host + ":" +
                           _server.port);
  }

  // Removing headers.
  auto start = response.find("{");
  if (start == std::string::npos) {
    throw RoutingException("Invalid routing response: " + response);
  }
  auto end = response.rfind("}");
  if (end == std::string::npos) {
    throw RoutingException("Invalid routing response: " + response);
  }

  std::string json_string = response.substr(start, end - start + 1);

  return json_string;
}

std::string HttpWrapper::ssl_send_then_receive(const std::string& query) const {
  std::string response;

  try {
    asio::io_service io_service;

    asio::ssl::context ctx(asio::ssl::context::method::sslv23_client);
    asio::ssl::stream<asio::ip::tcp::socket> ssock(io_service, ctx);

    tcp::resolver r(io_service);

    tcp::resolver::query q(_server.host, _server.port);

    asio::connect(ssock.lowest_layer(), r.resolve(q));
    ssock.handshake(asio::ssl::stream_base::handshake_type::client);

    asio::write(ssock, asio::buffer(query));

    char buf[512];
    std::error_code error;
    for (;;) {
      std::size_t len = ssock.read_some(asio::buffer(buf), error);
      response.append(buf, len);
      if (error == asio::error::eof) {
        // Connection closed cleanly.
        break;
      } else {
        if (error) {
          throw std::system_error(error);
        }
      }
    }
  } catch (std::system_error& e) {
    throw RoutingException("Failed to connect to " + _server.host + ":" +
                           _server.port);
  }

  // Removing headers.
  auto start = response.find("{");
  if (start == std::string::npos) {
    throw RoutingException("Invalid routing response: " + response);
  }
  auto end = response.rfind("}");
  if (end == std::string::npos) {
    throw RoutingException("Invalid routing response: " + response);
  }
  std::string json_string = response.substr(start, end - start + 1);

  return json_string;
}

std::string HttpWrapper::run_query(const std::string& query) const {
  return (_server.port == HTTPS_PORT) ? ssl_send_then_receive(query)
                                      : send_then_receive(query);
}

void HttpWrapper::parse_response(rapidjson::Document& json_result,
                                 const std::string& json_content) const {
#ifdef NDEBUG
  json_result.Parse(json_content.c_str());
#else
  assert(!json_result.Parse(json_content.c_str()).HasParseError());
#endif
}

Matrix<UserCost>
HttpWrapper::get_matrix(const std::vector<Location>& locs) const {
  std::string query = this->build_query(locs, _matrix_service);
  std::string json_string = this->run_query(query);

  // Expected matrix size.
  std::size_t m_size = locs.size();

  rapidjson::Document json_result;
  this->parse_response(json_result, json_string);
  this->check_response(json_result, _matrix_service);

  if (!json_result.HasMember(_matrix_durations_key.c_str())) {
    throw RoutingException("Missing " + _matrix_durations_key + ".");
  }
  assert(json_result[_matrix_durations_key.c_str()].Size() == m_size);

  // Build matrix while checking for unfound routes ('null' values) to
  // avoid unexpected behavior.
  Matrix<UserCost> m(m_size);

  std::vector<unsigned> nb_unfound_from_loc(m_size, 0);
  std::vector<unsigned> nb_unfound_to_loc(m_size, 0);

  for (rapidjson::SizeType i = 0; i < m_size; ++i) {
    const auto& line = json_result[_matrix_durations_key.c_str()][i];
    assert(line.Size() == m_size);
    for (rapidjson::SizeType j = 0; j < line.Size(); ++j) {
      if (duration_value_is_null(line[j])) {
        // No route found between i and j. Just storing info as we
        // don't know yet which location is responsible between i
        // and j.
        ++nb_unfound_from_loc[i];
        ++nb_unfound_to_loc[j];
      } else {
        m[i][j] = get_duration_value(line[j]);
      }
    }
  }

  check_unfound(locs, nb_unfound_from_loc, nb_unfound_to_loc);

  return m;
}

void HttpWrapper::add_route_info(Route& route) const {
  // Ordering locations for the given steps, excluding
  // breaks.
  std::vector<Location> non_break_locations;
  std::vector<unsigned> number_breaks_after;

  for (const auto& step : route.steps) {
    if (step.step_type == STEP_TYPE::BREAK) {
      if (!number_breaks_after.empty()) {
        ++(number_breaks_after.back());
      }
    } else {
      non_break_locations.push_back(step.location);
      number_breaks_after.push_back(0);
    }
  }
  assert(!non_break_locations.empty());

  std::string query =
    build_query(non_break_locations, _route_service, _extra_args);

  std::string json_string = this->run_query(query);

  rapidjson::Document json_result;
  parse_response(json_result, json_string);
  this->check_response(json_result, _route_service);

  // Total distance and route geometry.
  route.distance = round_cost(get_total_distance(json_result));
  route.geometry = get_geometry(json_result);

  auto nb_legs = get_legs_number(json_result);
  assert(nb_legs == non_break_locations.size() - 1);

  double sum_distance = 0;

  // Start step has zero distance.
  unsigned steps_rank = 0;
  route.steps[0].distance = 0;

  for (rapidjson::SizeType i = 0; i < nb_legs; ++i) {
    const auto& step = route.steps[steps_rank];

    // Next element in steps that is not a break and associated
    // distance after current route leg.
    auto& next_step = route.steps[steps_rank + number_breaks_after[i] + 1];
    assert(step.duration <= next_step.duration);
    auto next_duration = next_step.duration - step.duration;
    double next_distance = get_distance_for_leg(json_result, i);

    // Pro rata temporis distance update for breaks between current
    // non-breaks steps.
    for (unsigned b = 1; b <= number_breaks_after[i]; ++b) {
      auto& break_step = route.steps[steps_rank + b];
      if (next_duration == 0) {
        break_step.distance = round_cost(sum_distance);
      } else {
        break_step.distance =
          round_cost(sum_distance +
                     ((break_step.duration - step.duration) * next_distance) /
                       next_duration);
      }
    }

    sum_distance += next_distance;
    next_step.distance = round_cost(sum_distance);

    steps_rank += number_breaks_after[i] + 1;
  }
}

} // namespace routing
} // namespace vroom
