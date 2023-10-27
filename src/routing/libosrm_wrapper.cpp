/*

This file is part of VROOM.

Copyright (c) 2015-2022, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <cstdint>

#include "osrm/coordinate.hpp"
#include "osrm/json_container.hpp"
#include "osrm/route_parameters.hpp"
#include "osrm/status.hpp"
#include "osrm/table_parameters.hpp"

#include "routing/libosrm_wrapper.h"
#include "utils/helpers.h"

namespace vroom {
namespace routing {

osrm::EngineConfig LibosrmWrapper::get_config(const std::string& profile) {
  osrm::EngineConfig config;

  // Only update non-default values.
  config.max_alternatives = 1;
  config.dataset_name = profile;

  return config;
}

LibosrmWrapper::LibosrmWrapper(const std::string& profile)
  : Wrapper(profile), _config(get_config(profile)), _osrm(_config) {
}

Matrices LibosrmWrapper::get_matrices(const std::vector<Location>& locs) const {
  osrm::TableParameters params;
  params.annotations = osrm::engine::api::TableParameters::AnnotationsType::All;

  params.coordinates.reserve(locs.size());
  params.radiuses.reserve(locs.size());
  for (auto const& location : locs) {
    assert(location.has_coordinates());
    params.coordinates
      .emplace_back(osrm::util::FloatLongitude({location.lon()}),
                    osrm::util::FloatLatitude({location.lat()}));
    params.radiuses.emplace_back(DEFAULT_LIBOSRM_SNAPPING_RADIUS);
  }

  osrm::json::Object result;
  osrm::Status status = _osrm.Table(params, result);

  if (status == osrm::Status::Error) {
    const std::string code =
      result.values["code"].get<osrm::json::String>().value;
    const std::string message =
      result.values["message"].get<osrm::json::String>().value;

    const std::string snapping_error_base =
      "Could not find a matching segment for coordinate ";
    if (code == "NoSegment" && message.starts_with(snapping_error_base)) {
      auto error_loc =
        std::stoul(message.substr(snapping_error_base.size(),
                                  message.size() - snapping_error_base.size()));
      auto coordinates = "[" + std::to_string(locs[error_loc].lon()) + "," +
                         std::to_string(locs[error_loc].lat()) + "]";
      throw RoutingException("Could not find route near location " +
                             coordinates);
    }

    // Other error in response.
    throw RoutingException("libOSRM: " + code + ": " + message);
  }

  const auto& durations = result.values["durations"].get<osrm::json::Array>();
  const auto& distances = result.values["distances"].get<osrm::json::Array>();

  // Expected matrix size.
  std::size_t m_size = locs.size();
  assert(durations.values.size() == m_size);
  assert(distances.values.size() == m_size);

  // Build matrix while checking for unfound routes to avoid
  // unexpected behavior (OSRM raises 'null').
  Matrices m(m_size);

  std::vector<unsigned> nb_unfound_from_loc(m_size, 0);
  std::vector<unsigned> nb_unfound_to_loc(m_size, 0);

  std::string reason;
  for (std::size_t i = 0; i < m_size; ++i) {
    const auto& duration_line = durations.values.at(i).get<osrm::json::Array>();
    const auto& distance_line = distances.values.at(i).get<osrm::json::Array>();
    assert(duration_line.values.size() == m_size);
    assert(distance_line.values.size() == m_size);

    for (std::size_t j = 0; j < m_size; ++j) {
      const auto& duration_el = duration_line.values.at(j);
      const auto& distance_el = distance_line.values.at(j);
      if (duration_el.is<osrm::json::Null>() ||
          distance_el.is<osrm::json::Null>()) {
        // No route found between i and j. Just storing info as we
        // don't know yet which location is responsible between i
        // and j.
        ++nb_unfound_from_loc[i];
        ++nb_unfound_to_loc[j];
      } else {
        m.durations[i][j] = utils::round<UserDuration>(
          duration_el.get<osrm::json::Number>().value);
        m.distances[i][j] = utils::round<UserDistance>(
          distance_el.get<osrm::json::Number>().value);
      }
    }
  }

  check_unfound(locs, nb_unfound_from_loc, nb_unfound_to_loc);

  return m;
}

void LibosrmWrapper::add_geometry(Route& route) const {
  // Default options for routing.
  osrm::RouteParameters params(false, // steps
                               false, // alternatives
                               false, // annotations
                               osrm::RouteParameters::GeometriesType::Polyline,
                               osrm::RouteParameters::OverviewType::Full,
                               false // continue_straight
  );
  params.coordinates.reserve(route.steps.size());

  // Ordering locations for the given steps, excluding
  // breaks.
  for (auto& step : route.steps) {
    if (step.step_type != STEP_TYPE::BREAK) {
      assert(step.location.has_value());
      const auto& loc = step.location.value();
      assert(loc.has_coordinates());
      params.coordinates.emplace_back(osrm::util::FloatLongitude({loc.lon()}),
                                      osrm::util::FloatLatitude({loc.lat()}));
    }
  }

  osrm::json::Object result;
  osrm::Status status = _osrm.Route(params, result);

  if (status == osrm::Status::Error) {
    throw RoutingException(
      result.values["code"].get<osrm::json::String>().value + ": " +
      result.values["message"].get<osrm::json::String>().value);
  }

  auto& result_routes = result.values["routes"].get<osrm::json::Array>();
  auto& json_route = result_routes.values.at(0).get<osrm::json::Object>();

  // Total distance and route geometry.
  route.geometry =
    std::move(json_route.values["geometry"].get<osrm::json::String>().value);
}

} // namespace routing
} // namespace vroom
