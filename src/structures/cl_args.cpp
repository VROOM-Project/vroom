/*

This file is part of VROOM.

Copyright (c) 2015-2022, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <cassert>

#include "structures/cl_args.h"

namespace vroom::io {

void update_host(Servers& servers, const std::string& value) {
  // Determine profile and host from a "car:0.0.0.0"-like value.
  std::string profile = DEFAULT_PROFILE;
  std::string host;
  std::string path = "";

  auto index = value.find(':');
  if (index == std::string::npos) {
    host = value;
  } else {
    profile = value.substr(0, index);
    host = value.substr(index + 1);
  }

  if (!host.empty()) {
    // remove any trailing slash
    if (host.back() == '/') {
      host.pop_back();
    }

    // pull out a path if any and append a trailing slash for query building
    index = host.find('/');
    if (index != std::string::npos) {
      path = host.substr(index + 1) + "/";
      host = host.erase(index);
    }
  }

  auto existing_profile = servers.find(profile);
  if (existing_profile != servers.end()) {
    existing_profile->second.host = host;
    existing_profile->second.path = path;
  } else {
    auto add_result = servers.try_emplace(profile);
    assert(add_result.second);
    add_result.first->second.host = host;
    add_result.first->second.path = path;
  }
}

void update_port(Servers& servers, const std::string& value) {
  // Determine profile and port from a "car:0.0.0.0"-like value.
  std::string profile = DEFAULT_PROFILE;
  std::string port;

  auto index = value.find(':');
  if (index == std::string::npos) {
    port = value;
  } else {
    profile = value.substr(0, index);
    port = value.substr(index + 1);
  }

  auto existing_profile = servers.find(profile);
  if (existing_profile != servers.end()) {
    existing_profile->second.port = port;
  } else {
    auto add_result = servers.try_emplace(profile);
    assert(add_result.second);
    add_result.first->second.port = port;
  }
}

} // namespace vroom::io
