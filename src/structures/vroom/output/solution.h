#ifndef SOLUTION_H
#define SOLUTION_H

/*

This file is part of VROOM.

Copyright (c) 2015-2016, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "./route.h"
#include "./summary.h"

struct solution{
  index_t code;
  boost::optional<std::string> error;
  const std::vector<route_t> routes;
  const boost::optional<summary_t> summary;

  solution(index_t code, std::string error):
    code(code),
    error(error){}

  solution(index_t code,
           const std::vector<route_t>& routes,
           summary_t summary):
    code(code),
    routes(std::move(routes)),
    summary(summary){}
};

#endif
