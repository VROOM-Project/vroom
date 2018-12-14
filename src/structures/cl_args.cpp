/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/cl_args.h"

const unsigned CLArgs::max_exploration_level = 5;

// Default values.
CLArgs::CLArgs()
  : osrm_address("0.0.0.0"),
    geometry(false),
    osrm_port("5000"),
    use_libosrm(false),
    nb_threads(4),
    osrm_profile("car"),
    exploration_level(5) {
}
