#ifndef VERSION_H
#define VERSION_H

/*

This file is part of VROOM.

Copyright (c) 2015-2016, Julien Coupey.
All rights reserved (see LICENSE).

*/

#define MAJOR 1
#define MINOR 1
#define PATCH 0
#define DEV 1
#define RC 0

std::string get_version(){
  std::string version = std::to_string(MAJOR)
    + "." + std::to_string(MINOR)
    + "." + std::to_string(PATCH);
  if(DEV){
    version += "-dev";
  }
  else{
    if(RC){
      version += "-rc." + std::to_string(RC);
    }
  }
  return version;
}

#endif
