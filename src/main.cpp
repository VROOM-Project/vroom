/*
VROOM (Vehicle Routing Open-source Optimization Machine)
Copyright (C) 2015, Julien Coupey

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or (at
your option) any later version.

This program is distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <string>
#include <sstream>
#include <unistd.h>
#include "./structures/typedefs.h"
#include "./heuristics/tsp_strategy.h"

void display_usage(){
  std::string usage = "VROOM Copyright (C) 2015, Julien Coupey\n";
  usage += "This program is distributed under the terms of the GNU General Public\n";
  usage += "License, version 3, and comes with ABSOLUTELY NO WARRANTY.\n";
  std::cout << usage << std::endl;
  exit(0);
}

int main(int argc, char **argv){
  // Default options.
  cl_args_t cl_args;
  cl_args.loader = 0;
  cl_args.osrm_address = "0.0.0.0";
  cl_args.osrm_port = 5000;

  // Parsing command-line arguments.
  const char* optString = "a:ei:o:p:vh?";
  int opt = getopt(argc, argv, optString);

  while(opt != -1) {
    switch(opt){
    case 'a':
      cl_args.osrm_address = optarg;
      break;
    case 'e':
      cl_args.loader = 1;
      break;
    case 'h':
      display_usage();
      break;
    case 'i':
      cl_args.input_file = optarg;
      break;
    case 'o':
      cl_args.output_file = optarg;
      break;
    case 'p':
      cl_args.osrm_port = strtol(optarg, nullptr, 10);
      break;
    case 'v':
      cl_args.verbose = true;
      break;
    default:
      break;
    }
    opt = getopt(argc, argv, optString);
  }

  if(cl_args.input_file.empty()){
    // Getting list of locations from command-line.
    if(argc == optind){
      // Missing argument!
      display_usage();
    }
    cl_args.places = argv[optind];
  }
  else{
    // Getting list of locations from input file. Fast way to get the
    // file content as a string, not meant for large files...
    std::ifstream ifs (cl_args.input_file, std::ifstream::in);
    std::stringstream buffer;
    buffer << ifs.rdbuf();
    cl_args.places = buffer.str();
  }
  
  solve_atsp(cl_args);

  return 0;
}
