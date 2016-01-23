# VROOM (Vehicle Routing Open-source Optimization Machine)
# Copyright (C) 2015, Julien Coupey

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or (at
# your option) any later version.

# This program is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
# General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

# Variables.
CC = g++
FLAGS = -std=c++14 -Wpedantic -Wall -O3
LDLIBS = -lboost_system -lboost_regex -lpthread

# Using all cpp files in current directory.
MAIN = ../bin/vroom
SRC = $(wildcard *.cpp)\
			$(wildcard ./algorithms/*.cpp)\
			$(wildcard ./heuristics/*.cpp)\
			$(wildcard ./structures/*.cpp)\
			$(wildcard ./utils/*.cpp)
OBJ = $(SRC:.cpp=.o)

# Main target.
all : $(MAIN)

$(MAIN) : $(OBJ) main.o
	$(CC) $(FLAGS) -o $@ $^ $(LDLIBS) 

# Building .o files.
%.o : %.cpp %.h
	$(CC) $(FLAGS) -c $< -o $@

# Specific target for main (no .h file).
main.o : main.cpp
	$(CC) $(FLAGS) -c $< -o $@

clean : 
	rm $(OBJ)
	rm $(MAIN)
