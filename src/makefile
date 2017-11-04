# This file is part of VROOM.
#
# Copyright (c) 2015-2017, Julien Coupey.
# All rights reserved (see LICENSE).

# Variables.
CXX = g++
CXXFLAGS = -std=c++14 -Wextra -Wpedantic -Wall -O3 -DBOOST_LOG_DYN_LINK
LDLIBS = -lboost_system -lboost_regex -lboost_log -lboost_log_setup -lpthread -lboost_thread

# Using all cpp files in current directory.
MAIN = ../bin/vroom
LIB = ../lib/libvroom.a
SRC = $(wildcard *.cpp)\
			$(wildcard ./algorithms/*.cpp)\
			$(wildcard ./routing/*.cpp)\
			$(wildcard ./problems/*.cpp)\
			$(wildcard ./problems/*/*.cpp)\
			$(wildcard ./problems/*/heuristics/*.cpp)\
			$(wildcard ./structures/abstract/*.cpp)\
			$(wildcard ./structures/vroom/*.cpp)\
			$(wildcard ./structures/vroom/input/*.cpp)\
			$(wildcard ./structures/vroom/solution/*.cpp)\
			$(wildcard ./structures/*.cpp)\
			$(wildcard ./utils/*.cpp)

# Checking for libosrm
ifeq ($(shell pkg-config --exists libosrm && echo 1),1)
LDLIBS += $(shell pkg-config --libs libosrm) -lboost_filesystem -lboost_iostreams -lrt -ltbb
CXXFLAGS += $(shell pkg-config --cflags libosrm) -D LIBOSRM=true
else
	SRC := $(filter-out ./routing/libosrm_wrapper.cpp, $(SRC))
endif

OBJ = $(SRC:.cpp=.o)

# Main target.
all : $(MAIN) $(LIB)

$(MAIN) : $(OBJ) main.o
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDLIBS)

$(LIB) : $(OBJ)
	$(AR) cr $@ $^

# Building .o files.
%.o : %.cpp %.h
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Specific target for main (no .h file).
main.o : main.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean :
	rm $(OBJ)
	rm $(MAIN)
