# This file is part of VROOM.
#
# Copyright (c) 2015-2018, Julien Coupey.
# All rights reserved (see LICENSE).

# Variables.
CXX ?= g++
CXXFLAGS = -MMD -MP -std=c++14 -Wextra -Wpedantic -Wall -O3 -DBOOST_LOG_DYN_LINK
LDLIBS = -lboost_system -lboost_log -lboost_log_setup -lpthread -lboost_thread

# Using all cpp files in current directory.
MAIN = ../bin/vroom
LIB = ../lib/libvroom.a
SRC = $(wildcard *.cpp)\
			$(wildcard ./algorithms/*.cpp)\
			$(wildcard ./routing/*.cpp)\
			$(wildcard ./problems/*.cpp)\
			$(wildcard ./problems/*/*.cpp)\
			$(wildcard ./problems/*/heuristics/*.cpp)\
			$(wildcard ./problems/*/local_search/*.cpp)\
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
DEPS = $(SRC:.cpp=.d)

# Main target.
all : $(MAIN) $(LIB)

$(MAIN) : $(OBJ) main.o
	mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDLIBS)

$(LIB) : $(OBJ)
	mkdir -p $(@D)
	$(AR) cr $@ $^

# Building .o files.
%.o : %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

-include ${DEPS}

clean :
	$(RM) $(OBJ) $(DEPS)
	$(RM) $(MAIN)
	$(RM) $(LIB)
