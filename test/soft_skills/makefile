# Soft-vehicle-skills test harness.
#
# Scaffold makefile mirroring libvroom_examples/makefile: test objects are
# compiled against the libvroom headers (-I../../src) and linked against the
# static libvroom archive, which is built on demand via the main src makefile.

CXX ?= g++
USE_ROUTING ?= true

CXXFLAGS = -MMD -MP -I../../src -std=c++20 -Wextra -Wpedantic -Wall -O3
LDLIBS = -L../../lib -lvroom -lpthread -lssl -lcrypto

# Checking for libglpk based on whether the header file is found as
# glpk does not provide a pkg-config setup.
GLPK_HEADER := $(strip $(wildcard /usr/include/glpk.h))
ifneq ($(GLPK_HEADER),)
	LDLIBS += -lglpk
endif

RUNNER = test_runner
SRC = test_runner.cpp \
			test_matching.cpp \
			test_threshold.cpp \
			test_parsing.cpp \
			test_solver.cpp \
			test_output.cpp \
			test_thread_safety.cpp
OBJ = $(SRC:.cpp=.o)
DEPS = $(SRC:.cpp=.d)

LIBVROOM = ../../lib/libvroom.a

# Main target.
all : $(RUNNER)

$(RUNNER) : $(OBJ) $(LIBVROOM)
	$(CXX) $(CXXFLAGS) -o $@ $(OBJ) $(LDLIBS)

# Build libvroom on demand through the main makefile (same archive that
# bin/vroom links against).
$(LIBVROOM) :
	$(MAKE) -C ../../src USE_ROUTING=$(USE_ROUTING) ../lib/libvroom.a

# Building .o files.
%.o : %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

-include ${DEPS}

clean :
	$(RM) $(OBJ) $(DEPS)
	$(RM) $(RUNNER)

.PHONY : all clean
