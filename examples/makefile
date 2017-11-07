CXX = g++
CXXFLAGS = -std=c++14 -Wextra -Wpedantic -Wall -O3 -DBOOST_LOG_DYN_LINK
LDLIBS = -L../lib/ -lvroom -lboost_system -lboost_log -lboost_log_setup -lpthread -lboost_thread

MAIN = ./libvroom-example
SRC = libvroom.cpp

all : $(MAIN)

$(MAIN) : $(SRC)
	$(CXX) $(CXXFLAGS) $^ $(LDLIBS) -o $@

clean :
	rm $(MAIN)
