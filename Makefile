CXX      ?= c++
CXXFLAGS ?= -std=c++11 -O2 -Wall -Wextra -Wno-unused-parameter

ifeq ($(shell uname),Darwin)
LIBS = -lncurses
else
LIBS = -lncursesw -lutil
endif

nanoemu: nanoemu.cpp
	$(CXX) $(CXXFLAGS) -o $@ $< $(LIBS)

# Rebuild the example firmware (needs arduino-cli with the arduino:avr core)
examples:
	./examples/build.sh

clean:
	rm -f nanoemu

.PHONY: examples clean
