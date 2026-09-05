CXX ?= c++
CXXFLAGS ?= -O3 -std=c++20 -Wall -Wextra -Wpedantic -Werror
CPPFLAGS := -Iinclude
.PHONY: all test sanitize clean
all: build/experiment
build:
	mkdir -p build
build/experiment: src/experiment.cpp include/motion_guard/geometry.hpp include/motion_guard/world.hpp | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< -o $@
build/test: tests/test.cpp include/motion_guard/geometry.hpp include/motion_guard/world.hpp | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< -o $@
test: build/test build/experiment
	./build/test
	./build/experiment --scenario
sanitize:
	$(MAKE) clean
	$(MAKE) test CXXFLAGS='-O1 -g -std=c++20 -Wall -Wextra -Wpedantic -Werror -fsanitize=address,undefined -fno-omit-frame-pointer'
clean:
	rm -rf build
