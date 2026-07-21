CXX      ?= c++
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra -Wno-unused-parameter
BUILD    := build

CORE_SRC := src/cpu.cpp src/decoder.cpp
SIM_SRC  := $(CORE_SRC) src/main.cpp
TEST_SRC := $(CORE_SRC) tests/test_main.cpp

SIM      := $(BUILD)/oooc
TEST     := $(BUILD)/tests
GEN      := $(BUILD)/gen-examples

.PHONY: all test examples clean debug

all: $(SIM) examples

$(SIM): $(SIM_SRC) $(wildcard src/*.h) | $(BUILD)
	$(CXX) $(CXXFLAGS) $(SIM_SRC) -o $@

$(TEST): $(TEST_SRC) $(wildcard src/*.h) $(wildcard tests/*.h) | $(BUILD)
	$(CXX) $(CXXFLAGS) $(TEST_SRC) -o $@

$(GEN): tools/gen_examples.cpp tests/asm.h | $(BUILD)
	$(CXX) $(CXXFLAGS) $< -o $@

$(BUILD):
	@mkdir -p $(BUILD)

test: $(TEST)
	@./$(TEST)

examples: $(GEN)
	@mkdir -p examples
	@./$(GEN) examples

# Build with assertions and sanitizers enabled.
debug: CXXFLAGS := -std=c++17 -O1 -g -Wall -Wextra -Wno-unused-parameter \
                   -fsanitize=address,undefined
debug: clean $(TEST)
	@./$(TEST)

clean:
	rm -rf $(BUILD)
