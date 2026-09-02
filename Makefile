# Mini-CPU — 7-stage out-of-order RV32IM simulator
#
# Targets: all (release) · debug (ASan+UBSan) · test · trace · examples · clean · help

CXX      ?= g++
CXXSTD    = -std=c++17
CXXWARN   = -Wall -Wextra -Wpedantic
CXXINC    = -Isrc -Itests

CXXFLAGS_REL = $(CXXSTD) -O2 $(CXXWARN) $(CXXINC)
CXXFLAGS_DBG = $(CXXSTD) -O1 -g $(CXXWARN) $(CXXINC) -fsanitize=address,undefined
LDFLAGS_DBG  = -fsanitize=address,undefined

BUILD  = build
OBJDIR = $(BUILD)/obj
DBGDIR = $(BUILD)/obj-dbg

# Wildcards, so a new source file needs no Makefile edit.
CPU_SRC = $(wildcard src/*.cpp)
CPU_OBJ = $(patsubst src/%.cpp,$(OBJDIR)/%.o,$(CPU_SRC))
DBG_OBJ = $(patsubst src/%.cpp,$(DBGDIR)/%.o,$(CPU_SRC))

# One test binary per tests/test_*.cpp; test_support.h provides main().
TEST_SRC   = $(wildcard tests/test_*.cpp)
TEST_BIN   = $(patsubst tests/%.cpp,$(BUILD)/%,$(TEST_SRC))
TEST_DBG   = $(patsubst tests/%.cpp,$(BUILD)/%-debug,$(TEST_SRC))
TOOLS_SRC  = tools/gen_examples.cpp
HDR        = $(wildcard src/*.h) $(wildcard tests/*.h)

# The tests reuse every src/*.cpp but main.cpp, whose main() test_pipeline
# replaces by #including it.
LIB_SRC = $(filter-out src/main.cpp,$(CPU_SRC))

.PHONY: all debug test examples trace clean help
.DEFAULT_GOAL := all

# ---------------------------------------------------------------- release ---
all: $(BUILD)/oooc

$(BUILD)/oooc: $(CPU_OBJ) | $(BUILD)
	@if [ -z "$(CPU_OBJ)" ]; then echo "no src/*.cpp to build"; exit 1; fi
	$(CXX) $(CXXFLAGS_REL) $(CPU_OBJ) -o $@

$(OBJDIR)/%.o: src/%.cpp | $(OBJDIR)
	$(CXX) $(CXXFLAGS_REL) -MMD -MP -c $< -o $@

# ------------------------------------------------------------------ test ---
# Each test binary is one translation unit plus the simulator sources; they
# pull in nearly every header, so they rebuild on any header change rather
# than being tracked dependency by dependency.
$(BUILD)/test_%: tests/test_%.cpp $(LIB_SRC) $(CPU_SRC) $(HDR) | $(BUILD)
	$(CXX) $(CXXFLAGS_REL) $< $(LIB_SRC) -o $@

test: $(TEST_BIN) examples
	@fail=0; \
	for t in $(TEST_BIN); do \
	  echo "==== $$t ===="; \
	  ./$$t || fail=1; \
	done; \
	exit $$fail

# ------------------------------------------------------ debug / sanitized ---
# Builds the instrumented CLI binary and runs the whole suite under
# ASan + UBSan.
debug: $(BUILD)/oooc-debug $(TEST_DBG) examples
	@fail=0; \
	for t in $(TEST_DBG); do \
	  echo "==== $$t ===="; \
	  ./$$t || fail=1; \
	done; \
	exit $$fail

$(BUILD)/oooc-debug: $(DBG_OBJ) | $(BUILD)
	@if [ -z "$(DBG_OBJ)" ]; then echo "no src/*.cpp to build"; exit 1; fi
	$(CXX) $(CXXFLAGS_DBG) $(DBG_OBJ) -o $@ $(LDFLAGS_DBG)

$(DBGDIR)/%.o: src/%.cpp | $(DBGDIR)
	$(CXX) $(CXXFLAGS_DBG) -MMD -MP -c $< -o $@

$(BUILD)/test_%-debug: tests/test_%.cpp $(LIB_SRC) $(CPU_SRC) $(HDR) | $(BUILD)
	$(CXX) $(CXXFLAGS_DBG) $< $(LIB_SRC) -o $@ $(LDFLAGS_DBG)

# ----------------------------------------------------------------- trace ---
# A cycle trace of a bundled program, sized for the visualiser rather than for
# a full run: records average ~8 KB, so a window is what you want.
TRACE_PROG   ?= fib
TRACE_CYCLES ?= 2000
TRACE_OUT     = $(BUILD)/$(TRACE_PROG).ndjson

# oooc exits with the traced program's own exit code, so the status says
# nothing about whether the trace was written — the file does.
trace: all examples
	./$(BUILD)/oooc --hex examples/$(TRACE_PROG).hex --base 0x1000 \
	  --trace=$(TRACE_OUT) --trace-max $(TRACE_CYCLES) || true
	@test -s $(TRACE_OUT) || { echo "trace: nothing written to $(TRACE_OUT)"; exit 1; }
	@echo "open tools/oooviz.html in a browser and load $(TRACE_OUT)"

# --------------------------------------------------------------- tooling ---
$(BUILD)/gen_examples: $(TOOLS_SRC) tests/workloads.h tests/asm.h | $(BUILD)
	$(CXX) $(CXXFLAGS_REL) $(TOOLS_SRC) -o $@

examples: $(BUILD)/gen_examples
	@mkdir -p examples && ./$(BUILD)/gen_examples

# ---------------------------------------------------------------- housekeeping
$(BUILD) $(OBJDIR) $(DBGDIR):
	@mkdir -p $@

clean:
	rm -rf $(BUILD) examples

help:
	@echo "Mini-CPU targets:"
	@echo "  all      build/oooc          (release, -O2)"
	@echo "  test     build and run the six test binaries (release)"
	@echo "  debug    the same suite plus build/oooc-debug under ASan + UBSan"
	@echo "  trace    write a cycle trace for tools/oooviz.html"
	@echo "           (TRACE_PROG=fib TRACE_CYCLES=2000)"
	@echo "  examples assemble the bundled programs into examples/"
	@echo "  clean    remove build/ and examples/"

# Auto-generated header dependencies.
-include $(CPU_OBJ:.o=.d) $(DBG_OBJ:.o=.d)
