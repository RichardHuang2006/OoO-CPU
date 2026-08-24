# Mini-CPU — 7-stage out-of-order RV32IM simulator
#
# Targets: all (release) · debug (ASan+UBSan) · test · clean · help

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

TEST_SRC   = tests/test_main.cpp
TOOLS_SRC  = tools/gen_examples.cpp
HDR        = $(wildcard src/*.h) $(wildcard tests/*.h)

# The tests reuse every src/*.cpp but main.cpp, whose main() they replace by
# #including it.
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

# ------------------------------------------------------ debug / sanitized ---
# Builds the instrumented CLI binary and runs the full test suite under
# ASan + UBSan.
debug: $(BUILD)/oooc-debug $(BUILD)/test_main-debug examples
	./$(BUILD)/test_main-debug

$(BUILD)/oooc-debug: $(DBG_OBJ) | $(BUILD)
	@if [ -z "$(DBG_OBJ)" ]; then echo "no src/*.cpp to build"; exit 1; fi
	$(CXX) $(CXXFLAGS_DBG) $(DBG_OBJ) -o $@ $(LDFLAGS_DBG)

$(DBGDIR)/%.o: src/%.cpp | $(DBGDIR)
	$(CXX) $(CXXFLAGS_DBG) -MMD -MP -c $< -o $@

$(BUILD)/test_main-debug: $(TEST_SRC) $(CPU_SRC) $(HDR) | $(BUILD)
	@if [ ! -f $(TEST_SRC) ]; then echo "no $(TEST_SRC)"; exit 1; fi
	$(CXX) $(CXXFLAGS_DBG) $(TEST_SRC) $(LIB_SRC) -o $@ $(LDFLAGS_DBG)

# --------------------------------------------------------------- tooling ---
$(BUILD)/gen_examples: $(TOOLS_SRC) | $(BUILD)
	$(CXX) $(CXXFLAGS_REL) $(TOOLS_SRC) -o $@

# Skipped while the generator does not exist, rather than failing on a
# prerequisite that cannot be built.
examples:
	@if [ -f $(TOOLS_SRC) ]; then \
	  $(MAKE) --no-print-directory $(BUILD)/gen_examples && \
	  mkdir -p examples && ./$(BUILD)/gen_examples; \
	else \
	  echo "examples: skipped, no $(TOOLS_SRC)"; \
	fi

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

# ------------------------------------------------------------------ test ---
# One translation unit that pulls in nearly every header, so it is rebuilt on
# any header change rather than tracked dependency by dependency.
$(BUILD)/test_main: $(TEST_SRC) $(CPU_SRC) $(HDR) | $(BUILD)
	@if [ ! -f $(TEST_SRC) ]; then echo "no $(TEST_SRC)"; exit 1; fi
	$(CXX) $(CXXFLAGS_REL) $(TEST_SRC) $(LIB_SRC) -o $@

test: $(BUILD)/test_main examples
	./$(BUILD)/test_main

# ---------------------------------------------------------------- housekeeping
$(BUILD) $(OBJDIR) $(DBGDIR):
	@mkdir -p $@

clean:
	rm -rf $(BUILD) examples

help:
	@echo "Mini-CPU targets:"
	@echo "  all      build/oooc         (release, -O2, warnings-as-errors off)"
	@echo "  debug    build/oooc-debug + run the test suite under ASan + UBSan"
	@echo "  test     compile+run tests/test_main after regenerating examples/"
	@echo "  trace    write a cycle trace for tools/oooviz.html"
	@echo "           (TRACE_PROG=fib TRACE_CYCLES=2000)"
	@echo "  clean    remove build/ and examples/"

# Auto-generated header dependencies.
-include $(CPU_OBJ:.o=.d) $(DBG_OBJ:.o=.d)
