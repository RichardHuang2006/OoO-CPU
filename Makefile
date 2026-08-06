# Mini-CPU — 7-stage out-of-order RV32IM simulator
#
# Targets: all (release) · debug (ASan+UBSan) · test · clean · help
# See PLAN.md for the step-by-step build order.

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

# Wildcards let each PLAN.md step add its source file with no Makefile edit.
CPU_SRC = $(wildcard src/*.cpp)
CPU_OBJ = $(patsubst src/%.cpp,$(OBJDIR)/%.o,$(CPU_SRC))
DBG_OBJ = $(patsubst src/%.cpp,$(DBGDIR)/%.o,$(CPU_SRC))

TEST_SRC   = tests/test_main.cpp
TOOLS_SRC  = tools/gen_examples.cpp

# The test binary reuses every src/*.cpp except src/main.cpp, whose main() the
# test harness replaces with its own.
LIB_SRC = $(filter-out src/main.cpp,$(CPU_SRC))

.PHONY: all debug test examples clean help
.DEFAULT_GOAL := all

# ---------------------------------------------------------------- release ---
all: $(BUILD)/oooc

$(BUILD)/oooc: $(CPU_OBJ) | $(BUILD)
	@if [ -z "$(CPU_OBJ)" ]; then \
	  echo "no src/*.cpp yet — start with PLAN.md Steps 1.1 onward"; exit 1; \
	fi
	$(CXX) $(CXXFLAGS_REL) $(CPU_OBJ) -o $@

$(OBJDIR)/%.o: src/%.cpp | $(OBJDIR)
	$(CXX) $(CXXFLAGS_REL) -MMD -MP -c $< -o $@

# ------------------------------------------------------ debug / sanitized ---
debug: $(BUILD)/oooc-debug

$(BUILD)/oooc-debug: $(DBG_OBJ) | $(BUILD)
	@if [ -z "$(DBG_OBJ)" ]; then \
	  echo "no src/*.cpp yet — start with PLAN.md Steps 1.1 onward"; exit 1; \
	fi
	$(CXX) $(CXXFLAGS_DBG) $(DBG_OBJ) -o $@ $(LDFLAGS_DBG)

$(DBGDIR)/%.o: src/%.cpp | $(DBGDIR)
	$(CXX) $(CXXFLAGS_DBG) -MMD -MP -c $< -o $@

# --------------------------------------------------------------- tooling ---
$(BUILD)/gen_examples: $(TOOLS_SRC) | $(BUILD)
	$(CXX) $(CXXFLAGS_REL) $(TOOLS_SRC) -o $@

# The generator arrives in Step 8.4. Until then `make test` skips example
# generation instead of failing on a prerequisite that cannot exist yet.
examples:
	@if [ -f $(TOOLS_SRC) ]; then \
	  $(MAKE) --no-print-directory $(BUILD)/gen_examples && \
	  mkdir -p examples && ./$(BUILD)/gen_examples; \
	else \
	  echo "examples: skipped, no $(TOOLS_SRC) yet (PLAN.md Step 8.4)"; \
	fi

# ------------------------------------------------------------------ test ---
$(BUILD)/test_main: $(TEST_SRC) $(LIB_SRC) | $(BUILD)
	@if [ ! -f $(TEST_SRC) ]; then \
	  echo "no $(TEST_SRC) yet — see PLAN.md Step 2.3"; exit 1; \
	fi
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
	@echo "  debug    build/oooc-debug   (ASan + UBSan, -O1 -g)"
	@echo "  test     compile+run tests/test_main after regenerating examples/"
	@echo "  clean    remove build/ and examples/"

# Pick up auto-generated .d dependency fragments.
-include $(CPU_OBJ:.o=.d) $(DBG_OBJ:.o=.d)
