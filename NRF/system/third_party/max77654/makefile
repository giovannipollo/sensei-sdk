# Copyright (c) 2022 Philip Schilk
# SPDX-License-Identifier: Apache-2.0

#
# max77654 makefile
# Philipp Schilk 2022
#

# Compiler + Flags
CC=gcc
LDFLAGS=
CFLAGS=-Wall -Wextra -Wpedantic -Werror=vla -fsanitize=address -g -Isrc -Iinc -Itest/Unity -Itest/utils
DEPFLAGS=-MMD -MP -MF $(BUILDDIR)/$*.d

SOURCES=test/Unity/unity.c test/utils/mock_device.c
TEST_SOURCES=$(wildcard test/*.c)
TESTS=$(addprefix $(BUILDDIR)/,$(TEST_SOURCES:.c=.out))
OBJECTS=$(addprefix $(BUILDDIR)/,$(SOURCES:.c=.o))
PREPROC_EXPANDED_SRCS=$(addprefix $(BUILDDIR)/preproc/,$(SOURCES))
PREPROC_EXPANDED_TEST_SRCS=$(addprefix $(BUILDDIR)/preproc/,$(TEST_SOURCES))
DEPENDENCIES=$(addprefix $(BUILDDIR)/,$(SOURCES:.c=.d))
DEPENDENCIES+=$(addprefix $(BUILDDIR)/,$(TEST_SOURCES:.c=.d))

# DEBUG:
# 	@echo SOURCES:
# 	@echo $(SOURCES)
# 	@echo TEST_SOURCES:
# 	@echo $(TEST_SOURCES)

BUILDDIR=build

SILENT?=

.PHONY: all run_tests build_tests clean format

all: run_tests

run_tests: build_tests
	-python3 scripts/run_tests.py $(TESTS)

build_tests: $(TESTS)

clean:
	rm -rf $(BUILDDIR)

format:
	python3 scripts/clang_format.py

# Link tests:
$(BUILDDIR)/%.out: $(BUILDDIR)/%.o $(OBJECTS)
	$(SILENT) $(CC) $(CFLAGS) $^ -o $@

# Compile sources and test sources
$(BUILDDIR)/%.o: %.c makefile
	@mkdir -p $(dir $@)
	$(SILENT) $(CC) -c $(CFLAGS) $(DEPFLAGS) $*.c -o $@

# Generate C files with all preproc expansion:
.PHONY: preproc_expanded
preproc_expanded: $(PREPROC_EXPANDED_SRCS) $(PREPROC_EXPANDED_TEST_SRCS)
$(BUILDDIR)/preproc/%.c: %.c makefile
	@mkdir -p $(dir $@)
	$(SILENT) $(CC) -c $(CFLAGS) -E -C $*.c -o $@

# Re-generate compile_commands.json using either bear or compiledb
# GEN_COMP_COMMANDS_CMD=compiledb make
GEN_COMP_COMMANDS_CMD=bear -- make
compile_commands.json: clean
	$(GEN_COMP_COMMANDS_CMD)

# Documentation:
.PHONY: doc
doc:
	doxygen

# Keep dependencies around, make them an explicit target:
$(DEPENDENCIES):

# Keep object files and output files:
.PRECIOUS: $(BUILDDIR)/%.out
.PRECIOUS: $(BUILDDIR)/%.o

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(MAKECMDGOALS),format)
ifneq ($(MAKECMDGOALS),compile_commands)
include $(DEPENDENCIES)
endif
endif
endif
