# Makefile — Build libreachy_codec shared library for pytest/ctypes testing.
#
# Usage:
#   make              # build the shared library
#   make clean        # remove build artifacts
#
# The shared library is built from reachy_codec_shim.c which wraps the
# static inline functions in reachy_codec.h into real exported symbols.

CC       ?= cc
CFLAGS   := -std=c11 -Wall -Wextra -O2 -fPIC

# Auto-detect platform for shared library extension
UNAME_S := $(shell uname -s)
ifeq ($(UNAME_S),Darwin)
  LIB_EXT := dylib
  LDFLAGS := -shared -dynamiclib
else
  LIB_EXT := so
  LDFLAGS := -shared
endif

TARGET := libreachy_codec.$(LIB_EXT)
SRC    := reachy_codec_shim.c

.PHONY: all clean

all: $(TARGET)

$(TARGET): $(SRC) ../arduino/reachy-sensor/reachy_codec.h
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $(SRC) -lm

clean:
	rm -f libreachy_codec.so libreachy_codec.dylib
