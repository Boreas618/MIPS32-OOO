# Common build configuration for MIPS test cases
# Supports both Linux (gcc) and macOS (LLVM)

UNAME_S := $(shell uname -s)

ifeq ($(UNAME_S),Darwin)
    # macOS: Use LLVM toolchain
    LLVM_PATH ?= /opt/homebrew/opt/llvm@17/bin
    CC = $(LLVM_PATH)/clang
    LD = $(LLVM_PATH)/ld.lld
    OBJCOPY = $(LLVM_PATH)/llvm-objcopy
    CFLAGS = --target=mipsel-unknown-linux-gnu -c -I../
    LDFLAGS = -m elf32ltsmip -e _start --strip-all
    OBJCOPY_FLAGS = -O binary -j .text
else
    # Linux: Use GCC cross-compiler
    CC = mips-linux-gnu-gcc
    LD = /usr/mips-linux-gnu/bin/ld
    OBJCOPY = /usr/bin/mips-linux-gnu-objcopy
    CFLAGS = -O0 -c -EL -nostdlib -I../
    LDFLAGS = -EL -static
    OBJCOPY_FLAGS = -O binary
endif
