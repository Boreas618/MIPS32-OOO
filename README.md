# MIPS32 Pipeline

A naive 5-stage pipeline MIPS CPU.

## Quick Start

### macOS

Install the required toolchains using Homebrew:

```shell
brew install verilator llvm@17
echo 'export PATH="/opt/homebrew/opt/llvm@17/bin:$PATH"' >> ~/.zshrc
source ~/.zshrc
```

For better development experience, add the following paths to `includePath` in VSCode (paths may vary based on your installed versions):

```
/opt/homebrew/Cellar/verilator/<version>/share/verilator/include
/opt/homebrew/Cellar/verilator/<version>/share/verilator/include/vltstd
```

### Linux (Ubuntu/Debian)

Install the required toolchains:

```shell
sudo apt install verilator llvm-17
sudo apt install gcc-mips-linux-gnu binutils-mips-linux-gnu
```

## Build and Test

### Build the CPU Simulator

```shell
make all
```

### Build Test Images

Navigate to `./tests` and run:

```shell
cd tests && make all
```

The test Makefiles automatically detect the platform and use the appropriate toolchain:
- **macOS**: Uses LLVM/Clang for cross-compilation
- **Linux**: Uses GCC MIPS cross-compiler

### Run Tests

Run all tests with:

```shell
make test
```

Expected output:

```
[OK]    TEST1-JUMP.
[OK]    TEST2-BITWISE.
[OK]    TEST3-IMM.
[OK]    TEST4-OPs.
[OK]    TEST5-LOAD_STORE.
[OK]    TEST6-BRANCH.
[OK]    TEST7-LUI.
[OK]    TEST8-QSORT.
ACCEPTED.
```

### Debug Mode

To debug the CPU step by step:

```shell
make run
```

Debug commands:
- `n` - Execute next instruction
- `r` - Run until halt or breakpoint
- `b 0x<addr>` - Set breakpoint
- `p` - Print registers
- `q` - Quit

You can change the target image in the Makefile under the root folder.

## Overview

The CPU is built from the following reference architecture, with some fixtures and modifications made by me, including memory access latencies, bitwise operations, data forwarding, and more.

![MIPS-Pipeline](https://p.ipic.vip/bg6ikm.png)

Some details of implementation can be found in the comments from the source code.
