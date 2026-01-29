# MIPS32-OOO

A MIPS32 CPU with out-of-order execution, branch prediction, and L1 cache, implemented in SystemVerilog.

## Features

- **Out-of-Order Execution**
  - Tomasulo-style reservation stations
  - Reorder buffer (ROB) for in-order commit
  - Register renaming with physical register file
  - Common data bus (CDB) for result broadcast

- **Branch Prediction**
  - Branch target buffer (BTB)
  - 2-bit saturating counter predictor
  - Return address stack (RAS)

- **Memory Subsystem**
  - L1 instruction cache
  - L1 data cache (write-back)
  - Load/store queues with memory disambiguation
  - Store-to-load forwarding

- **Recovery**
  - Precise exception handling
  - Branch misprediction recovery
  - Memory violation recovery

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
[OK]    TEST1-JUMP
[OK]    TEST2-BITWISE
[OK]    TEST3-IMM
[OK]    TEST4-OPs
[OK]    TEST5-LOAD_STORE
[OK]    TEST6-BRANCH
[OK]    TEST7-LUI
[OK]    TEST8-QSORT
[OK]    TEST9-OOO_DEPS
[OK]    TEST10-OOO_MEM
[OK]    TEST11-OOO_BRANCH

ACCEPTED.
```

The test harness also reports branch prediction statistics for each test.

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

## Architecture

### Pipeline Overview

The CPU implements a superscalar out-of-order pipeline with the following stages:

1. **Fetch** - Instruction fetch with branch prediction (BTB + RAS)
2. **Decode** - Instruction decode and register renaming
3. **Issue** - Dynamic scheduling via reservation stations
4. **Execute** - Out-of-order execution with multiple functional units
5. **Memory** - Load/store with disambiguation
6. **Commit** - In-order retirement via reorder buffer

### Key Modules

| Module | Description |
|--------|-------------|
| `FetchUnit.sv` | Instruction fetch with branch prediction |
| `DecodeUnit.sv` | Decode and dispatch logic |
| `RenameUnit.sv` | Register renaming (RAT + free list) |
| `ReservationStation.sv` | Tomasulo-style issue queue |
| `IssueUnit.sv` | Dynamic instruction scheduling |
| `IntegerUnits.sv` | ALU and branch execution units |
| `ROB.sv` | 64-entry reorder buffer |
| `LoadQueue.sv` / `StoreQueue.sv` | Memory ordering |
| `MemoryDisambiguation.sv` | Load/store forwarding |
| `L1ICache.sv` / `L1DCache.sv` | L1 caches |
| `BranchPredictor.sv` | 2-bit predictor with BTB |
| `RecoveryUnit.sv` | Misprediction/exception recovery |

### Reference

The base pipeline design is derived from classic MIPS architecture references:

![MIPS-Pipeline](https://p.ipic.vip/bg6ikm.png)
