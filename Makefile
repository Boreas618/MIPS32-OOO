default: all

# ARCH can be dummy, mips or loongarch
ARCH = mips
IMG = $(abspath ./tests/load_store/load_store.bin)

ROOT = $(abspath .)
CSRCS = $(shell find $(ROOT)/csrc -name "*.c" -o -name "*.cpp" -o -name "*.cc")
VSRCS = $(shell find $(ROOT)/vsrc -name "*.v" -o -name "*.sv")
TOPNAME = Top
VERILATOR_FLAGS = -Wall -Wno-LATCH --cc --exe --build --trace -O2 -Ivsrc
CFLAGS += -std=c++17 -I$(ROOT)/csrc -I$(ROOT)/csrc/include

# Check if LLVM is available
LLVM_CONFIG := $(shell which llvm-config 2>/dev/null)
ifneq ($(LLVM_CONFIG),)
CFLAGS += -DHAS_LLVM -I$(shell llvm-config --includedir)
LDFLAGS += $(shell llvm-config --ldflags --libs)
endif

BUILD_DIR = $(ROOT)/build
OBJ_DIR = $(BUILD_DIR)/obj_dir
BIN = $(BUILD_DIR)/V$(TOPNAME)
$(shell mkdir -p $(BUILD_DIR))

all: $(BIN)

$(BIN): $(CSRCS) $(VSRCS)
	@verilator $(VERILATOR_FLAGS) \
		--top-module $(TOPNAME) $^ \
		$(addprefix -CFLAGS , $(CFLAGS)) \
		$(addprefix -LDFLAGS , $(LDFLAGS)) \
		-o $(abspath $(BIN)) --Mdir $(OBJ_DIR)

run: $(BIN) $(IMG)
	@ARCH=$(ARCH) $(BIN) --debug $(IMG)

test: $(BIN)
	@ARCH=$(ARCH) python3 ./test.py $(BIN)

clean:
	@$(RM) -rf $(BUILD_DIR)
