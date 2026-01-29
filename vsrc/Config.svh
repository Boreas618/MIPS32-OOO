`ifndef __CONFIG_SVH__
`define __CONFIG_SVH__

`define TEXT_BASE 32'h1000
`define EXIT_ADDR `TEXT_BASE + 32'h8080
`define MAGIC_NUM 32'h12345
`define STACK_BASE 32'h8000

// L1 Cache latency parameters (no L2/L3 - direct to main memory)
`define L1_HIT_LATENCY 1          // 1 cycle for cache hit
`define L1_MISS_LATENCY 20        // 20 cycles for main memory fetch
`define L1_WRITEBACK_LATENCY 10   // 10 cycles for dirty line write-back

`endif
