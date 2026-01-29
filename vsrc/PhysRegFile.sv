/* verilator lint_off UNUSEDSIGNAL */
/* verilator lint_off WIDTHTRUNC */
/* verilator lint_off UNUSEDPARAM */
`include "Config.svh"

/**
 * Physical Register File - Extended for Out-of-Order Execution
 *
 * Expands from 32 architectural registers to 64-96 physical registers.
 * Supports multiple read and write ports for superscalar execution.
 *
 * Configuration:
 * - 64 physical registers (expandable to 96)
 * - 8 read ports (2 per instruction × 4-wide)
 * - 4 write ports (1 per instruction × 4-wide)
 * - Ready bits per register for wake-up logic
 */
module PhysRegFile #(
    parameter NUM_PHYS_REGS = 64,           // Total physical registers
    parameter NUM_ARCH_REGS = 32,           // Architectural registers (MIPS)
    parameter READ_PORTS = 8,               // Number of read ports
    parameter WRITE_PORTS = 4               // Number of write ports
) (
    input   logic                           clk,
    input   logic                           rst,

    // Read ports - combinational read
    input   logic   [$clog2(NUM_PHYS_REGS)-1:0] read_addr [READ_PORTS-1:0],
    output  logic   [31:0]                  read_data [READ_PORTS-1:0],
    output  logic   [READ_PORTS-1:0]        read_ready, // Source operand ready

    // Write ports - sequential write
    input   logic   [WRITE_PORTS-1:0]       write_en,
    input   logic   [$clog2(NUM_PHYS_REGS)-1:0] write_addr [WRITE_PORTS-1:0],
    input   logic   [31:0]                  write_data [WRITE_PORTS-1:0],

    // Ready bit management
    input   logic   [WRITE_PORTS-1:0]       set_ready,  // Set ready bit on write completion
    input   logic   [$clog2(NUM_PHYS_REGS)-1:0] clear_ready_addr [WRITE_PORTS-1:0],
    input   logic   [WRITE_PORTS-1:0]       clear_ready, // Clear ready bit on allocation

    // Bypass network - check if any write port has matching address
    output  logic   [31:0]                  bypass_data [READ_PORTS-1:0],
    output  logic   [READ_PORTS-1:0]        bypass_valid,

    // Debug/DPI interface for architectural register viewing
    output  logic   [31:0]                  arch_regs [NUM_ARCH_REGS-1:0],
    output  logic                           magic
);

    localparam PREG_BITS = $clog2(NUM_PHYS_REGS);

    // Register storage
    logic [31:0] regs [NUM_PHYS_REGS-1:0];
    logic [NUM_PHYS_REGS-1:0] ready_bits;

    // Magic signal for test framework (check if $v0 (r2) contains MAGIC_NUM)
    // In OoO mode, we need to track which physical register maps to $v0
    assign magic = (arch_regs[2] == `MAGIC_NUM);

    // Combinational read with bypass
    always_comb begin
        for (int r = 0; r < READ_PORTS; r++) begin
            // Default: read from register file
            read_data[r] = regs[read_addr[r]];
            read_ready[r] = ready_bits[read_addr[r]];
            bypass_valid[r] = 1'b0;
            bypass_data[r] = 32'b0;

            // Check for bypass from write ports (same-cycle forwarding)
            for (int w = 0; w < WRITE_PORTS; w++) begin
                if (write_en[w] && (write_addr[w] == read_addr[r])) begin
                    bypass_data[r] = write_data[w];
                    bypass_valid[r] = 1'b1;
                    read_data[r] = write_data[w]; // Also update read data
                    read_ready[r] = 1'b1;         // Data is ready via bypass
                end
            end

            // Register 0 is always 0 and ready (maps to physical reg 0)
            if (read_addr[r] == '0) begin
                read_data[r] = 32'b0;
                read_ready[r] = 1'b1;
                bypass_data[r] = 32'b0;
                bypass_valid[r] = 1'b0;
            end
        end
    end

    // Expose architectural registers (physical regs 0-31 at reset)
    // In full OoO, this would need RAT lookup
    always_comb begin
        for (int i = 0; i < NUM_ARCH_REGS; i++) begin
            arch_regs[i] = regs[i];
        end
    end

    // Sequential write and ready bit management
    always_ff @(posedge clk) begin
        if (rst) begin
            // Initialize all registers to 0
            for (int i = 0; i < NUM_PHYS_REGS; i++) begin
                regs[i] <= 32'b0;
            end

            // Set up initial architectural state
            regs[29] <= `STACK_BASE;  // $sp
            regs[31] <= `EXIT_ADDR;   // $ra

            // Mark architectural registers (0-31) as ready
            // Physical registers 32-63 start as not ready (will be allocated)
            ready_bits <= {{(NUM_PHYS_REGS-NUM_ARCH_REGS){1'b0}}, {NUM_ARCH_REGS{1'b1}}};

            // Register 0 is always ready (hardwired to 0)
            ready_bits[0] <= 1'b1;
        end else begin
            // Process writes
            for (int w = 0; w < WRITE_PORTS; w++) begin
                if (write_en[w] && write_addr[w] != '0) begin
                    regs[write_addr[w]] <= write_data[w];
                end
            end

            // Set ready bits on write completion
            for (int w = 0; w < WRITE_PORTS; w++) begin
                if (set_ready[w]) begin
                    ready_bits[write_addr[w]] <= 1'b1;
                end
            end

            // Clear ready bits on allocation (new destination)
            for (int w = 0; w < WRITE_PORTS; w++) begin
                if (clear_ready[w] && clear_ready_addr[w] != '0) begin
                    ready_bits[clear_ready_addr[w]] <= 1'b0;
                end
            end

            // Register 0 is always 0 and ready
            regs[0] <= 32'b0;
            ready_bits[0] <= 1'b1;
        end
    end

endmodule
