/* verilator lint_off UNUSEDSIGNAL */
/* verilator lint_off UNUSEDPARAM */
/* verilator lint_off WIDTHEXPAND */
/* verilator lint_off WIDTHTRUNC */
`include "Config.svh"

/**
 * Bypass Network - Operand Forwarding
 *
 * Provides forwarding paths for back-to-back dependent operations.
 * Sources data from:
 * - CDB broadcasts (results from just-completed instructions)
 * - Execution unit outputs (for same-cycle forwarding)
 *
 * Destinations:
 * - Reservation station dispatch (for newly dispatched instructions)
 * - Issue stage (for instructions about to execute)
 */
module BypassNetwork #(
    parameter NUM_PHYS_REGS = 64,
    parameter CDB_WIDTH = 4,
    parameter NUM_ALU = 2,
    parameter FORWARD_PORTS = 8              // Number of operand requests to handle
) (
    input   logic                           clk,
    input   logic                           rst,

    // Operand requests (from dispatch or issue)
    input   logic   [$clog2(NUM_PHYS_REGS)-1:0] req_tag [FORWARD_PORTS-1:0],
    input   logic   [FORWARD_PORTS-1:0]     req_valid,
    
    // CDB broadcast (for forwarding)
    input   logic   [CDB_WIDTH-1:0]         cdb_valid,
    input   logic   [$clog2(NUM_PHYS_REGS)-1:0] cdb_tag [CDB_WIDTH-1:0],
    input   logic   [31:0]                  cdb_data [CDB_WIDTH-1:0],

    // ALU outputs (for same-cycle forwarding from just-executed)
    input   logic   [NUM_ALU-1:0]           alu_valid,
    input   logic   [$clog2(NUM_PHYS_REGS)-1:0] alu_prd [NUM_ALU-1:0],
    input   logic   [31:0]                  alu_data [NUM_ALU-1:0],

    // Physical register file read (fallback)
    input   logic   [31:0]                  prf_read_data [FORWARD_PORTS-1:0],
    input   logic   [FORWARD_PORTS-1:0]     prf_read_ready,

    // Forwarded outputs
    output  logic   [31:0]                  fwd_data [FORWARD_PORTS-1:0],
    output  logic   [FORWARD_PORTS-1:0]     fwd_valid
);

    localparam PREG_BITS = $clog2(NUM_PHYS_REGS);

    // Bypass logic: check all sources for matching tag
    always_comb begin
        for (int p = 0; p < FORWARD_PORTS; p++) begin
            fwd_data[p] = prf_read_data[p];
            fwd_valid[p] = prf_read_ready[p];

            if (req_valid[p]) begin
                // Check CDB for matching tag (highest priority - just completed)
                for (int c = 0; c < CDB_WIDTH; c++) begin
                    if (cdb_valid[c] && cdb_tag[c] == req_tag[p]) begin
                        fwd_data[p] = cdb_data[c];
                        fwd_valid[p] = 1'b1;
                    end
                end

                // Check ALU outputs (for pipeline bypass)
                for (int a = 0; a < NUM_ALU; a++) begin
                    if (alu_valid[a] && alu_prd[a] == req_tag[p]) begin
                        fwd_data[p] = alu_data[a];
                        fwd_valid[p] = 1'b1;
                    end
                end

                // Physical register 0 is always 0 and valid
                if (req_tag[p] == '0) begin
                    fwd_data[p] = 32'b0;
                    fwd_valid[p] = 1'b1;
                end
            end
        end
    end

endmodule
