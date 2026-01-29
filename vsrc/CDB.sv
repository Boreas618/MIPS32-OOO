/* verilator lint_off UNUSEDSIGNAL */
/* verilator lint_off UNUSEDPARAM */
/* verilator lint_off WIDTHEXPAND */
/* verilator lint_off WIDTHTRUNC */
`include "Config.svh"

/**
 * Common Data Bus (CDB) - Result Broadcast Network
 *
 * Arbitrates and broadcasts execution results to:
 * - Reservation stations (wake-up waiting instructions)
 * - Reorder buffer (mark instructions complete)
 * - Physical register file (write result)
 *
 * Configuration:
 * - 4-wide broadcast (2 ALUs + 1 MUL + 1 MEM/DIV)
 * - Priority: ALU > MUL > DIV > MEM
 * - Stalls lower-priority producers if CDB is full
 */
module CDB #(
    parameter NUM_PHYS_REGS = 64,
    parameter CDB_WIDTH = 4,                 // Number of CDB broadcast slots
    parameter NUM_ALU = 2,
    parameter NUM_MEM = 1,
    parameter NUM_BRANCH = 1
) (
    input   logic                           clk,
    input   logic                           rst,

    // ALU results
    input   logic   [NUM_ALU-1:0]           alu_result_valid,
    input   logic   [$clog2(NUM_PHYS_REGS)-1:0] alu_result_prd [NUM_ALU-1:0],
    input   logic   [31:0]                  alu_result_data [NUM_ALU-1:0],
    input   logic   [6:0]                   alu_result_rob_idx [NUM_ALU-1:0],
    output  logic   [NUM_ALU-1:0]           alu_result_ack,

    // Multiplier result
    input   logic                           mul_result_valid,
    input   logic   [$clog2(NUM_PHYS_REGS)-1:0] mul_result_prd,
    input   logic   [31:0]                  mul_result_data,
    input   logic   [6:0]                   mul_result_rob_idx,
    output  logic                           mul_result_ack,

    // Divider result
    input   logic                           div_result_valid,
    input   logic   [$clog2(NUM_PHYS_REGS)-1:0] div_result_prd,
    input   logic   [31:0]                  div_result_data,
    input   logic   [6:0]                   div_result_rob_idx,
    output  logic                           div_result_ack,

    // Memory result (loads)
    input   logic   [NUM_MEM-1:0]           mem_result_valid,
    input   logic   [$clog2(NUM_PHYS_REGS)-1:0] mem_result_prd [NUM_MEM-1:0],
    input   logic   [31:0]                  mem_result_data [NUM_MEM-1:0],
    input   logic   [6:0]                   mem_result_rob_idx [NUM_MEM-1:0],
    output  logic   [NUM_MEM-1:0]           mem_result_ack,

    // Branch results (for ROB completion, no data to broadcast)
    input   logic   [NUM_BRANCH-1:0]        branch_result_valid,
    input   logic   [NUM_BRANCH-1:0]        branch_mispredicted,
    input   logic   [31:0]                  branch_correct_pc [NUM_BRANCH-1:0],
    input   logic   [6:0]                   branch_result_rob_idx [NUM_BRANCH-1:0],
    output  logic   [NUM_BRANCH-1:0]        branch_result_ack,

    // CDB broadcast outputs
    output  logic   [CDB_WIDTH-1:0]         cdb_valid,
    output  logic   [$clog2(NUM_PHYS_REGS)-1:0] cdb_tag [CDB_WIDTH-1:0],
    output  logic   [31:0]                  cdb_data [CDB_WIDTH-1:0],
    output  logic   [6:0]                   cdb_rob_idx [CDB_WIDTH-1:0],

    // Branch completion signals (separate from data CDB)
    output  logic   [NUM_BRANCH-1:0]        cdb_branch_complete,
    output  logic   [NUM_BRANCH-1:0]        cdb_branch_mispredicted,
    output  logic   [31:0]                  cdb_branch_correct_pc [NUM_BRANCH-1:0],
    output  logic   [6:0]                   cdb_branch_rob_idx [NUM_BRANCH-1:0],

    // Flush interface
    input   logic                           flush
);

    localparam PREG_BITS = $clog2(NUM_PHYS_REGS);

    // Arbitration: assign results to CDB slots
    // Priority: ALU0 > ALU1 > MUL > DIV > MEM

    always_comb begin
        // Initialize outputs
        cdb_valid = '0;
        for (int i = 0; i < CDB_WIDTH; i++) begin
            cdb_tag[i] = '0;
            cdb_data[i] = '0;
            cdb_rob_idx[i] = '0;
        end
        
        alu_result_ack = '0;
        mul_result_ack = 1'b0;
        div_result_ack = 1'b0;
        mem_result_ack = '0;
        branch_result_ack = '0;

        // Simple priority-based allocation
        // Slot 0: ALU0 if valid
        if (NUM_ALU > 0 && alu_result_valid[0]) begin
            cdb_valid[0] = 1'b1;
            cdb_tag[0] = alu_result_prd[0];
            cdb_data[0] = alu_result_data[0];
            cdb_rob_idx[0] = alu_result_rob_idx[0];
            alu_result_ack[0] = 1'b1;
        end

        // Slot 1: ALU1 if valid
        if (NUM_ALU > 1 && alu_result_valid[1]) begin
            cdb_valid[1] = 1'b1;
            cdb_tag[1] = alu_result_prd[1];
            cdb_data[1] = alu_result_data[1];
            cdb_rob_idx[1] = alu_result_rob_idx[1];
            alu_result_ack[1] = 1'b1;
        end

        // Slot 2: MUL if valid
        if (mul_result_valid) begin
            cdb_valid[2] = 1'b1;
            cdb_tag[2] = mul_result_prd;
            cdb_data[2] = mul_result_data;
            cdb_rob_idx[2] = mul_result_rob_idx;
            mul_result_ack = 1'b1;
        end else if (div_result_valid) begin
            // DIV uses slot 2 if MUL not using it
            cdb_valid[2] = 1'b1;
            cdb_tag[2] = div_result_prd;
            cdb_data[2] = div_result_data;
            cdb_rob_idx[2] = div_result_rob_idx;
            div_result_ack = 1'b1;
        end

        // Slot 3: MEM or DIV overflow
        if (NUM_MEM > 0 && mem_result_valid[0]) begin
            cdb_valid[3] = 1'b1;
            cdb_tag[3] = mem_result_prd[0];
            cdb_data[3] = mem_result_data[0];
            cdb_rob_idx[3] = mem_result_rob_idx[0];
            mem_result_ack[0] = 1'b1;
        end else if (div_result_valid && !div_result_ack) begin
            // DIV uses slot 3 if slot 2 was used by MUL
            cdb_valid[3] = 1'b1;
            cdb_tag[3] = div_result_prd;
            cdb_data[3] = div_result_data;
            cdb_rob_idx[3] = div_result_rob_idx;
            div_result_ack = 1'b1;
        end

        // Branch completion (separate channel, always ack)
        for (int b = 0; b < NUM_BRANCH; b++) begin
            branch_result_ack[b] = branch_result_valid[b];
            cdb_branch_complete[b] = branch_result_valid[b];
            cdb_branch_mispredicted[b] = branch_result_valid[b] && branch_mispredicted[b];
            cdb_branch_correct_pc[b] = branch_correct_pc[b];
            cdb_branch_rob_idx[b] = branch_result_rob_idx[b];
        end
    end

endmodule
