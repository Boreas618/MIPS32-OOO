/* verilator lint_off UNUSEDSIGNAL */
/* verilator lint_off UNUSEDPARAM */
/* verilator lint_off WIDTHEXPAND */
/* verilator lint_off WIDTHTRUNC */
`include "Config.svh"
`include "BranchTypes.svh"

/**
 * Branch Execution Unit - OoO Branch Resolution
 *
 * Executes branch instructions and compares results with predictions:
 * - Computes actual taken/not-taken
 * - Computes actual target address
 * - Compares with prediction from branch predictor
 * - Signals misprediction for recovery
 *
 * Supports all MIPS branch types:
 * - BEQ, BNE: Compare two registers
 * - BGEZ, BLTZ: Compare register with zero
 * - J, JAL: Unconditional jumps
 * - JR: Jump register
 */
module BranchExecute #(
    parameter NUM_PHYS_REGS = 64,
    parameter NUM_BRANCH = 1
) (
    input   logic                           clk,
    input   logic                           rst,

    // Branch inputs from IssueUnit
    input   logic   [NUM_BRANCH-1:0]        branch_valid,
    input   logic   [31:0]                  branch_src1 [NUM_BRANCH-1:0],    // rs value
    input   logic   [31:0]                  branch_src2 [NUM_BRANCH-1:0],    // rt value
    input   logic   [31:0]                  branch_pc [NUM_BRANCH-1:0],
    input   logic   [31:0]                  branch_target [NUM_BRANCH-1:0],  // PC-relative target
    input   logic   [31:0]                  jump_target [NUM_BRANCH-1:0],    // J-type target
    input   logic   [3:0]                   branch_type [NUM_BRANCH-1:0],
    input   logic   [31:0]                  predicted_target [NUM_BRANCH-1:0],
    input   logic   [NUM_BRANCH-1:0]        predicted_taken,
    input   logic   [6:0]                   branch_rob_idx [NUM_BRANCH-1:0],

    // Branch availability
    output  logic   [NUM_BRANCH-1:0]        branch_available,

    // Branch resolution outputs
    output  logic   [NUM_BRANCH-1:0]        branch_result_valid,
    output  logic   [NUM_BRANCH-1:0]        branch_actual_taken,
    output  logic   [31:0]                  branch_actual_target [NUM_BRANCH-1:0],
    output  logic   [NUM_BRANCH-1:0]        branch_mispredicted,
    output  logic   [31:0]                  branch_correct_pc [NUM_BRANCH-1:0],
    output  logic   [6:0]                   branch_result_rob_idx [NUM_BRANCH-1:0],

    // Flush interface
    input   logic                           flush
);

    // Branch unit is always available (1-cycle)
    assign branch_available = {NUM_BRANCH{1'b1}};

    // Intermediate signals for computation
    logic [NUM_BRANCH-1:0] taken;
    logic [31:0] actual_target [NUM_BRANCH-1:0];
    logic [NUM_BRANCH-1:0] mispredict;

    // Branch evaluation - combinational
    always_comb begin
        for (int b = 0; b < NUM_BRANCH; b++) begin
            taken[b] = 1'b0;
            actual_target[b] = branch_pc[b] + 32'd4;  // Default: not taken
            mispredict[b] = 1'b0;

            if (branch_valid[b]) begin
                case (branch_type[b])
                    `BRANCH_TYPE_BEQ: begin
                        // Branch if equal
                        taken[b] = (branch_src1[b] == branch_src2[b]);
                        actual_target[b] = taken[b] ? branch_target[b] : (branch_pc[b] + 32'd4);
                    end
                    
                    `BRANCH_TYPE_BNE: begin
                        // Branch if not equal
                        taken[b] = (branch_src1[b] != branch_src2[b]);
                        actual_target[b] = taken[b] ? branch_target[b] : (branch_pc[b] + 32'd4);
                    end
                    
                    `BRANCH_TYPE_BGEZ: begin
                        // Branch if >= 0 (signed)
                        taken[b] = ($signed(branch_src1[b]) >= 0);
                        actual_target[b] = taken[b] ? branch_target[b] : (branch_pc[b] + 32'd4);
                    end
                    
                    `BRANCH_TYPE_BLTZ: begin
                        // Branch if < 0 (signed)
                        taken[b] = ($signed(branch_src1[b]) < 0);
                        actual_target[b] = taken[b] ? branch_target[b] : (branch_pc[b] + 32'd4);
                    end
                    
                    `BRANCH_TYPE_J: begin
                        // Unconditional jump
                        taken[b] = 1'b1;
                        actual_target[b] = jump_target[b];
                    end
                    
                    `BRANCH_TYPE_JAL: begin
                        // Jump and link
                        taken[b] = 1'b1;
                        actual_target[b] = jump_target[b];
                    end
                    
                    `BRANCH_TYPE_JR: begin
                        // Jump register
                        taken[b] = 1'b1;
                        actual_target[b] = branch_src1[b];
                    end
                    
                    default: begin
                        taken[b] = 1'b0;
                        actual_target[b] = branch_pc[b] + 32'd4;
                    end
                endcase

                // Check for misprediction
                // Misprediction occurs if:
                // 1. Direction mismatch (taken vs predicted_taken)
                // 2. Target mismatch (for taken branches)
                if (taken[b] != predicted_taken[b]) begin
                    mispredict[b] = 1'b1;
                end else if (taken[b] && (actual_target[b] != predicted_target[b])) begin
                    mispredict[b] = 1'b1;
                end
            end
        end
    end

    // Pipeline register for branch results (1-cycle latency)
    always_ff @(posedge clk) begin
        if (rst || flush) begin
            for (int b = 0; b < NUM_BRANCH; b++) begin
                branch_result_valid[b] <= 1'b0;
                branch_actual_taken[b] <= 1'b0;
                branch_actual_target[b] <= '0;
                branch_mispredicted[b] <= 1'b0;
                branch_correct_pc[b] <= '0;
                branch_result_rob_idx[b] <= '0;
            end
        end else begin
            for (int b = 0; b < NUM_BRANCH; b++) begin
                branch_result_valid[b] <= branch_valid[b];
                branch_actual_taken[b] <= taken[b];
                branch_actual_target[b] <= actual_target[b];
                branch_mispredicted[b] <= mispredict[b];
                branch_correct_pc[b] <= actual_target[b];
                branch_result_rob_idx[b] <= branch_rob_idx[b];
            end
        end
    end

endmodule
