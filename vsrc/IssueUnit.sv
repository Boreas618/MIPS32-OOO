/* verilator lint_off UNUSEDSIGNAL */
/* verilator lint_off UNUSEDPARAM */
/* verilator lint_off WIDTHEXPAND */
/* verilator lint_off WIDTHTRUNC */
`include "Config.svh"
`include "BranchTypes.svh"

/**
 * Issue Unit - Instruction Selection and Dispatch to Functional Units
 *
 * Coordinates instruction issue from multiple reservation stations:
 * - Integer RS (ALU operations)
 * - Memory RS (loads/stores) 
 * - Branch RS (branches)
 *
 * Features:
 * - Oldest-first selection from each RS
 * - Structural hazard checking (FU availability)
 * - Multi-issue (2-4 instructions per cycle)
 * - Handles multi-cycle operations
 */
module IssueUnit #(
    parameter NUM_PHYS_REGS = 64,
    parameter RS_ENTRIES = 16,
    parameter ISSUE_WIDTH = 4,               // Max instructions issued per cycle
    parameter NUM_ALU = 2,                   // Number of ALU units
    parameter NUM_MEM = 1,                   // Number of memory units
    parameter NUM_BRANCH = 1                 // Number of branch units
) (
    input   logic                           clk,
    input   logic                           rst,

    // Integer RS issue interface
    input   logic   [ISSUE_WIDTH-1:0]       int_rs_valid,
    input   logic   [$clog2(NUM_PHYS_REGS)-1:0] int_rs_prd [ISSUE_WIDTH-1:0],
    input   logic   [31:0]                  int_rs_src1 [ISSUE_WIDTH-1:0],
    input   logic   [31:0]                  int_rs_src2 [ISSUE_WIDTH-1:0],
    input   logic   [3:0]                   int_rs_alu_control [ISSUE_WIDTH-1:0],
    input   logic   [1:0]                   int_rs_alu_src [ISSUE_WIDTH-1:0],
    input   logic   [31:0]                  int_rs_imm [ISSUE_WIDTH-1:0],
    input   logic   [4:0]                   int_rs_shamt [ISSUE_WIDTH-1:0],
    input   logic   [ISSUE_WIDTH-1:0]       int_rs_reg_write,
    input   logic   [ISSUE_WIDTH-1:0]       int_rs_mem_to_reg,
    input   logic   [ISSUE_WIDTH-1:0]       int_rs_mem_write,
    input   logic   [ISSUE_WIDTH-1:0]       int_rs_mem_read,
    input   logic   [ISSUE_WIDTH-1:0]       int_rs_branch,
    input   logic   [31:0]                  int_rs_pc [ISSUE_WIDTH-1:0],
    input   logic   [31:0]                  int_rs_pc_plus_4 [ISSUE_WIDTH-1:0],
    input   logic   [31:0]                  int_rs_branch_target [ISSUE_WIDTH-1:0],
    input   logic   [31:0]                  int_rs_jump_target [ISSUE_WIDTH-1:0],
    input   logic   [3:0]                   int_rs_branch_type [ISSUE_WIDTH-1:0],
    input   logic   [31:0]                  int_rs_predicted_target [ISSUE_WIDTH-1:0],
    input   logic   [ISSUE_WIDTH-1:0]       int_rs_predicted_taken,
    input   logic   [6:0]                   int_rs_rob_idx [ISSUE_WIDTH-1:0],
    input   logic   [$clog2(RS_ENTRIES)-1:0] int_rs_idx [ISSUE_WIDTH-1:0],
    output  logic   [ISSUE_WIDTH-1:0]       int_rs_ack,

    // Functional unit availability (from execution units)
    input   logic   [NUM_ALU-1:0]           alu_available,
    input   logic   [NUM_MEM-1:0]           mem_available,
    input   logic   [NUM_BRANCH-1:0]        branch_available,

    // Issue to ALU units
    output  logic   [NUM_ALU-1:0]           alu_issue_valid,
    output  logic   [$clog2(NUM_PHYS_REGS)-1:0] alu_issue_prd [NUM_ALU-1:0],
    output  logic   [31:0]                  alu_issue_src1 [NUM_ALU-1:0],
    output  logic   [31:0]                  alu_issue_src2 [NUM_ALU-1:0],
    output  logic   [3:0]                   alu_issue_alu_control [NUM_ALU-1:0],
    output  logic   [1:0]                   alu_issue_alu_src [NUM_ALU-1:0],
    output  logic   [31:0]                  alu_issue_imm [NUM_ALU-1:0],
    output  logic   [4:0]                   alu_issue_shamt [NUM_ALU-1:0],
    output  logic   [NUM_ALU-1:0]           alu_issue_reg_write,
    output  logic   [31:0]                  alu_issue_pc [NUM_ALU-1:0],
    output  logic   [31:0]                  alu_issue_pc_plus_4 [NUM_ALU-1:0],
    output  logic   [6:0]                   alu_issue_rob_idx [NUM_ALU-1:0],

    // Issue to memory unit
    output  logic   [NUM_MEM-1:0]           mem_issue_valid,
    output  logic   [$clog2(NUM_PHYS_REGS)-1:0] mem_issue_prd [NUM_MEM-1:0],
    output  logic   [31:0]                  mem_issue_addr [NUM_MEM-1:0],
    output  logic   [31:0]                  mem_issue_data [NUM_MEM-1:0],
    output  logic   [NUM_MEM-1:0]           mem_issue_mem_write,
    output  logic   [NUM_MEM-1:0]           mem_issue_mem_read,
    output  logic   [6:0]                   mem_issue_rob_idx [NUM_MEM-1:0],

    // Issue to branch unit
    output  logic   [NUM_BRANCH-1:0]        branch_issue_valid,
    output  logic   [31:0]                  branch_issue_src1 [NUM_BRANCH-1:0],
    output  logic   [31:0]                  branch_issue_src2 [NUM_BRANCH-1:0],
    output  logic   [31:0]                  branch_issue_pc [NUM_BRANCH-1:0],
    output  logic   [31:0]                  branch_issue_branch_target [NUM_BRANCH-1:0],
    output  logic   [31:0]                  branch_issue_jump_target [NUM_BRANCH-1:0],
    output  logic   [3:0]                   branch_issue_branch_type [NUM_BRANCH-1:0],
    output  logic   [31:0]                  branch_issue_predicted_target [NUM_BRANCH-1:0],
    output  logic   [NUM_BRANCH-1:0]        branch_issue_predicted_taken,
    output  logic   [6:0]                   branch_issue_rob_idx [NUM_BRANCH-1:0],

    // Flush interface
    input   logic                           flush
);

    localparam PREG_BITS = $clog2(NUM_PHYS_REGS);

    // Classification signals
    logic [ISSUE_WIDTH-1:0] is_alu_op;
    logic [ISSUE_WIDTH-1:0] is_mem_op;
    logic [ISSUE_WIDTH-1:0] is_branch_op;

    // Classify operations
    always_comb begin
        for (int i = 0; i < ISSUE_WIDTH; i++) begin
            // Memory operations (load or store)
            is_mem_op[i] = int_rs_valid[i] && (int_rs_mem_read[i] || int_rs_mem_write[i]);

            // Branch operations - but NOT JAL which needs to go to ALU to write PC+8 to $ra
            // JAL has both branch=1 and reg_write=1; regular branches have branch=1 but reg_write=0
            // J (unconditional jump without link) has branch=1 and reg_write=0, so it goes to BranchExecute
            is_branch_op[i] = int_rs_valid[i] && int_rs_branch[i] && !int_rs_mem_read[i] &&
                              !int_rs_mem_write[i] && !int_rs_reg_write[i];

            // ALU operations (everything else that's valid, including JAL which has reg_write=1)
            is_alu_op[i] = int_rs_valid[i] && !is_mem_op[i] && !is_branch_op[i];
        end
    end

    // Track which units are still available for this cycle
    logic [NUM_ALU-1:0] alu_avail;
    logic [NUM_MEM-1:0] mem_avail;
    logic [NUM_BRANCH-1:0] branch_avail;

    // Issue selection: assign operations to available units
    always_comb begin
        // Initialize outputs
        int_rs_ack = '0;
        
        for (int a = 0; a < NUM_ALU; a++) begin
            alu_issue_valid[a] = 1'b0;
            alu_issue_prd[a] = '0;
            alu_issue_src1[a] = '0;
            alu_issue_src2[a] = '0;
            alu_issue_alu_control[a] = '0;
            alu_issue_alu_src[a] = '0;
            alu_issue_imm[a] = '0;
            alu_issue_shamt[a] = '0;
            alu_issue_reg_write[a] = '0;
            alu_issue_pc[a] = '0;
            alu_issue_pc_plus_4[a] = '0;
            alu_issue_rob_idx[a] = '0;
        end
        
        for (int m = 0; m < NUM_MEM; m++) begin
            mem_issue_valid[m] = 1'b0;
            mem_issue_prd[m] = '0;
            mem_issue_addr[m] = '0;
            mem_issue_data[m] = '0;
            mem_issue_mem_write[m] = '0;
            mem_issue_mem_read[m] = '0;
            mem_issue_rob_idx[m] = '0;
        end
        
        for (int b = 0; b < NUM_BRANCH; b++) begin
            branch_issue_valid[b] = 1'b0;
            branch_issue_src1[b] = '0;
            branch_issue_src2[b] = '0;
            branch_issue_pc[b] = '0;
            branch_issue_branch_target[b] = '0;
            branch_issue_jump_target[b] = '0;
            branch_issue_branch_type[b] = '0;
            branch_issue_predicted_target[b] = '0;
            branch_issue_predicted_taken[b] = '0;
            branch_issue_rob_idx[b] = '0;
        end

        alu_avail = alu_available;
        mem_avail = mem_available;
        branch_avail = branch_available;

        // Process each RS entry in priority order (entry 0 is highest priority/oldest)
        for (int i = 0; i < ISSUE_WIDTH; i++) begin
            if (is_alu_op[i]) begin
                // Find available ALU
                for (int a = 0; a < NUM_ALU; a++) begin
                    if (alu_avail[a] && !alu_issue_valid[a]) begin
                        alu_issue_valid[a] = 1'b1;
                        alu_issue_prd[a] = int_rs_prd[i];
                        alu_issue_src1[a] = int_rs_src1[i];
                        alu_issue_src2[a] = int_rs_src2[i];
                        alu_issue_alu_control[a] = int_rs_alu_control[i];
                        alu_issue_alu_src[a] = int_rs_alu_src[i];
                        alu_issue_imm[a] = int_rs_imm[i];
                        alu_issue_shamt[a] = int_rs_shamt[i];
                        alu_issue_reg_write[a] = int_rs_reg_write[i];
                        alu_issue_pc[a] = int_rs_pc[i];
                        alu_issue_pc_plus_4[a] = int_rs_pc_plus_4[i];
                        alu_issue_rob_idx[a] = int_rs_rob_idx[i];
                        int_rs_ack[i] = 1'b1;
                        alu_avail[a] = 1'b0;
                        break;
                    end
                end
            end else if (is_mem_op[i]) begin
                // Find available memory unit
                for (int m = 0; m < NUM_MEM; m++) begin
                    if (mem_avail[m] && !mem_issue_valid[m]) begin
                        mem_issue_valid[m] = 1'b1;
                        mem_issue_prd[m] = int_rs_prd[i];
                        // Address is src1 + imm (base + offset)
                        mem_issue_addr[m] = int_rs_src1[i] + int_rs_imm[i];
                        mem_issue_data[m] = int_rs_src2[i];
                        mem_issue_mem_write[m] = int_rs_mem_write[i];
                        mem_issue_mem_read[m] = int_rs_mem_read[i];
                        mem_issue_rob_idx[m] = int_rs_rob_idx[i];
                        int_rs_ack[i] = 1'b1;
                        mem_avail[m] = 1'b0;
                        break;
                    end
                end
            end else if (is_branch_op[i]) begin
                // Find available branch unit
                for (int b = 0; b < NUM_BRANCH; b++) begin
                    if (branch_avail[b] && !branch_issue_valid[b]) begin
                        branch_issue_valid[b] = 1'b1;
                        branch_issue_src1[b] = int_rs_src1[i];
                        branch_issue_src2[b] = int_rs_src2[i];
                        branch_issue_pc[b] = int_rs_pc[i];
                        branch_issue_branch_target[b] = int_rs_branch_target[i];
                        branch_issue_jump_target[b] = int_rs_jump_target[i];
                        branch_issue_branch_type[b] = int_rs_branch_type[i];
                        branch_issue_predicted_target[b] = int_rs_predicted_target[i];
                        branch_issue_predicted_taken[b] = int_rs_predicted_taken[i];
                        branch_issue_rob_idx[b] = int_rs_rob_idx[i];
                        int_rs_ack[i] = 1'b1;
                        branch_avail[b] = 1'b0;
                        break;
                    end
                end
            end
        end
    end

endmodule
